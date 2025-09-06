#include "gz/sensors/FFmpegEncoder.hh"
// #include "phntm_bridge/lib.hpp"
// #include "phntm_bridge/const.hpp"

#include <algorithm>
#include <cstddef>
#include <functional>
#include <libavcodec/avcodec.h>
#include <libavcodec/codec.h>
#include <libavutil/pixfmt.h>
#include <libswscale/swscale.h>

#include <mutex>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <string>
#include <stdexcept>

namespace phntm {

    std::vector<AVCodecID> FFmpegEncoder::encoder_input_logged;

    FFmpegEncoder::FFmpegEncoder(int src_frame_width, int src_frame_height, std::string src_frame_encoding, AVPixelFormat opencv_format, AVPixelFormat codec_input_format, std::string output_frame_id, std::string output_topic, std::shared_ptr<rclcpp::Node> ros_node, std::string& hw_device, int thread_count, int gop_size, int bit_rate, PacketCallback callback)
        : width(src_frame_width), height(src_frame_height), src_encoding(src_frame_encoding), packet_callback(callback), frame_id(output_frame_id), topic(output_topic), node(ros_node) {

        // initialize FFmpeg
        avformat_network_init();
        
        // find hardware device type
        if (hw_device == "cuda") {
            this->hw_device_type = av_hwdevice_find_type_by_name("cuda");
        } else if (hw_device == "vaapi") {
            this->hw_device_type = av_hwdevice_find_type_by_name("vaapi");
        } else {
            this->hw_device_type = AV_HWDEVICE_TYPE_NONE;
        }

        // find the H.264 encoder
        const AVCodec* codec = nullptr;
        if (this->hw_device_type != AV_HWDEVICE_TYPE_NONE) {
            if (hw_device == "cuda") {
                RCLCPP_INFO(this->node->get_logger(), "[%s] Setting codec to cuda", this->toString().c_str());
                codec = avcodec_find_encoder_by_name("h264_nvenc"); // NVIDIA
            } else if (hw_device == "vaapi") {
                RCLCPP_INFO(this->node->get_logger(), "[%s] Setting codec to h264_amf", this->toString().c_str());
                codec = avcodec_find_encoder_by_name("h264_amf"); // AMD
                if (!codec) {
                    RCLCPP_INFO(this->node->get_logger(), "[%s] Setting codec to h264_vaapi", this->toString().c_str());
                    codec = avcodec_find_encoder_by_name("h264_vaapi"); // Intel
                }
            }
        }

        if (!codec) {
            codec = avcodec_find_encoder(AV_CODEC_ID_H264); // Fallback to software
            //log("["+this->toString()+"] Warning: Software h.264 encoding selected for " + topic+", this is rather slow and expensive");
            RCLCPP_WARN(this->node->get_logger(), "[%s] Software h.264 encoding selected for %s, this is rather slow and expensive", this->toString().c_str(),  this->topic.c_str());
        }
        
        if (!codec) {
            throw std::runtime_error("["+this->toString()+"] H.264 encoder not found for " + topic);
        }

        // output supported input pixel formats for each codec
        if (std::find(FFmpegEncoder::encoder_input_logged.begin(), FFmpegEncoder::encoder_input_logged.end(), codec->id) == FFmpegEncoder::encoder_input_logged.end()) {

            FFmpegEncoder::encoder_input_logged.push_back(codec->id); //only once
            if (codec->pix_fmts) {
                const enum AVPixelFormat *p = codec->pix_fmts;
                while (*p != AV_PIX_FMT_NONE) {
                    RCLCPP_INFO(this->node->get_logger(), "[AVCodec %s] Supported input pixel format: %s", codec->name, av_get_pix_fmt_name(*p));
                    // Optionally, use av_get_pix_fmt_name(*p) to print the name
                    p++;
                }
            } else {
                RCLCPP_WARN(this->node->get_logger(), "[AVCodec %s] No supported input pixel formats detected!", codec->name);
            }
        }
        
        RCLCPP_INFO(this->node->get_logger(), "[AVCodec] OpenCV conversion format for sw-scaling: %s", av_get_pix_fmt_name(opencv_format));
        RCLCPP_INFO(this->node->get_logger(), "[AVCodec %s] Selected input pixel format: %s", codec->name, av_get_pix_fmt_name(codec_input_format));

        // set up codec context
        this->codec_ctx = avcodec_alloc_context3(codec);
        if (!this->codec_ctx) {
            throw std::runtime_error("["+this->toString()+"] Could not allocate codec context");
        }
        this->codec_ctx->width = width;
        this->codec_ctx->height = height;
        this->codec_ctx->time_base = AVRational{1, fps}; // t
        this->codec_ctx->framerate = AVRational{fps, 1};
        this->codec_ctx->pix_fmt = hw_device_type == AV_HWDEVICE_TYPE_VAAPI ? AV_PIX_FMT_VAAPI : codec_input_format; // this is input to the codec (output of sws_scale)
        this->codec_ctx->gop_size = gop_size; // 60
        this->codec_ctx->max_b_frames = 0;
        this->codec_ctx->thread_count = thread_count;
        this->codec_ctx->bit_rate = bit_rate; // 512 * 1024 * 8; // 0.5 MB/s
        this->codec_ctx->flags &= ~AV_CODEC_FLAG_GLOBAL_HEADER;
        this->codec_ctx->flags |= AV_CODEC_FLAG_LOW_DELAY;     // For real-time
        // this->codec_ctx->flags2 |= AV_CODEC_FLAG2_FAST;        // Faster encoding        // set encoder options
        av_opt_set(this->codec_ctx->priv_data, "preset", "fast", 0);
        if (hw_device_type == AV_HWDEVICE_TYPE_NONE)
            av_opt_set(this->codec_ctx->priv_data, "tune", "zerolatency", 0);
        av_opt_set(this->codec_ctx->priv_data, "profile", "high", 0);

        // initialize sw conversion context
        this->sws_ctx = sws_getContext(width, height,
                                       opencv_format, // OpenCV format
                                       width, height,
                                       codec_input_format, // codec input
                                       SWS_POINT,
                                       nullptr, nullptr,
                                       nullptr);

        // allocate sw frames pool
        for (uint i = 0; i < this->num_frame_buffers; i++) {
            auto sw_frame = av_frame_alloc();
            if (!sw_frame) {
                throw std::runtime_error("["+this->toString()+"] Could not allocate sw frame #" + std::to_string(i) + " for "+topic);
            }
            sw_frame->format = codec_input_format;
            sw_frame->width = width;
            sw_frame->height = height;
            if (av_frame_get_buffer(sw_frame, 0) < 0) {
                throw std::runtime_error("["+this->toString()+"] Could not allocate frame #" + std::to_string(i) + " data for " + topic);
            }
            this->sw_frame_buffers.push_back(sw_frame);
        }

        // create hardware device context
        if (hw_device_type != AV_HWDEVICE_TYPE_NONE) {
            if (hw_device_type == AV_HWDEVICE_TYPE_VAAPI) {
                RCLCPP_INFO(this->node->get_logger(), "[AVCodec] Making hw device ctx for VAAPI");
                if (av_hwdevice_ctx_create(&this->hw_device_ctx, hw_device_type, "/dev/dri/renderD128", nullptr, 0) < 0) {
                   throw std::runtime_error("["+this->toString()+"] Failed to create hardware device context for " + hw_device);
                }
                this->codec_ctx->hw_device_ctx = av_buffer_ref(this->hw_device_ctx);
            } else {
                RCLCPP_INFO(this->node->get_logger(), "[AVCodec] Making hw device ctx");
                if (av_hwdevice_ctx_create(&this->hw_device_ctx, hw_device_type, nullptr, nullptr, 0) < 0) {
                   throw std::runtime_error("["+this->toString()+"] Failed to create hardware device context for " + hw_device);
                }
                this->codec_ctx->hw_device_ctx = av_buffer_ref(this->hw_device_ctx);
            }
        }

        if (hw_device_type == AV_HWDEVICE_TYPE_VAAPI) {
            RCLCPP_INFO(this->node->get_logger(), "[AVCodec %s] Making hw frames ctx", codec->name);
            
            int err = 0;
            AVBufferRef *hw_frames_ref;
            AVHWFramesContext *hw_frames_ctx = NULL;

            if (!(hw_frames_ref = av_hwframe_ctx_alloc(hw_device_ctx))) {
                throw std::runtime_error("Failed to create VAAPI frame context.");
            }
            hw_frames_ctx = (AVHWFramesContext *)(hw_frames_ref->data);
            hw_frames_ctx->format = AV_PIX_FMT_VAAPI;       // Hardware pixel format
            hw_frames_ctx->sw_format = codec_input_format;     // SW pixel format to upload from
            hw_frames_ctx->width = width;
            hw_frames_ctx->height = height;
            hw_frames_ctx->initial_pool_size = this->num_frame_buffers;

            if ((err = av_hwframe_ctx_init(hw_frames_ref)) < 0) {
                av_buffer_unref(&hw_frames_ref);
                throw std::runtime_error("Failed to initialize VAAPI frame context. Error code: " + std::to_string(err));
            }

            this->codec_ctx->hw_frames_ctx = av_buffer_ref(hw_frames_ref);
            if (!this->codec_ctx->hw_frames_ctx) {
                av_buffer_unref(&hw_frames_ref);
                throw std::runtime_error("Failed to allocate codec_ctx->hw_frames_ctx");
            }

            av_buffer_unref(&hw_frames_ref);

            
            // allocate hw frames pool
            for (uint i = 0; i < this->num_frame_buffers; i++) {

                AVFrame * hw_frame;
                
                if (!(hw_frame = av_frame_alloc())) {
                    err = AVERROR(ENOMEM);
                    throw std::runtime_error("Error allocating hw frame " + std::to_string(i));
                }
                if ((err = av_hwframe_get_buffer(this->codec_ctx->hw_frames_ctx, hw_frame, 0)) < 0) {
                    throw std::runtime_error("Error getting buffer for hw frame " + std::to_string(i));
                }
                if (!hw_frame->hw_frames_ctx) {
                    throw std::runtime_error("Error checking allocated hw frame " + std::to_string(i));
                }

                this->hw_frame_buffers.push_back(hw_frame);
            }
        }
        
        // open codec
        if (avcodec_open2(this->codec_ctx, codec, nullptr) < 0) {
            throw std::runtime_error("["+this->toString()+"] Could not open codec for " + topic);
        }
        
        // lanuch workers
        this->running = true;
        this->scaler_thread = std::thread(&FFmpegEncoder::scalerWorker, this);
        this->scaler_thread.detach();
        this->encoder_thread = std::thread(&FFmpegEncoder::encoderWorker, this);
        this->encoder_thread.detach();
    }

    uint64_t convertToRtpTimestamp(int32_t sec, uint32_t nanosec) {
        // Convert to nanoseconds first to avoid floating-point precision loss
        constexpr uint64_t NS_PER_SEC = 1'000'000'000ULL;
        constexpr uint64_t CLOCK_RATE = 90'000ULL; // 90kHz
    
        uint64_t total_ns = static_cast<uint64_t>(sec) * NS_PER_SEC + nanosec;
        uint64_t rtp_timestamp = (total_ns * CLOCK_RATE) / NS_PER_SEC;

        return rtp_timestamp;
    }

    void FFmpegEncoder::encodeFrame(const cv::Mat& raw_frame, std_msgs::msg::Header header) {
    
        if (raw_frame.empty()) {
            throw std::invalid_argument("["+this->toString()+"] Empty frame provided");
        }

        if (!this->running)
            return;

        auto req = ScalerRequest { raw_frame, header };
        {
            std::lock_guard<std::mutex> queue_lock(this->scaler_mutex);
            this->scaler_queue.push(req);
            this->scaler_cv.notify_one();
        }
    }

    void FFmpegEncoder::scalerWorker() {
         std::cout << "["+this->toString()+"] FFmpegEncoder scaler runnig..." << std::endl;

        int err;
        while (this->running) {

            std::unique_lock<std::mutex> scaler_lock(this->scaler_mutex);
            this->scaler_cv.wait(scaler_lock, [this] { return !this->scaler_queue.empty() || !this->running; });

            if (this->scaler_queue.empty() || !this->running) 
                break;
            
            ScalerRequest req;
            while (!this->scaler_queue.empty()) {
                req = this->scaler_queue.front();
                this->scaler_queue.pop();
            }

            scaler_lock.unlock();

            AVFrame* sw_frame = this->sw_frame_buffers[this->current_frame_buffer];
            AVFrame* hw_frame;
            if (hw_device_type == AV_HWDEVICE_TYPE_VAAPI) {
                hw_frame = this->hw_frame_buffers[this->current_frame_buffer];
            }
            
            this->current_frame_buffer++;
            if (this->current_frame_buffer == this->num_frame_buffers) {
                this->current_frame_buffer = 0;
            }
            const uint8_t* src_data[] = { req.raw_frame.data };
            int src_linesize[] = { static_cast<int>(req.raw_frame.step) };
            
            // sw scaling & copy here - expensive
            sws_scale(this->sws_ctx, src_data, src_linesize, 0, this->height, 
                    sw_frame->data, sw_frame->linesize);

            // Send for encoding
            if (hw_device_type == AV_HWDEVICE_TYPE_VAAPI) {
                if ((err = av_hwframe_transfer_data(hw_frame, sw_frame, 0)) < 0) {
                    throw std::runtime_error("Error while transferring frame data to surface. Error code: " + std::to_string(err));
                }
                hw_frame->pts = convertToRtpTimestamp(req.header.stamp.sec, req.header.stamp.nanosec);
                this->sendFrameToEncoder(hw_frame);
            } else {
                sw_frame->pts = convertToRtpTimestamp(req.header.stamp.sec, req.header.stamp.nanosec);
                this->sendFrameToEncoder(sw_frame);
            }
        }

        std::cout << "["+this->toString()+"] FFmpegEncoder scaler stopped..." << std::endl;
    }

    void FFmpegEncoder::sendFrameToEncoder(AVFrame* input_frame) {
        std::lock_guard<std::mutex> queue_lock(this->encoder_mutex);
        this->encoder_queue.push(input_frame);
        this->encoder_cv.notify_one();
    }

    void FFmpegEncoder::flush() {
        sendFrameToEncoder(nullptr);  // Flush the encoder
    }

    void FFmpegEncoder::encoderWorker() {

        std::cout << "["+this->toString()+"] FFmpegEncoder worker runnig..." << std::endl;

        while (this->running) {

            auto pkt = av_packet_alloc();
            if (!pkt) {
                throw std::runtime_error("["+this->toString()+"] Could not allocate packet");
            }

            std::unique_lock<std::mutex> queue_lock(this->encoder_mutex);
            this->encoder_cv.wait(queue_lock, [this] { return !this->encoder_queue.empty() || !this->running; });

            if (this->encoder_queue.empty()) 
                break;

            AVFrame* frame;
            while (!this->encoder_queue.empty()) {
                frame = this->encoder_queue.front();
                this->encoder_queue.pop();
            }

            queue_lock.unlock();

            int ret = avcodec_send_frame(this->codec_ctx, frame);
            if (ret < 0) {
                throw std::runtime_error("["+this->toString()+"] Error sending frame to encoder");
            }

            if (frame == nullptr) { // flushed
                this->running = false;
                break;
            }
                
            // Convert from OpenCV BGR to encoder's format
            ret = avcodec_receive_packet(this->codec_ctx, pkt);

            if (ret == AVERROR(EAGAIN)) { // The encoder needs more input frames before it can output a packet
                // No more packets will be produced (flush completed)
                // log("["+this->toString()+"] Error during receiving frame - EAGAIN");
                av_packet_free(&pkt);
                continue;
            } else if (ret == AVERROR_EOF) {
                std::cout << "["+this->toString()+"] Error during receiving frame - AVERROR_EOF" << std::endl;
                av_packet_free(&pkt);
                this->running = false;
                break;
            } else if (ret < 0) {
                std::cout << "["+this->toString()+"] Error during receiving frame, " + std::to_string(ret) << std::endl;
                av_packet_free(&pkt);
                this->running = false;
                break;
            }

            if (packet_callback) {
                auto msg = std::make_shared<ffmpeg_image_transport_msgs::msg::FFMPEGPacket>();
                msg->header = std_msgs::msg::Header();
                msg->header.frame_id = this->frame_id;
                msg->header.stamp = node->now();
                msg->encoding = "h.264";
                msg->width = width;
                msg->height = height;
                msg->flags = pkt->flags;
                msg->is_bigendian = false;
                msg->pts = frame->pts; //calculated from the initial header stamp
                // frame->data.resize(pkt->size);
                msg->data.assign(pkt->data, pkt->data + pkt->size);
                packet_callback(msg); // produce ros message
            }
            
            av_packet_unref(pkt);

        }    
        std::cout << "["+this->toString()+"] FFmpegEncoder worker finished." << std::endl;
    }

    FFmpegEncoder::~FFmpegEncoder() {

        std::cout << "["+this->toString()+"] Destroying encoder..." << std::endl;

        this->flush(); // flush encoder, kills the thread when complete

        while (this->running) { // flish sets to false
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        this->scaler_cv.notify_one();
        
        std::cout << "["+this->toString()+"] Claning up..." << std::endl;
        // Cleanup
        for (uint i = 0; i < this->num_frame_buffers; i++) {
            if (this->sw_frame_buffers[i])
                av_frame_free(&this->sw_frame_buffers[i]);
            if (this->hw_frame_buffers[i])
                av_frame_free(&this->hw_frame_buffers[i]);
        }
        this->sw_frame_buffers.clear();
        this->hw_frame_buffers.clear();
        
        if (this->codec_ctx) {
            this->codec_ctx->hw_device_ctx = nullptr; //remove this before deallocating
            avcodec_free_context(&this->codec_ctx);
        }
        if (this->hw_device_ctx)
            av_buffer_unref(&this->hw_device_ctx);
        if (this->fmt_ctx)
            avformat_free_context(this->fmt_ctx);
        if (this->sws_ctx)
            sws_freeContext(this->sws_ctx);

        if (this->node.get() != nullptr)
            this->node.reset();

        std::cout << "["+this->toString()+"] Cleanup done." << std::endl;
    }
}