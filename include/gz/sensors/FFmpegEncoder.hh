#pragma once

#include "opencv2/opencv.hpp"

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/opt.h>
#include <libavutil/hwcontext.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>
}

#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <GLES3/gl31.h>
#include <GLES2/gl2ext.h>
#include <GLES3/gl3ext.h>
#include <va/va.h>
#include <gbm.h>

#include <memory>
#include "std_msgs/msg/header.hpp"
#include <ffmpeg_image_transport_msgs/msg/detail/ffmpeg_packet__struct.hpp>

#include "rclcpp/rclcpp.hpp"
#include <thread>
#include <sdf/sdf.hh>


namespace phntm {
    class FFmpegEncoder {
    public:
        using PacketCallback = std::function<void(std::shared_ptr<ffmpeg_image_transport_msgs::msg::FFMPEGPacket> frame)>;

        FFmpegEncoder(int width, int height, const std::string src_encoding, AVPixelFormat opencv_format, AVPixelFormat codec_input_format, std::string frame_id, std::string topic, std::shared_ptr<rclcpp::Node> node, std::string& hw_device, int thread_count, int gop_size, int bit_rate, PacketCallback callback = nullptr);
        ~FFmpegEncoder();
        
        void encodeFrame(const cv::Mat& raw_frame, std_msgs::msg::Header header);
        bool encodeFrameZeroCopy(uint gl_id, std_msgs::msg::Header header);
        bool checkCompatibility(const int frame_width, const int frame_height, const std::string & frame_encoding) { return frame_width == this->width && frame_height == this->height && frame_encoding == this->src_encoding; };

        static std::string GetGZPixelFormatName(sdf::PixelFormatType pixelFormat) {
            switch (pixelFormat) {
                case sdf::PixelFormatType::UNKNOWN_PIXEL_FORMAT: return "UNKNOWN_PIXEL_FORMAT";
                case sdf::PixelFormatType::L_INT8: return "L_INT8";
                case sdf::PixelFormatType::L_INT16: return "L_INT16";
                case sdf::PixelFormatType::RGB_INT8: return "RGB_INT8";
                case sdf::PixelFormatType::RGBA_INT8: return "RGBA_INT8";
                case sdf::PixelFormatType::BGRA_INT8: return "BGRA_INT8";
                case sdf::PixelFormatType::RGB_INT16: return "RGB_INT16";
                case sdf::PixelFormatType::RGB_INT32: return "RGB_INT32";
                case sdf::PixelFormatType::BGR_INT8: return "BGR_INT8";
                case sdf::PixelFormatType::BGR_INT16: return "BGR_INT16";
                case sdf::PixelFormatType::BGR_INT32: return "BGR_INT32";
                case sdf::PixelFormatType::R_FLOAT16: return "R_FLOAT16";
                case sdf::PixelFormatType::RGB_FLOAT16: return "RGB_FLOAT16";
                case sdf::PixelFormatType::R_FLOAT32: return "R_FLOAT32";
                case sdf::PixelFormatType::RGB_FLOAT32: return "RGB_FLOAT32";
                case sdf::PixelFormatType::BAYER_RGGB8: return "BAYER_RGGB8";
                case sdf::PixelFormatType::BAYER_BGGR8: return "BAYER_BGGR8";
                case sdf::PixelFormatType::BAYER_GBRG8: return "BAYER_GBRG8";
                case sdf::PixelFormatType::BAYER_GRBG8: return "BAYER_GRBG8";
                default: return "unknown";
            }
        };

    private:
        int width, height;
        const int fps = 30;
        std::string src_encoding;
        int64_t pts_counter = 0;
        PacketCallback packet_callback;

        AVFormatContext* fmt_ctx = nullptr;
        AVCodecContext* codec_ctx = nullptr;
        SwsContext* sws_ctx = nullptr;

        AVBufferRef* hw_device_ctx = nullptr;
        AVBufferRef* hw_frames_ctx = nullptr;

        EGLDisplay egl_display;
        EGLContext egl_ctx;
        VADisplay va_display;

        int drm_fd;
        gbm_device* gbm_dev;
        
        VAConfigID va_config;
        VAContextID va_ctx;
        //PFNGLEGLIMAGETARGETTEXTURE2DOESPROC glEGLImageTargetTexture2DOES;
        PFNEGLCREATEIMAGEKHRPROC eglCreateImageKHR = nullptr;
        PFNEGLDESTROYIMAGEKHRPROC eglDestroyImageKHR = nullptr;
        //PFNEGLEXPORTDMABUFIMAGEQUERYMESAPROC eglExportDMABUFImageQueryMESA;
        PFNEGLEXPORTDMABUFIMAGEMESAPROC eglExportDMABUFImageMESA;

        GLuint nv12_conversion_program;

        struct ZeroCopyGPUStructs {
            GLuint y_tex = 0, uv_tex = 0;
            EGLImage egl_image_y;
            EGLint y_stride, y_offset;
            unsigned long y_fd;
            EGLImage egl_image_uv;
            EGLint uv_stride, uv_offset;
            unsigned long uv_fd;
            VASurfaceID va_surface = 0;
            AVFrame* va_frame = nullptr;

        };
        int zero_copy_current = 0;
        int zero_copy_pool_size = 8;
        std::vector<ZeroCopyGPUStructs> zero_copy_gpu_pool;

         
        // EGLImage egl_image_y = nullptr;
        // EGLImage egl_image_uv = nullptr;
        // int y_fd, uv_fd;
        // EGLint y_stride, uv_stride;
        // EGLint y_offset, uv_offset;
        
        // AVFrame* nv12_frame = nullptr;
        // AVBufferRef* hw_frames_ref   = nullptr;
        // AVHWFramesContext* hw_frames_ctx = nullptr;
        
        enum AVHWDeviceType hw_device_type = AV_HWDEVICE_TYPE_NONE;
        
        std::string frame_id, topic;
        std::shared_ptr<rclcpp::Node> node;
        bool running = false;

        struct ScalerRequest {
            cv::Mat raw_frame;
            std_msgs::msg::Header header;
        };

        uint num_frame_buffers = 4;
        uint current_sw_frame_buffer = 0;
        uint current_hw_frame_buffer = 0;

        std::vector<AVFrame*> sw_frame_buffers;
        std::vector<AVFrame*> hw_frame_buffers;

        std::thread scaler_thread;
        
        std::queue<ScalerRequest> scaler_queue;
        std::condition_variable scaler_cv;
        std::mutex scaler_mutex;

        std::thread encoder_thread;
        std::condition_variable encoder_cv;
        std::queue<AVFrame*> encoder_queue;
        std::mutex encoder_mutex;
        
        void sendFrameToEncoder(AVFrame* input_frame);
        void scalerWorker();
        void encoderWorker();
        void flush();

        static std::vector<AVCodecID> encoder_input_logged;

        std::string getThreadId() {
            std::ostringstream oss;
            oss << std::this_thread::get_id();
            return oss.str();
        };

        bool debugTexture(int tex, int tex_width, int tex_height);
        std::string toString() { return "Enc " + this->getThreadId() + " " + this->topic; };
        bool setupZeroCopyConverter();
        bool setupSWScaler(AVPixelFormat opencv_format, AVPixelFormat codec_input_format);
        void cleanupZeroCopyConverter();
        bool loadEGLExtensions();
        // VASurfaceID convertToNV12ZeroCopy(GLuint rgbTextureId);
        VAStatus createVASurfaceFromDMABuf(int dmabuf_fd, int stride, VASurfaceID* surface_id);
        AVFrame* convertRGBToNV12(AVFrame* rgb_frame);

        const std::string RED = "\033[31m";
        const std::string CLR = "\033[0m";

        void log(std::string msg, bool error = false, bool append_endl = true) {
            std::string out_msg = "[" + this->toString() + "] " + msg;
            if (error)  {
                std::cerr << (RED + out_msg + CLR) + (append_endl ? "\n" : "");
                fflush(stderr);
            } else {
                std::cout << out_msg + (append_endl ? "\n" : "");
                fflush(stdout);
            }
        };

        void err(std::string msg, bool append_endl = true) {
            this->log(msg, true, append_endl);
        };
    };

}