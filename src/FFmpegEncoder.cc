#include "gz/sensors/FFmpegEncoder.hh"
// #include "phntm_bridge/lib.hpp"
// #include "phntm_bridge/const.hpp"


#include <algorithm>
#include <cstddef>
#include <cstdio>
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

#include <fcntl.h>
#include <unistd.h>
#include <va/va.h>
#include <va/va_drm.h>
#include <va/va_drmcommon.h>
#include <gbm.h>
#include <xf86drm.h>

#include <EGL/egl.h>
#include <EGL/eglext.h>

#include <GLES3/gl31.h>
#include <GLES2/gl2ext.h>
#include <GLES3/gl3ext.h>
#include <drm_fourcc.h>

extern "C" {
    #include <libavutil/hwcontext.h>
    #include <libavutil/hwcontext_drm.h>
    #include <libavutil/hwcontext_vaapi.h>
}

namespace phntm {

    std::vector<AVCodecID> FFmpegEncoder::encoder_input_logged;

    // Helper to find DRM render node
    int openDRMRenderNode() {
        const char* base_path = "/dev/dri/renderD";
        for (int i = 128; i < 132; ++i) {
            std::string path = base_path + std::to_string(i);
            int fd = open(path.c_str(), O_RDWR | O_CLOEXEC);
            if (fd >= 0) return fd;
        }
        return -1;
    }

    uint64_t convertToRtpTimestamp(int32_t sec, uint32_t nanosec) {
        // Convert to nanoseconds first to avoid floating-point precision loss
        constexpr uint64_t NS_PER_SEC = 1'000'000'000ULL;
        constexpr uint64_t CLOCK_RATE = 90'000ULL; // 90kHz
    
        uint64_t total_ns = static_cast<uint64_t>(sec) * NS_PER_SEC + nanosec;
        uint64_t rtp_timestamp = (total_ns * CLOCK_RATE) / NS_PER_SEC;

        return rtp_timestamp;
    }

    const char* eglErrorString(EGLint err) {
        switch(err) {
            case EGL_SUCCESS: return "SUCCESS";
            case EGL_BAD_ACCESS: return "BAD_ACCESS";
            case EGL_BAD_ALLOC: return "BAD_ALLOC";
            case EGL_BAD_ATTRIBUTE: return "BAD_ATTRIBUTE";
            case EGL_BAD_CONTEXT: return "BAD_CONTEXT";
            case EGL_BAD_CONFIG: return "BAD_CONFIG";
            case EGL_BAD_DISPLAY: return "BAD_DISPLAY";
            case EGL_BAD_SURFACE: return "BAD_SURFACE";
            case EGL_BAD_MATCH: return "BAD_MATCH";
            case EGL_BAD_PARAMETER: return "BAD_PARAMETER";
            case EGL_BAD_NATIVE_PIXMAP: return "BAD_NATIVE_PIXMAP";
            case EGL_BAD_NATIVE_WINDOW: return "BAD_NATIVE_WINDOW";
            case EGL_CONTEXT_LOST: return "CONTEXT_LOST";
            default: return "UNKNOWN_ERROR";
        }
    }

    FFmpegEncoder::FFmpegEncoder(int src_frame_width, int src_frame_height, std::string src_frame_encoding, AVPixelFormat opencv_format, AVPixelFormat codec_input_format, std::string output_frame_id, std::string output_topic, std::shared_ptr<rclcpp::Node> ros_node, std::string& hw_device, int thread_count, int gop_size, int bit_rate, PacketCallback callback)
        : width(src_frame_width), height(src_frame_height), src_encoding(src_frame_encoding), packet_callback(callback), frame_id(output_frame_id), topic(output_topic), node(ros_node) {

        // initialize FFmpeg
        avformat_network_init();
        
        // find hardware device type
        if (hw_device == "cuda") {
            hw_device_type = av_hwdevice_find_type_by_name("cuda");
        } else if (hw_device == "vaapi") {
            hw_device_type = av_hwdevice_find_type_by_name("vaapi");
        } else {
            hw_device_type = AV_HWDEVICE_TYPE_NONE;
        }

        // find the H.264 encoder
        const AVCodec* codec = nullptr;
        if (hw_device_type != AV_HWDEVICE_TYPE_NONE) {
            if (hw_device == "cuda") {
                RCLCPP_INFO(this->node->get_logger(), "[AVCodec] Setting codec to cuda");
                codec = avcodec_find_encoder_by_name("h264_nvenc"); // NVIDIA
            } else if (hw_device == "vaapi") {
                RCLCPP_INFO(this->node->get_logger(), "[AVCodec] Setting codec to h264_vaapi");
                codec = avcodec_find_encoder_by_name("h264_vaapi"); // Using VAAPI encoder
                if (!codec) {
                    throw std::runtime_error("["+this->toString()+"] h264_vaapi encoder not found");
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
        //this->codec_ctx->flags |= AV_CODEC_FLAG_LOW_DELAY;     // For real-time
        this->codec_ctx->flags2 |= AV_CODEC_FLAG2_FAST;        // Faster encoding        // set encoder options
        av_opt_set(this->codec_ctx->priv_data, "preset", "fast", 0);
        if (hw_device_type == AV_HWDEVICE_TYPE_NONE)
            av_opt_set(this->codec_ctx->priv_data, "tune", "zerolatency", 0);
        av_opt_set(this->codec_ctx->priv_data, "profile", "high", 0);

        // setup sw scaler
        if (hw_device_type != AV_HWDEVICE_TYPE_VAAPI) {
            this->setupSWScaler(opencv_format, codec_input_format);
        }

        // create hardware device context
        if (hw_device_type == AV_HWDEVICE_TYPE_VAAPI) {

            RCLCPP_INFO(this->node->get_logger(), "[AVCodec] Making hw device ctx for VAAPI");
            AVDictionary* opts = nullptr;
            char display_str[32];
            snprintf(display_str, sizeof(display_str), "%p", (void*)this->va_display);
            av_dict_set(&opts, "connection_type", "drm", 0);
            av_dict_set(&opts, "kernel_driver", "amdgpu", 0);
            if (av_hwdevice_ctx_create(&this->hw_device_ctx, hw_device_type, "/dev/dri/renderD128", opts, 0) < 0) {
                av_dict_free(&opts);
                throw std::runtime_error("["+this->toString()+"] Failed to create hardware device context for " + hw_device);
            }
            av_dict_free(&opts);
            this->codec_ctx->hw_device_ctx = av_buffer_ref(this->hw_device_ctx);

            RCLCPP_INFO(this->node->get_logger(), "[AVCodec %s] Making hw frames ctx", codec->name);
            
            int err = 0;
            // AVBufferRef *hw_frames_ref;

            if (!(this->hw_frames_ctx = av_hwframe_ctx_alloc(this->hw_device_ctx))) {
                throw std::runtime_error("Failed to create VAAPI frame context.");
            }
            
            AVHWFramesContext* hw_frames_context_ref = (AVHWFramesContext *)(this->hw_frames_ctx->data);
            hw_frames_context_ref->format = AV_PIX_FMT_VAAPI;       // Hardware pixel format
            hw_frames_context_ref->sw_format = AV_PIX_FMT_NV12;     // SW pixel format to upload from
            hw_frames_context_ref->width = width;
            hw_frames_context_ref->height = height;
            hw_frames_context_ref->initial_pool_size = this->zero_copy_pool_size + 1;

            if ((err = av_hwframe_ctx_init(this->hw_frames_ctx)) < 0) {
                av_buffer_unref(&this->hw_frames_ctx);
                throw std::runtime_error("Failed to initialize VAAPI frame context. Error code: " + std::to_string(err));
            }

            this->log("VAAPI frame context init ok");

            this->codec_ctx->hw_frames_ctx = av_buffer_ref(this->hw_frames_ctx);
            if (!this->codec_ctx->hw_frames_ctx) {
                av_buffer_unref(&this->hw_frames_ctx);
                throw std::runtime_error("Failed to allocate codec_ctx->hw_frames_ctx");
            }

            // av_buffer_unref(&hw_frames_ref);

            
            // allocate hw frames pool
            // for (uint i = 0; i < this->num_frame_buffers; i++) {

            //     AVFrame * hw_frame;
                
            //     if (!(hw_frame = av_frame_alloc())) {
            //         err = AVERROR(ENOMEM);
            //         throw std::runtime_error("Error allocating hw frame " + std::to_string(i));
            //     }
            //     if ((err = av_hwframe_get_buffer(this->codec_ctx->hw_frames_ctx, hw_frame, 0)) < 0) {
            //         throw std::runtime_error("Error getting buffer for hw frame " + std::to_string(i));
            //     }
            //     if (!hw_frame->hw_frames_ctx) {
            //         throw std::runtime_error("Error checking allocated hw frame " + std::to_string(i));
            //     }

            //     this->hw_frame_buffers.push_back(hw_frame);
            // }

        } else if (hw_device_type != AV_HWDEVICE_TYPE_NONE) {

            RCLCPP_INFO(this->node->get_logger(), "[AVCodec] Making hw device ctx");
            if (av_hwdevice_ctx_create(&this->hw_device_ctx, hw_device_type, nullptr, nullptr, 0) < 0) {
                throw std::runtime_error("["+this->toString()+"] Failed to create hardware device context for " + hw_device);
            }
            this->codec_ctx->hw_device_ctx = av_buffer_ref(this->hw_device_ctx);

        }
        
        // open codec
        if (avcodec_open2(this->codec_ctx, codec, nullptr) < 0) {
            throw std::runtime_error("["+this->toString()+"] Could not open codec for " + topic);
        }
        
        // lanuch workers
        this->running = true;
        if (hw_device_type != AV_HWDEVICE_TYPE_VAAPI) {
            this->scaler_thread = std::thread(&FFmpegEncoder::scalerWorker, this);
            this->scaler_thread.detach();
            this->encoder_thread = std::thread(&FFmpegEncoder::encoderWorker, this);
            this->encoder_thread.detach();
        } else {
            if (!this->setupZeroCopyConverter()) {
                throw std::runtime_error( "[AVCodec Failed setting up zero-copy converter");
            }
        }
    }

    // VAStatus FFmpegEncoder::createVASurfaceFromDMABuf(int dmabuf_fd, int stride, VASurfaceID* surface_id) {
    //     VASurfaceAttrib attribs[2];
    //     VASurfaceAttribExternalBuffers external;
        
    //     external.pixel_format = VA_FOURCC_RGBA;
    //     external.width = this->width;
    //     external.height = this->height;
    //     external.data_size = stride * this->height;
    //     external.num_planes = 1;
    //     external.pitches[0] = stride;
    //     external.offsets[0] = 0;
    //     external.buffers = (uintptr_t*)&dmabuf_fd;
    //     external.num_buffers = 1;
    //     external.flags = VA_SURFACE_ATTRIB_MEM_TYPE_DRM_PRIME;
        
    //     attribs[0].type = VASurfaceAttribMemoryType;
    //     attribs[0].flags = VA_SURFACE_ATTRIB_SETTABLE;
    //     attribs[0].value.type = VAGenericValueTypeInteger;
    //     attribs[0].value.value.i = VA_SURFACE_ATTRIB_MEM_TYPE_DRM_PRIME;
        
    //     attribs[1].type = VASurfaceAttribExternalBufferDescriptor;
    //     attribs[1].flags = VA_SURFACE_ATTRIB_SETTABLE;
    //     attribs[1].value.type = VAGenericValueTypePointer;
    //     attribs[1].value.value.p = &external;
        
    //     return vaCreateSurfaces(this->va_display, VA_RT_FORMAT_RGB32, this->width, this->height,
    //                            surface_id, 1, attribs, 2);
    // }

    bool FFmpegEncoder::setupZeroCopyConverter() {

        this->log(">>>> Setting up Zero-copy Converter");

        // DRM Setup
        this->drm_fd = openDRMRenderNode();
        if (this->drm_fd < 0) {
            this->err("Failed to open DRM render node");
            return false;
        }

        // Create GBM device
        this->gbm_dev = gbm_create_device(this->drm_fd);
        if (!this->gbm_dev) {
            this->err("Failed to create GBM device");
            return false;
        }

        // get context and display from ogre2
        this->egl_ctx = eglGetCurrentContext();
        if (this->egl_ctx == EGL_NO_CONTEXT) {
            this->err("Error getting EGL context");
            return false;
        }
        this->egl_display = eglGetCurrentDisplay();
        if (this->egl_display == EGL_NO_DISPLAY) {
            this->err("Error getting EGL display");
            return false;
        }       

        if (!this->loadEGLExtensions())
            return false;

        // VAAPI Setup
        this->va_display = vaGetDisplayDRM(this->drm_fd);
        if (!this->va_display) {
            this->err("Failed to get VAAPI display");
            return false;
        }

        int major_version, minor_version;
        if (vaInitialize(this->va_display, &major_version, &minor_version) != VA_STATUS_SUCCESS) {
            this->err("VAAPI initialization failed");
            return false;
        }
        this->log("VAAPI initializated with v" + std::to_string(major_version) + "." +  std::to_string(minor_version));

        // Create VA encoder configuration
        VAConfigAttrib attribs[1];
        attribs[0].type = VAConfigAttribRateControl;
        vaGetConfigAttributes(this->va_display, VAProfileH264Main, VAEntrypointEncSlice, attribs, 1);
        
        if (!(attribs[0].value & VA_RC_CBR)) {
            this->err("CBR rate control not supported");
            return false;
        }
        
        attribs[0].value = VA_RC_CBR;  // Use CBR mode
        
        if (vaCreateConfig(this->va_display, VAProfileH264Main, VAEntrypointEncSlice, attribs, 1, &this->va_config) != VA_STATUS_SUCCESS) {
            this->err("VAAPI config failed");
            return false;
        }

        // Create compute shader program
        const char* compute_src = R"(
            #version 450
            layout(local_size_x = 16, local_size_y = 16) in;
            
            uniform sampler2D inputTex;

            layout(r8, binding = 1) writeonly uniform image2D y_plane;
            layout(rg8, binding = 2) writeonly uniform image2D uv_plane;

            void main() {
                ivec2 pos = ivec2(gl_GlobalInvocationID.xy);
                
                ivec2 size = textureSize(inputTex, 0);
    
                if (pos.x >= size.x || pos.y >= size.y) return;

                vec3 rgb = texelFetch(inputTex, pos, 0).rgb; // in [0,1]

                rgb = vec3(0.5);

                // BT.709 / linear RGB -> YUV (Y in [0,1], U/V centered at 0.5)
                float Y = 0.2126 * rgb.r + 0.7152 * rgb.g + 0.0722 * rgb.b;
                float U = -0.1146 * rgb.r - 0.3854 * rgb.g + 0.5    * rgb.b; // range roughly [-0.5,0.5]
                float V =  0.5    * rgb.r - 0.4542 * rgb.g - 0.0458 * rgb.b; // range roughly [-0.5,0.5]

                // Convert U/V to unsigned 0..1 where 0.5 == neutral
                U = U + 0.5;
                V = V + 0.5;

                // clamp just in case
                Y = clamp(Y, 0.0, 1.0);
                U = clamp(U, 0.0, 1.0);
                V = clamp(V, 0.0, 1.0);

                // store Y
                imageStore(y_plane, pos, vec4(Y, 0.0, 0.0, 1.0));

                // store UV for even coordinates (NV12 sub-sampling)
                if ((pos.x % 2 == 0) && (pos.y % 2 == 0)) {
                    ivec2 uv_pos = pos / 2;
                    imageStore(uv_plane, uv_pos, vec4(U, V, 0.0, 1.0));
                }
            }
            )";

        GLuint shader = glCreateShader(GL_COMPUTE_SHADER);
        glShaderSource(shader, 1, &compute_src, nullptr);
        glCompileShader(shader);

        GLint shader_status;
        glGetShaderiv(shader, GL_COMPILE_STATUS, &shader_status);
        if (shader_status != GL_TRUE) {
            // Compilation failed; get and print info log
            char buffer[512];
            glGetShaderInfoLog(shader, 512, nullptr, buffer);
            this->err("Shader compile error: " + std::string(buffer));
            return false;
        }

        this->nv12_conversion_program = glCreateProgram();
        glAttachShader(this->nv12_conversion_program, shader);
        glLinkProgram(this->nv12_conversion_program);

        GLenum err;
        std::ostringstream oss; // debugs

        for (int i = 0; i < this->zero_copy_pool_size; i++) {

            ZeroCopyGPUStructs gpu_structs;

            this->log("Initializing zero-copy GPU pool structs " + std::to_string(i));
            std::string l = "GPU pool " + std::to_string(i) + ": ";

            this->log(l + "Making Y texture");
            // Y plane: R8 format at full resolution
            glGenTextures(1, &gpu_structs.y_tex);
            glBindTexture(GL_TEXTURE_2D, gpu_structs.y_tex);
            glTexStorage2D(GL_TEXTURE_2D, 1, GL_R8, this->width, this->height);
            // glClearColor(1.0,1.0,1.0,1.0);
            this->log(l + "Y texture ready, id=" + std::to_string(gpu_structs.y_tex));

            this->log(l + "Making UV texture");
            // UV plane: RG8 format at half resolution
            glGenTextures(1, &gpu_structs.uv_tex);
            glBindTexture(GL_TEXTURE_2D, gpu_structs.uv_tex);
            glTexStorage2D(GL_TEXTURE_2D, 1, GL_RG8, this->width/2, this->height/2);
            this->log(l + "UV texture ready, id=" + std::to_string(gpu_structs.uv_tex));

            // Export Y plane
            this->log(l + "Making Y image");
            EGLint y_attrs[] = {
                EGL_GL_TEXTURE_LEVEL_KHR, 0,
                EGL_IMAGE_PRESERVED_KHR, EGL_TRUE,
                EGL_LINUX_DRM_FOURCC_EXT, DRM_FORMAT_R8,
                EGL_NONE
            };
            gpu_structs.egl_image_y = eglCreateImageKHR(this->egl_display, this->egl_ctx, EGL_GL_TEXTURE_2D_KHR, 
                                                    (EGLClientBuffer)(uintptr_t)gpu_structs.y_tex, y_attrs);
            if (gpu_structs.egl_image_y == EGL_NO_IMAGE) {
                err = eglGetError();
                oss << std::hex << err;  // lowercase hex, no prefix
                std::string err_hex = oss.str();
                oss.clear();
                this->err("Failed to create EGL Y image, err=0x" + err_hex + ": " + eglErrorString(err));
                return false;
            }
            this->log(l + "Exporting Y image");
            int y_fd;
            if (!this->eglExportDMABUFImageMESA(this->egl_display, gpu_structs.egl_image_y, &y_fd, &gpu_structs.y_stride, &gpu_structs.y_offset)) {
                err = eglGetError();
                oss << std::hex << err;  // lowercase hex, no prefix
                std::string err_hex = oss.str();
                oss.clear();
                this->err("Failed to export Y DMA-BUF, err=0x" + err_hex + ": " + eglErrorString(err));
                eglDestroyImageKHR(this->egl_display, gpu_structs.egl_image_y);
                return false;
            }
            gpu_structs.y_fd = y_fd;
            this->log(l + "Y image buf exported; fd=" + std::to_string(gpu_structs.y_fd) + ", stride=" + std::to_string(gpu_structs.y_stride) + ", offset=" + std::to_string(gpu_structs.y_offset));

            // Export UV plane
            this->log(l + "Making UV image");
            EGLint uv_attrs[] = {
                EGL_GL_TEXTURE_LEVEL_KHR, 0,
                EGL_IMAGE_PRESERVED_KHR, EGL_TRUE,
                EGL_LINUX_DRM_FOURCC_EXT, DRM_FORMAT_GR88,
                EGL_NONE
            };
            gpu_structs.egl_image_uv = eglCreateImageKHR(this->egl_display, this->egl_ctx, EGL_GL_TEXTURE_2D_KHR, 
                                                    (EGLClientBuffer)(uintptr_t)gpu_structs.uv_tex, uv_attrs);
            if (gpu_structs.egl_image_uv == EGL_NO_IMAGE) {
                err = eglGetError();
                oss << std::hex << err;  // lowercase hex, no prefix
                std::string err_hex = oss.str();
                oss.clear();
                this->err("Failed to create EGL UV image, err=0x" + err_hex + ": " + eglErrorString(err));
                return false;
            }
            this->log(l + "Exporting UV image");
            int uv_fd;
            if (!this->eglExportDMABUFImageMESA(this->egl_display, gpu_structs.egl_image_uv, &uv_fd, &gpu_structs.uv_stride, &gpu_structs.uv_offset)) {
                err = eglGetError();
                oss << std::hex << err;  // lowercase hex, no prefix
                std::string err_hex = oss.str();
                oss.clear();
                this->err("Failed to export UV DMA-BUF, err=0x" + err_hex + ": " + eglErrorString(err));
                eglDestroyImageKHR(this->egl_display, gpu_structs.egl_image_y);
                eglDestroyImageKHR(this->egl_display, gpu_structs.egl_image_uv);
                return false;
            }
            gpu_structs.uv_fd = uv_fd;
            this->log(l + "UV image buf exported; fd=" + std::to_string(gpu_structs.uv_fd) + ", stride=" + std::to_string(gpu_structs.uv_stride) + ", offset=" + std::to_string(gpu_structs.uv_offset));

            this->log(l + "Making VA surface");

            // Create NV12 surface from DMA-BUF FDs
            VASurfaceAttribExternalBuffers attribs_ext_buf = {};
            attribs_ext_buf.pixel_format = VA_FOURCC_NV12;
            attribs_ext_buf.width = this->width;
            attribs_ext_buf.height = this->height;
            attribs_ext_buf.num_planes = 2;
            attribs_ext_buf.pitches[0] = gpu_structs.y_stride;
            attribs_ext_buf.offsets[0] = gpu_structs.y_offset;
            attribs_ext_buf.pitches[1] = gpu_structs.uv_stride;
            attribs_ext_buf.offsets[1] = gpu_structs.uv_offset;
            attribs_ext_buf.buffers = (unsigned long*)malloc(2 * sizeof(unsigned long));
            attribs_ext_buf.buffers[0] = gpu_structs.y_fd;
            attribs_ext_buf.buffers[1] = gpu_structs.uv_fd;
            attribs_ext_buf.num_buffers = 2;
            attribs_ext_buf.flags = VA_SURFACE_ATTRIB_MEM_TYPE_DRM_PRIME | VA_SURFACE_ATTRIB_USAGE_HINT_ENCODER;

            VASurfaceAttrib attribs[2] = {
                {
                    VASurfaceAttribMemoryType,
                    VA_SURFACE_ATTRIB_SETTABLE,
                    { VAGenericValueTypeInteger, { VA_SURFACE_ATTRIB_MEM_TYPE_DRM_PRIME } }
                },
                {
                    VASurfaceAttribExternalBufferDescriptor,
                    VA_SURFACE_ATTRIB_SETTABLE,
                    { VAGenericValueTypePointer, { } }
                }
            };
            attribs[1].value.value.p = &attribs_ext_buf;
            
            // va_surface = 0;
            VAStatus status = vaCreateSurfaces(this->va_display,
                                            VA_RT_FORMAT_YUV420,  // Changed from YUV422 to YUV420 to match NV12 format
                                            this->width, this->height,
                                            &gpu_structs.va_surface, 1,
                                            attribs, 2);
            //free(ext_buf.buffers);
            
            if (status != VA_STATUS_SUCCESS) {
                this->err("Failed to create VA surface: " + std::string(vaErrorStr(status)));
                eglDestroyImageKHR(this->egl_display, gpu_structs.egl_image_y);
                eglDestroyImageKHR(this->egl_display, gpu_structs.egl_image_uv);
                close(gpu_structs.y_fd);
                close(gpu_structs.uv_fd);
                return false;
            }

            this->log(l + "VA surface ready, id=" + std::to_string(gpu_structs.va_surface));

            this->log(l + "Making VA frame");
            gpu_structs.va_frame = av_frame_alloc();
            // if (!gpu_structs.va_frame) {
            //     this->err("Failed to allocate VA frame");
            //     vaDestroySurfaces(this->va_display, &gpu_structs.va_surface, 1);
            //     eglDestroyImageKHR(this->egl_display, gpu_structs.egl_image_y);
            //     eglDestroyImageKHR(this->egl_display, gpu_structs.egl_image_uv);
            //     close(gpu_structs.y_fd);
            //     close(gpu_structs.uv_fd);
            //     return false;
            // }

            err = av_hwframe_get_buffer(this->hw_frames_ctx, gpu_structs.va_frame, 0);
            if (err < 0) {
                char errbuf[64];
                av_strerror(err, errbuf, sizeof(errbuf));
                this->err("Failed to get frame buffer from pool: " + std::string(errbuf));
                av_frame_free(&gpu_structs.va_frame);
                return false;
            }

            // gpu_structs.va_frame->format = AV_PIX_FMT_VAAPI; // this has no effect on anything
            // gpu_structs.va_frame->hw_frames_ctx = av_buffer_ref(this->codec_ctx->hw_frames_ctx);
            gpu_structs.va_frame->data[3] = (uint8_t*)(uintptr_t)gpu_structs.va_surface;  // Store VASurfaceID in data[3]    
            // gpu_structs.va_frame->width = this->width;
            // gpu_structs.va_frame->height = this->height;

            // Set up cleanup callback
            struct VAAPICleanupData {
                int pool_number;
                FFmpegEncoder * that;
                ZeroCopyGPUStructs gpu_structs;
            };

            VAAPICleanupData* cleanup_data = new VAAPICleanupData{i, this, gpu_structs};

            gpu_structs.va_frame->buf[0] = av_buffer_create(nullptr, 0, [](void* opaque, uint8_t* data) {
                VAAPICleanupData* d = static_cast<VAAPICleanupData*>(opaque);
                d->that->log("VAAPI " + std::to_string(d->pool_number) +  " cleanup...");
                vaDestroySurfaces(d->that->egl_display, &d->gpu_structs.va_surface, 1);
                d->that->eglDestroyImageKHR(d->that->egl_display, d->gpu_structs.egl_image_y);
                d->that->eglDestroyImageKHR(d->that->egl_display, d->gpu_structs.egl_image_uv);
                // d->that->eglDestroyImageKHR(d->egl_dpy, d->egl_img_y);
                // d->that->eglDestroyImageKHR(d->egl_dpy, d->egl_img_uv);
                close(d->gpu_structs.y_fd);
                close(d->gpu_structs.uv_fd);
                d->that->log("VAAPI cleanup " + std::to_string(d->pool_number) + " done.");
                delete d;
            }, reinterpret_cast<void *>(cleanup_data), 0);

            this->log(l + "VA frame ready");

            // add to pool
            this->zero_copy_gpu_pool.push_back(gpu_structs);
        }

        this->log("<<<< Zero-copy Converter initialized.");

        // VASurfaceID tmp_surf;
        // VAStatus s = vaCreateSurfaces(va_display, VA_RT_FORMAT_YUV420, width, height, &tmp_surf, 1, NULL, 0);
        // if (s == VA_STATUS_SUCCESS) {
        //     VAImage img;
        //     s = vaDeriveImage(va_display, tmp_surf, &img);
        //     printf("vaDeriveImage on plain surface -> %d\n", s);
        //     if (s == VA_STATUS_SUCCESS) {
        //         vaDestroyImage(va_display, img.image_id);
        //     }
        //     vaDestroySurfaces(va_display, &tmp_surf, 1);
        // } else {
        //     printf("vaCreateSurfaces plain failed %d\n", s);
        // }

        // return false;

        return true;
    }

    bool FFmpegEncoder::loadEGLExtensions() {
        const char* exts = eglQueryString(this->egl_display, EGL_EXTENSIONS);
        if (!exts || 
            !strstr(exts, "EGL_MESA_image_dma_buf_export") || 
            !strstr(exts, "EGL_EXT_image_dma_buf_import")) {
            this->err("DMA-BUF export extensions not supported!");
            return false;
        }
        // if (!exts || !strstr(exts, "MESA_EGL_FORCE_LINEAR_ATTRIB")) {
        //     fprintf(stderr, "MESA_EGL_FORCE_LINEAR_ATTRIB not supported!\n");
        // }
        // if (!exts || !strstr(exts, "EGL_MESA_image_dma_buf_export_modifiers")) {
        //     fprintf(stderr, "Modifier export extension not supported!\n");
        //     // return false;
        // }
        if (!exts || !strstr(exts, "EGL_KHR_gl_colorspace")) {
            this->err("EGL_KHR_gl_colorspace not supported! Cannot use EGL_GL_COLORSPACE");
        }
    
        // Load GLES extensions
        // this->glEGLImageTargetTexture2DOES = (PFNGLEGLIMAGETARGETTEXTURE2DOESPROC)eglGetProcAddress("glEGLImageTargetTexture2DOES");
        // if (!this->glEGLImageTargetTexture2DOES) {
        //     this->err("EGL image extension not available");
        //     return false;
        // }
        this->eglCreateImageKHR = (PFNEGLCREATEIMAGEKHRPROC)eglGetProcAddress("eglCreateImageKHR");
        this->eglDestroyImageKHR = (PFNEGLDESTROYIMAGEKHRPROC)eglGetProcAddress("eglDestroyImageKHR");
        if (!this->eglCreateImageKHR || !this->eglDestroyImageKHR) {
            this->err("Failed to load EGL image extensions (1)");
            return false;
        }
        // this->eglExportDMABUFImageQueryMESA = (PFNEGLEXPORTDMABUFIMAGEQUERYMESAPROC) eglGetProcAddress("eglExportDMABUFImageQueryMESA");
        this->eglExportDMABUFImageMESA = (PFNEGLEXPORTDMABUFIMAGEMESAPROC) eglGetProcAddress("eglExportDMABUFImageMESA");
        if (!this->eglExportDMABUFImageMESA) {
            this->err("Failed to load EGL image extensions eglExportDMABUFImageMESA");
            return false;
        }
        // EGL setup done

        return true;
    }

    bool FFmpegEncoder::setupSWScaler(AVPixelFormat opencv_format, AVPixelFormat codec_input_format) {
        // initialize sw conversion context
        this->sws_ctx = sws_getContext(this->width, this->height,
                                       opencv_format, // OpenCV format
                                       this->width, this->height,
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

        return true;
    }

    bool FFmpegEncoder::debugTexture(int tex, int tex_width, int tex_height) {
        // Read back the texture data
        GLuint fbo;
        glGenFramebuffers(1, &fbo);
        glBindFramebuffer(GL_FRAMEBUFFER, fbo);

        // Attach Y texture to framebuffer
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, tex, 0);

        // Check framebuffer status
        if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
            this->err("Framebuffer not complete for Y texture readback");
            glDeleteFramebuffers(1, &fbo);
            return false;
        }

        // Set viewport
        glViewport(0, 0, tex_width, tex_height);

        // Read pixels
        std::vector<uint8_t> y_data(tex_width * tex_height);
        glReadPixels(0, 0, tex_width, tex_height, GL_RED, GL_UNSIGNED_BYTE, y_data.data());

        // Clean up
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        glDeleteFramebuffers(1, &fbo);

        this->log("Texture " + std::to_string(tex)+ " sample (first 64 pixels):");
        std::string pixels = "";
        for (int i = 0; i < std::min(64, (int)y_data.size()); i++) {
            if (i % 16 == 0)
                pixels += "\n";
            pixels += std::to_string(y_data[i]) + " ";
        }
        this->log(pixels);
        // fprintf(stdout, "\n");

        return true;
    }

    bool FFmpegEncoder::encodeFrameZeroCopy(uint gl_texture_id, std_msgs::msg::Header header) {

        if (!this->running)
            return false;

        this->log(">> Zero copy encoding gl_id = " + std::to_string(gl_texture_id) + ", using pool structs " + std::to_string(this->zero_copy_current) + " >>");
        ZeroCopyGPUStructs gpu_structs = this->zero_copy_gpu_pool[this->zero_copy_current];
        this->zero_copy_current++;
        if (this->zero_copy_current == zero_copy_pool_size-1) {
            this->zero_copy_current = 0;
        }
        
        std::ostringstream oss; // debugs
        GLenum gl_error = glGetError();
        oss << this->egl_display; 
        this->log("Egl_display = "+ oss.str());
        oss.clear();
        oss << this->egl_ctx;
        this->log("Egl_ctx = " + oss.str());
        oss.clear();

        // Verify texture exists in OpenGL
        GLboolean is_texture = glIsTexture(gl_texture_id);
        if (!is_texture) {
            this->err("Invalid texture ID: " + std::to_string(gl_texture_id));
            return false;
        }
       
        glBindTexture(GL_TEXTURE_2D, gl_texture_id);
        if (gl_error != GL_NO_ERROR) {
            oss << std::hex << gl_error;  // lowercase hex, no prefix
            std::string hexStr = oss.str();
            this->err("OpenGL error accessing texture " + std::to_string(gl_texture_id) + ": err 0x" + hexStr);
            oss.clear();
            return false;
        }
        
        // Get texture parameters to verify it's valid
        GLint tex_width, tex_height, format, red_size, green_size, blue_size, alpha_size;
        glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_WIDTH, &tex_width);
        glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_HEIGHT, &tex_height);
        glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_INTERNAL_FORMAT, &format);
        glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_RED_SIZE, &red_size);
        glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_GREEN_SIZE, &green_size);
        glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_BLUE_SIZE, &blue_size);
        glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_ALPHA_SIZE, &alpha_size);

        oss << std::hex << format;  // lowercase hex, no prefix
        std::string hex_format = oss.str();
        oss.clear();
        this->log("Texture info: " + std::to_string(tex_width) + "x" + std::to_string(tex_height) + ", format=0x" + hex_format
                + " RGBA=" + std::to_string(red_size) + std::to_string(green_size) + std::to_string(blue_size) + std::to_string(alpha_size));

        if (format != GL_SRGB8_ALPHA8) {
            oss << std::hex << GL_SRGB8_ALPHA8;  // lowercase hex, no prefix
            std::string correct_hex_format = oss.str();
            oss.clear();
            this->err("Texture format mismatch: 0x"+ hex_format +"  vs GL_SRGB8_ALPHA8 (0x" + correct_hex_format + ")");
            return false;
        }
        
        glFlush();

        this->log("Setting up conversion shader");
        glUseProgram(this->nv12_conversion_program);

        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, gl_texture_id);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

        glBindImageTexture(1, gpu_structs.y_tex, 0, GL_FALSE, 0, GL_READ_WRITE, GL_R8);
        glBindImageTexture(2, gpu_structs.uv_tex, 0, GL_FALSE, 0, GL_READ_WRITE, GL_RG8);

        GLint input_tex_loc = glGetUniformLocation(this->nv12_conversion_program, "inputTex");
        glUniform1i(input_tex_loc, 0);  // Texture unit 0

        this->log("Dispatching compute");
        glDispatchCompute((this->width + 15)/16, (this->height + 15)/16, 1);

        GLenum err = glGetError();
        if (err != GL_NO_ERROR) {
            oss << std::hex << err;  // lowercase hex, no prefix
            std::string err_hex = oss.str();
            oss.clear();
            this->err("OpenGL error after dispatch: 0x" + err_hex);
            return false;
        }

        glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT | GL_TEXTURE_FETCH_BARRIER_BIT);
        glFinish(); // Ensure complet

        err = glGetError();
        if (err != GL_NO_ERROR) {
            oss << std::hex << err;  // lowercase hex, no prefix
            std::string err_hex = oss.str();
            oss.clear();
            this->err("OpenGL error after glFinish: 0x" + err_hex);
            return false;
        }

        this->log("Conversion done");

        if (!debugTexture(gpu_structs.y_tex, this->width, this->height)) {
            oss << std::hex << err;  // lowercase hex, no prefix
            std::string err_hex = oss.str();
            oss.clear();
            this->err("OpenGL error after glFinish: 0x" + err_hex);
            return false;
        }

        // Debug UV texture (half resolution due to NV12 format)
        if (!debugTexture(gpu_structs.uv_tex, this->width/2, this->height/2)) {
            oss << std::hex << err;  // lowercase hex, no prefix
            std::string err_hex = oss.str();
            oss.clear();
            this->err("OpenGL error checking UV texture: 0x" + err_hex);
            return false;
        }

        // Wait for surface to be ready
        VAStatus sync_status = vaSyncSurface(this->va_display, gpu_structs.va_surface);
        if (sync_status != VA_STATUS_SUCCESS) {
            this->err("Surface sync failed: " + std::string(vaErrorStr(sync_status)));
            return false;
        }

        VASurfaceStatus surface_status;
        VAStatus query_status = vaQuerySurfaceStatus(this->va_display, gpu_structs.va_surface, &surface_status);
        if (query_status != VA_STATUS_SUCCESS) {
            this->err("Surface status query failed: " + std::string(vaErrorStr(query_status)));
            return false;
        }

        this->log("Surface status: " + std::to_string(surface_status));
        
        if (surface_status != VASurfaceReady) {
            this->err("Surface not ready, status: " + std::to_string(surface_status));
            return false;
        }

        // VAImage image;
        // VAStatus s = vaDeriveImage(this->va_display, gpu_structs.va_surface, &image);
        // if (s != VA_STATUS_SUCCESS) {
        //     fprintf(stderr, "vaDeriveImage failed: %d\n", s);
        // } else {
        //     void *p = nullptr;
        //     s = vaMapBuffer(this->va_display, image.buf, &p);
        //     if (s != VA_STATUS_SUCCESS) {
        //         fprintf(stderr, "vaMapBuffer failed: %d\n", s);
        //     } else {
        //         // image.format, image.width, image.height, image.pitches[], image.offsets[]
        //         // Dump Y plane (write PGM)
        //         FILE *f = fopen("/tmp/va_y_plane.pgm", "wb");
        //         if (f) {
        //             fprintf(f, "P5\n%d %d\n255\n", image.width, image.height);
        //             unsigned char *base = (unsigned char*)p + image.offsets[0];
        //             for (int y = 0; y < image.height; ++y) {
        //                 fwrite(base + y * image.pitches[0], 1, image.width, f);
        //             }
        //             fclose(f);
        //             printf("Wrote /tmp/va_y_plane.pgm\n");
        //         }
        //         // Optionally also dump UV plane if image.num_planes > 1
        //         vaUnmapBuffer(this->va_display, image.buf);
        //     }
        //     vaDestroyImage(this->va_display, image.image_id);
        // }


        // Encode the frame
        this->log("Sending frame to encoder");
        int ret = avcodec_send_frame(this->codec_ctx, gpu_structs.va_frame);
        this->log("Send frame returned=" + std::to_string(ret));

        if (ret < 0) {
            char errbuf[256];
            av_strerror(ret, errbuf, sizeof(errbuf));
            this->err("Error sending frame to encoder: " + std::string(errbuf) + " (ret=" + std::to_string(ret) + ")");
            return false;
        }
        
        auto pkt = av_packet_alloc();
        if (!pkt) {
            this->err("Could not allocate packet");
            return false;
        }

        ret = avcodec_receive_packet(this->codec_ctx, pkt);

        if (ret == AVERROR(EAGAIN)) { // The encoder needs more input frames before it can output a packet
            this->log("<< Encoder returned EAGAIN");
            av_packet_free(&pkt);
            return true;
        } else if (ret == AVERROR_EOF) {
            this->err("Error during receiving frame - AVERROR_EOF");
            av_packet_free(&pkt);
            this->running = false;
            return false;
        } else if (ret < 0) {
            this->err("Error during receiving frame: err=" + std::to_string(ret));
            av_packet_free(&pkt);
            this->running = false;
            return false;
        }

        if (packet_callback) {
            auto msg = std::make_shared<ffmpeg_image_transport_msgs::msg::FFMPEGPacket>();
            msg->header = header;
            msg->encoding = "h.264";
            msg->width = this->width;
            msg->height = this->height;
            msg->flags = pkt->flags;
            msg->is_bigendian = false;
            msg->pts = pkt->pts; //calculated from the initial header stamp
            // frame->data.resize(pkt->size);
            msg->data.assign(pkt->data, pkt->data + pkt->size);
            packet_callback(msg); // produce ros message
        }
        
        this->log("<< Zero copy encoding gl_id = " + std::to_string(gl_texture_id)+ " done. Pkt data size=" + std::to_string(pkt->size) + "B");

        av_packet_unref(pkt);

        return true;
    }

    void FFmpegEncoder::cleanupZeroCopyConverter() {
        // VAAPI cleanup

        this->log("Cleaning Zero-copy converter");

        if (this->va_display) {
            if (this->va_ctx) vaDestroyContext(this->va_display, this->va_ctx);
            if (this->va_config) vaDestroyConfig(this->va_display, this->va_config);
            vaTerminate(this->va_display);
        }

        // EGL/GBM cleanup
        if (this->egl_display != EGL_NO_DISPLAY) {
            eglMakeCurrent(this->egl_display, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
            if (this->egl_ctx != EGL_NO_CONTEXT) eglDestroyContext(this->egl_display, this->egl_ctx);
            eglTerminate(this->egl_display);
        }

        if (this->gbm_dev) gbm_device_destroy(this->gbm_dev);
        if (this->drm_fd >= 0) close(this->drm_fd);

        this->log("Zero-copy converter clean");
    }

    // Convert RGB8 GL texture to NV12 VA surface
    // VASurfaceID FFmpegEncoder::convertToNV12ZeroCopy(GLuint rgbTextureId) {

    //     // 1. Create temporary GBM buffer
    //     gbm_bo* bo = gbm_bo_create(this->gbm_dev, this->width, this->height, GBM_FORMAT_XRGB8888,
    //                             GBM_BO_USE_RENDERING | GBM_BO_USE_LINEAR);
    //     if (!bo) {
    //         fprintf(stderr, "Failed to create GBM buffer\n");
    //         return VA_INVALID_ID;
    //     }

    //     // 2. Create EGL image
    //     EGLImageKHR egl_img = this->eglCreateImageKHR(
    //         this->egl_display, this->egl_ctx, EGL_NATIVE_PIXMAP_KHR,
    //         (EGLClientBuffer)(uintptr_t)gbm_bo_get_handle(bo).u32, nullptr
    //     );
    //     if (egl_img == EGL_NO_IMAGE_KHR) {
    //         fprintf(stderr, "Failed to create EGL image\n");
    //         gbm_bo_destroy(bo);
    //         return VA_INVALID_ID;
    //     }

    //     // 3. Setup export texture
    //     GLuint export_tex;
    //     glGenTextures(1, &export_tex);
    //     glBindTexture(GL_TEXTURE_2D, export_tex);
    //     glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    //     glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    //     this->glEGLImageTargetTexture2DOES(GL_TEXTURE_2D, egl_img);

    //     // 4. Render to export texture
    //     GLuint fbo;
    //     glGenFramebuffers(1, &fbo);
    //     glBindFramebuffer(GL_FRAMEBUFFER, fbo);
    //     glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, export_tex, 0);
        
    //     // [Actual rendering from rgbTexture would go here]
        
    //     glBindFramebuffer(GL_FRAMEBUFFER, 0);
    //     glFinish();

    //     // 5. Export DMA-BUF
    //     int dma_fd = gbm_bo_get_fd(bo);
    //     if (dma_fd < 0) {
    //         fprintf(stderr, "Failed to get DMA-BUF fd\n");
    //         this->eglDestroyImageKHR(this->egl_display, egl_img);
    //         gbm_bo_destroy(bo);
    //         return VA_INVALID_ID;
    //     }
    //     uint32_t stride = gbm_bo_get_stride(bo);

    //     // 6. Create VA surface
    //     VASurfaceAttribExternalBuffers ext_buf = {
    //          VA_FOURCC_RGBA,
    //         static_cast<uint32_t>(this->width),
    //         static_cast<uint32_t>(this->height),
    //         stride * height,
    //         1,
    //         {stride},
    //         {0},
    //         reinterpret_cast<uintptr_t*>(&dma_fd),
    //         1,
    //         VA_SURFACE_ATTRIB_MEM_TYPE_DRM_PRIME_2,
    //         nullptr,
    //     };

    //     VASurfaceAttrib attribs[2] = {
    //         {
    //             VASurfaceAttribMemoryType,
    //             VA_SURFACE_ATTRIB_SETTABLE,
    //             {VAGenericValueTypeInteger, VA_SURFACE_ATTRIB_MEM_TYPE_DRM_PRIME_2}
    //         },
    //         {
    //             VASurfaceAttribExternalBufferDescriptor,
    //             VA_SURFACE_ATTRIB_SETTABLE,
    //             {VAGenericValueTypePointer, {}}
    //         }
    //     };
    //     attribs[1].value.value.p = &ext_buf;

    //     VASurfaceID va_surface;
    //     if (vaCreateSurfaces(this->va_display, VA_RT_FORMAT_RGB32, this->width, this->height,
    //                         &va_surface, 1, attribs, 2) != VA_STATUS_SUCCESS) {
    //         fprintf(stderr, "Failed to create VA surface\n");
    //         close(dma_fd);
    //         this->eglDestroyImageKHR(this->egl_display, egl_img);
    //         gbm_bo_destroy(bo);
    //         return VA_INVALID_ID;
    //     }

    //     // 7. Convert to NV12
    //     VASurfaceID nv12_surface;
    //     if (vaCreateSurfaces(this->va_display, VA_RT_FORMAT_YUV420, this->width, this->height,
    //                         &nv12_surface, 1, nullptr, 0) != VA_STATUS_SUCCESS) {
    //         fprintf(stderr, "Failed to create NV12 surface\n");
    //         vaDestroySurfaces(this->va_display, &va_surface, 1);
    //         close(dma_fd);
    //         this->eglDestroyImageKHR(this->egl_display, egl_img);
    //         gbm_bo_destroy(bo);
    //         return VA_INVALID_ID;
    //     }

    //     VABufferID pipeline_buf;
    //     if (vaCreateBuffer(this->va_display, this->va_ctx, VAProcPipelineParameterBufferType,
    //                     sizeof(VAProcPipelineParameterBuffer), 1, nullptr, &pipeline_buf) != VA_STATUS_SUCCESS) {
    //         fprintf(stderr, "Failed to create pipeline buffer\n");
    //         vaDestroySurfaces(this->va_display, &nv12_surface, 1);
    //         vaDestroySurfaces(this->va_display, &va_surface, 1);
    //         close(dma_fd);
    //         this->eglDestroyImageKHR(this->egl_display, egl_img);
    //         gbm_bo_destroy(bo);
    //         return VA_INVALID_ID;
    //     }

    //     VAProcPipelineParameterBuffer* params;
    //     vaMapBuffer(this->va_display, pipeline_buf, (void**)&params);
    //     memset(params, 0, sizeof(*params));
    //     params->surface = va_surface;
    //     vaUnmapBuffer(this->va_display, pipeline_buf);

    //     vaBeginPicture(this->va_display, this->va_ctx, nv12_surface);
    //     vaRenderPicture(this->va_display, this->va_ctx, &pipeline_buf, 1);
    //     vaEndPicture(this->va_display, this->va_ctx);

    //     // 8. Cleanup temporary resources
    //     vaDestroyBuffer(this->va_display, pipeline_buf);
    //     vaDestroySurfaces(this->va_display, &va_surface, 1);
    //     close(dma_fd);
    //     this->eglDestroyImageKHR(this->egl_display, egl_img);
    //     gbm_bo_destroy(bo);

    //     return nv12_surface;
    // }

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

    // AVFrame* FFmpegEncoder::convertRGBToNV12(AVFrame* rgb_frame) {
    //     // Create VAAPI Video Processing context for color conversion
    //     VAConfigID config_id;
    //     VAContextID context_id;
        
    //     VAStatus status = vaCreateConfig(this->va_display, VAProfileNone, VAEntrypointVideoProc,
    //                                    nullptr, 0, &config_id);
    //     if (status != VA_STATUS_SUCCESS) return nullptr;
        
    //     status = vaCreateContext(this->va_display, config_id, this->width, this->height, VA_PROGRESSIVE,
    //                            nullptr, 0, &context_id);
    //     if (status != VA_STATUS_SUCCESS) {
    //         vaDestroyConfig(this->va_display, config_id);
    //         return nullptr;
    //     }
        
    //     // Create NV12 output surface
    //     VASurfaceID nv12_surface;
    //     status = vaCreateSurfaces(this->va_display, VA_RT_FORMAT_YUV420, this->width, this->height,
    //                             &nv12_surface, 1, nullptr, 0);
    //     if (status != VA_STATUS_SUCCESS) {
    //         vaDestroyContext(this->va_display, context_id);
    //         vaDestroyConfig(this->va_display, config_id);
    //         return nullptr;
    //     }
        
    //     // Set up VPP pipeline
    //     VABufferID pipeline_buf;
    //     VAProcPipelineParameterBuffer pipeline_param = {};
    //     pipeline_param.surface = (VASurfaceID)(uintptr_t)rgb_frame->data[3];
    //     pipeline_param.surface_region = nullptr;
    //     pipeline_param.output_region = nullptr;
    //     pipeline_param.output_background_color = 0;
    //     pipeline_param.filter_flags = VA_FILTER_SCALING_DEFAULT;
        
    //     status = vaCreateBuffer(this->va_display, context_id, VAProcPipelineParameterBufferType,
    //                           sizeof(pipeline_param), 1, &pipeline_param, &pipeline_buf);
    //     if (status != VA_STATUS_SUCCESS) {
    //         vaDestroySurfaces(this->va_display, &nv12_surface, 1);
    //         vaDestroyContext(this->va_display, context_id);
    //         vaDestroyConfig(this->va_display, config_id);
    //         return nullptr;
    //     }
        
    //     // Execute conversion
    //     status = vaBeginPicture(this->va_display, context_id, nv12_surface);
    //     if (status == VA_STATUS_SUCCESS) {
    //         status = vaRenderPicture(this->va_display, context_id, &pipeline_buf, 1);
    //         if (status == VA_STATUS_SUCCESS) {
    //             status = vaEndPicture(this->va_display, context_id);
    //         }
    //     }
        
    //     vaDestroyBuffer(this->va_display, pipeline_buf);
    //     vaDestroyContext(this->va_display, context_id);
    //     vaDestroyConfig(this->va_display, config_id);
        
    //     if (status != VA_STATUS_SUCCESS) {
    //         vaDestroySurfaces(this->va_display, &nv12_surface, 1);
    //         return nullptr;
    //     }
        
    //     // Create AVFrame for NV12 surface
    //     AVFrame* nv12_frame = av_frame_alloc();
    //     if (!nv12_frame) {
    //         vaDestroySurfaces(this->va_display, &nv12_surface, 1);
    //         return nullptr;
    //     }
        
    //     nv12_frame->format = AV_PIX_FMT_VAAPI;
    //     nv12_frame->width = this->width;
    //     nv12_frame->height = this->height;
    //     nv12_frame->data[3] = (uint8_t*)(uintptr_t)nv12_surface;
    //     nv12_frame->hw_frames_ctx = av_buffer_ref(this->hw_frames_ctx);
        
    //     return nv12_frame;
    // }


    void FFmpegEncoder::scalerWorker() {
         std::cout << "["+this->toString()+"] FFmpegEncoder scaler runnig..." << std::endl;

        //int err;
        while (this->running) {

            std::unique_lock<std::mutex> scaler_lock(this->scaler_mutex);
            this->scaler_cv.wait(scaler_lock, [this] { return !this->scaler_queue.empty() || !this->running; });
            if (this->scaler_queue.empty() || !this->running) 
                break;

            ScalerRequest req;
            //while (this->scaler_queue.size()) {
            req = this->scaler_queue.front();
            this->scaler_queue.pop();
            //}

            scaler_lock.unlock();

            AVFrame* sw_frame = this->sw_frame_buffers[this->current_sw_frame_buffer];
            this->current_sw_frame_buffer++;
            if (this->current_sw_frame_buffer == this->num_frame_buffers) {
                this->current_sw_frame_buffer = 0;
            }

            const uint8_t* src_data[] = { req.raw_frame.data };
            int src_linesize[] = { static_cast<int>(req.raw_frame.step) };
            
            // sw scaling & copy here - expensive
            sws_scale(this->sws_ctx, src_data, src_linesize, 0, this->height, 
                    sw_frame->data, sw_frame->linesize);

            // Send for encoding
            sw_frame->pts = convertToRtpTimestamp(req.header.stamp.sec, req.header.stamp.nanosec);
            this->sendFrameToEncoder(sw_frame);
        }

        std::cout << "["+this->toString()+"] FFmpegEncoder scaler stopped..." << std::endl;
    }

    void FFmpegEncoder::sendFrameToEncoder(AVFrame* input_frame) {
        std::lock_guard<std::mutex> queue_lock(this->encoder_mutex);

        // this->encoder_queue.push(input_frame);
        // this->encoder_cv.notify_one();

        int err;

        auto pkt = av_packet_alloc();
        if (!pkt) {
            throw std::runtime_error("["+this->toString()+"] Could not allocate packet");
        }

        AVFrame* hw_frame = this->hw_frame_buffers[this->current_hw_frame_buffer];
        this->current_hw_frame_buffer++;
        if (this->current_hw_frame_buffer == this->num_frame_buffers) {
            this->current_hw_frame_buffer = 0;
        }

        if ((err = av_hwframe_transfer_data(hw_frame, input_frame, 0)) < 0) {
            throw std::runtime_error("Error while transferring sw frame data to hw surface. Error code: " + std::to_string(err));
        }
        hw_frame->pts = input_frame->pts;
        
        int ret = avcodec_send_frame(this->codec_ctx, hw_frame);
        if (ret < 0) {
            throw std::runtime_error("["+this->toString()+"] Error sending frame to encoder");
        }
            
        // Convert from OpenCV BGR to encoder's format
        ret = avcodec_receive_packet(this->codec_ctx, pkt);

        if (ret == AVERROR(EAGAIN)) { // The encoder needs more input frames before it can output a packet
            // No more packets will be produced (flush completed)
            // log("["+this->toString()+"] Error during receiving frame - EAGAIN");
            av_packet_free(&pkt);
            return;
        } else if (ret == AVERROR_EOF) {
            std::cout << "["+this->toString()+"] Error during receiving frame - AVERROR_EOF" << std::endl;
            av_packet_free(&pkt);
            this->running = false;
            return;
        } else if (ret < 0) {
            std::cout << "["+this->toString()+"] Error during receiving frame, " + std::to_string(ret) << std::endl;
            av_packet_free(&pkt);
            this->running = false;
            return;
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
            msg->pts = hw_frame->pts; //calculated from the initial header stamp
            // frame->data.resize(pkt->size);
            msg->data.assign(pkt->data, pkt->data + pkt->size);
            packet_callback(msg); // produce ros message
        }
        
        av_packet_unref(pkt);
    }

    void FFmpegEncoder::flush() {
        sendFrameToEncoder(nullptr);  // Flush the encoder
    }

    void FFmpegEncoder::encoderWorker() {

        std::cout << "["+this->toString()+"] FFmpegEncoder worker runnig..." << std::endl;

        int err;
        while (this->running) {

            auto pkt = av_packet_alloc();
            if (!pkt) {
                throw std::runtime_error("["+this->toString()+"] Could not allocate packet");
            }

            std::unique_lock<std::mutex> queue_lock(this->encoder_mutex);
            this->encoder_cv.wait(queue_lock, [this] { return !this->encoder_queue.empty() || !this->running; });

            if (this->encoder_queue.empty()) 
                break;

            AVFrame* sw_frame;
            while (this->encoder_queue.size()) {
                sw_frame = this->encoder_queue.front();
                this->encoder_queue.pop();
            }

            queue_lock.unlock();
            
            AVFrame* frame;
            if (hw_device_type == AV_HWDEVICE_TYPE_VAAPI) {
                AVFrame* hw_frame = this->hw_frame_buffers[this->current_hw_frame_buffer];
                this->current_hw_frame_buffer++;
                if (this->current_hw_frame_buffer == this->num_frame_buffers) {
                    this->current_hw_frame_buffer = 0;
                }

                if ((err = av_hwframe_transfer_data(hw_frame, sw_frame, 0)) < 0) {
                    throw std::runtime_error("Error while transferring sw frame data to hw surface. Error code: " + std::to_string(err));
                }
                hw_frame->pts = sw_frame->pts;
                frame = hw_frame;
            } else {
                frame = sw_frame;
            }

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

        if (hw_device_type == AV_HWDEVICE_TYPE_VAAPI) {
            this->cleanupZeroCopyConverter();
        }

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