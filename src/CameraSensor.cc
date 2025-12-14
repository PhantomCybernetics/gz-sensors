/*
 * Copyright (C) 2017 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <chrono>
#include <gz/math/Pose3.hh>
#include <gz/msgs/camera_info.pb.h>
#include <gz/msgs/image.pb.h>

#include <gz/rendering/PixelFormat.hh>
#include <libavutil/pixfmt.h>
#include <mutex>
#include <ostream>
#include <queue>
#include <rclcpp/subscription.hpp>
#include <string>

#include <gz/common/Console.hh>
#include <gz/common/Event.hh>
#include <gz/common/Image.hh>
#include <gz/common/Profiler.hh>
#include <gz/common/StringUtils.hh>
#include <gz/math/Angle.hh>
#include <gz/math/Helpers.hh>
#include <gz/msgs/Utility.hh>
#include <gz/transport/Node.hh>
#include <gz/transport/TopicUtils.hh>

#include "gz/sensors/CameraSensor.hh"
#include "gz/sensors/ImageBrownDistortionModel.hh"
#include "gz/sensors/ImageDistortion.hh"
#include "gz/sensors/ImageGaussianNoiseModel.hh"
#include "gz/sensors/ImageNoise.hh"
#include "gz/sensors/Manager.hh"
#include "gz/sensors/RenderingEvents.hh"
#include "gz/sensors/SensorFactory.hh"
#include "gz/sensors/SensorTypes.hh"

#include <gz/rendering/Utils.hh>

#include "gz/sensors/DirectRosNode.hh"
#include "gz/sensors/FFmpegEncoder.hh"
#include "std_msgs/msg/header.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <ffmpeg_image_transport_msgs/msg/ffmpeg_packet.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/qos.hpp>
#include <std_msgs/msg/header.hpp>
#include <thread>

#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <GLES3/gl31.h>
#include <GLES2/gl2ext.h>
#include <GLES3/gl3ext.h>

#include "phntm_interfaces/srv/get_float32.hpp"
#include "phntm_interfaces/srv/set_float32.hpp"

#include "tf2_msgs/msg/tf_message.hpp"

using namespace gz;
using namespace sensors;

/// \brief Private data for CameraSensor
class gz::sensors::CameraSensorPrivate
{
  /// \brief Save an image
  /// \param[in] _data the image data to be saved
  /// \param[in] _width width of image in pixels
  /// \param[in] _height height of image in pixels
  /// \param[in] _format The format the data is in
  /// \return True if the image was saved successfully. False can mean
  /// that the path provided to the constructor does exist and creation
  /// of the path was not possible.
  /// \sa ImageSaver
  public: bool SaveImage(const unsigned char *_data, unsigned int _width,
    unsigned int _height, gz::common::Image::PixelFormatType _format);

  /// \brief Computes the OpenGL NDC matrix
  /// \param[in] _left Left vertical clipping plane
  /// \param[in] _right Right vertical clipping plane
  /// \param[in] _bottom Bottom horizontal clipping plane
  /// \param[in] _top Top horizontal clipping plane
  /// \param[in] _near Distance to the nearer depth clipping plane
  ///            This value is negative if the plane is to be behind
  ///            the camera
  /// \param[in] _far Distance to the farther depth clipping plane
  ///            This value is negative if the plane is to be behind
  ///            the camera
  /// \return OpenGL NDC (Normalized Device Coordinates) matrix
  public: static math::Matrix4d BuildNDCMatrix(
          double _left, double _right,
          double _bottom, double _top,
          double _near, double _far);

  /// \brief Computes the OpenGL perspective matrix
  /// \param[in] _intrinsicsFx Horizontal focal length (in pixels)
  /// \param[in] _intrinsicsFy Vertical focal length (in pixels)
  /// \param[in] _intrinsicsCx X coordinate of principal point in pixels
  /// \param[in] _intrinsicsCy Y coordinate of principal point in pixels
  /// \param[in] _intrinsicsS Skew coefficient defining the angle between
  ///            the x and y pixel axes
  /// \param[in] _clipNear Distance to the nearer depth clipping plane
  ///            This value is negative if the plane is to be behind
  ///            the camera
  /// \param[in] _clipFar Distance to the farther depth clipping plane
  ///            This value is negative if the plane is to be behind
  ///            the camera
  /// \return OpenGL perspective matrix
  public: static math::Matrix4d BuildPerspectiveMatrix(
          double _intrinsicsFx, double _intrinsicsFy,
          double _intrinsicsCx, double _intrinsicsCy,
          double _intrinsicsS,
          double _clipNear, double _clipFar);

  /// \brief Computes the OpenGL projection matrix by multiplying
  ///        the OpenGL Normalized Device Coordinates matrix (NDC) with
  ///        the OpenGL perspective matrix
  ///        openglProjectionMatrix = ndcMatrix * perspectiveMatrix
  /// \param[in] _imageWidth Image width (in pixels)
  /// \param[in] _imageHeight Image height (in pixels)
  /// \param[in] _intrinsicsFx Horizontal focal length (in pixels)
  /// \param[in] _intrinsicsFy Vertical focal length (in pixels)
  /// \param[in] _intrinsicsCx X coordinate of principal point in pixels
  /// \param[in] _intrinsicsCy Y coordinate of principal point in pixels
  /// \param[in] _intrinsicsS Skew coefficient defining the angle between
  ///             the x and y pixel axes
  /// \param[in] _clipNear Distance to the nearer depth clipping plane
  ///            This value is negative if the plane is to be behind
  ///            the camera
  /// \param[in] _clipFar Distance to the farther depth clipping plane
  ///            This value is negative if the plane is to be behind
  ///            the camera
  /// \return OpenGL projection matrix
  public: static math::Matrix4d BuildProjectionMatrix(
          double _imageWidth, double _imageHeight,
          double _intrinsicsFx, double _intrinsicsFy,
          double _intrinsicsCx, double _intrinsicsCy,
          double _intrinsicsS,
          double _clipNear, double _clipFar);

  /// \brief node to create publisher
  public: transport::Node node;

  /// \brief publisher to publish images
  public: transport::Node::Publisher pub;

  /// \brief Camera info publisher to publish images
  public: transport::Node::Publisher infoPub;

  /// \brief publisher to publish h254
  public: transport::Node::Publisher pubH264;

  /// \brief true if Load() has been called and was successful
  public: bool initialized = false;

  /// \brief Rendering camera
  public: gz::rendering::CameraPtr camera;

  /// \brief pool of pointers to an images to be published
  uint num_pixel_buffers = 16;
  uint current_pixel_buffer = 0;
  uint frame_data_size = 0;
  std::vector<std::vector<unsigned char>> pixel_buffers;

  /// \brief Noise added to sensor data
  public: std::map<SensorNoiseType, NoisePtr> noises;

  /// \brief Distortion added to sensor data
  public: DistortionPtr distortion;

  /// \brief Event that is used to trigger callbacks when a new image
  /// is generated
  public: gz::common::EventT<
          void(const gz::msgs::Image &)> imageEvent;

  /// \brief Connection to the Manager's scene change event.
  public: gz::common::ConnectionPtr sceneChangeConnection;

  /// \brief Just a mutex for thread safety
  public: std::mutex mutex;

  /// \brief True to save images
  public: bool saveImage = false;

  /// \brief path directory to where images are saved
  public: std::string saveImagePath = "";

  /// \prefix of an image name
  public: std::string saveImagePrefix = "";

  /// \brief counter used to set the image filename
  public: std::uint64_t saveImageCounter = 0;

  /// \brief SDF Sensor DOM object.
  public: sdf::Sensor sdfSensor;

  /// \brief Camera information message.
  public: msgs::CameraInfo infoMsg;

  /// \brief The frame this camera uses in its camera_info topic.
  public: std::string opticalFrameId{""};

  /// \brief Topic for info message.
  public: std::string infoTopic{""};

  /// \brief Topic for h264 message.
  public: std::string h264Topic{""};

  /// \brief Baseline for stereo cameras.
  public: double baseline{0.0};

  /// \brief Flag to indicate if sensor is generating data
  public: bool generatingData = false;

  public:
    std::shared_ptr<rclcpp::Node> directRosNode;
    std::string directRosNodeName = "gz_cameras_direct";
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> imagePub;
    std::shared_ptr<rclcpp::Publisher<ffmpeg_image_transport_msgs::msg::FFMPEGPacket>> h264Pub;
    std::shared_ptr<phntm::FFmpegEncoder> encoder;

    std::string encoderHwDevice = ""; // "cuda", "vaapi", "" = sw
    std::string camerasResolution = "";
    AVPixelFormat encoderForceInputPixelFormat = AVPixelFormat::AV_PIX_FMT_NONE; // overrides auto codec input pixel format selection
    int encoderThreadCount = 1;
    int encoderGOPSize = 60;
    int encoderBitRate = 1000000;
    bool encoderError = false;
    std::chrono::steady_clock::time_point last_debug_time;

    bool postRenderThreadRunning;
    std::thread postRenderThread;
    std::mutex postRenderMutex;
    std::condition_variable postRenderCV;
    std::queue<uint> postRenderQueue;

    bool hasImageConnections = false, hasH264Connections = false;
    std::chrono::steady_clock::duration now;

    EGLContext eglCtx = nullptr;
    EGLDisplay eglDisplay = nullptr;
    EGLSurface eglSurface = nullptr;

    EGLContext eglWorkerCtx = nullptr;
    bool eglWorkerCtxSet = false;

    std::shared_ptr<rclcpp::Service<phntm_interfaces::srv::SetFloat32>> set_pose_z_srv;
    std::shared_ptr<rclcpp::Service<phntm_interfaces::srv::GetFloat32>> get_pose_z_srv;
    bool pose_dirty;
    gz::math::Pose3d pose_to_set;
    // EGLSurface eglWorkerSurface = nullptr;

    void srvSetPoseZ(std::shared_ptr<phntm_interfaces::srv::SetFloat32::Request> req, std::shared_ptr<phntm_interfaces::srv::SetFloat32::Response> res);
    void srvGetPoseZ(std::shared_ptr<phntm_interfaces::srv::GetFloat32::Request>, std::shared_ptr<phntm_interfaces::srv::GetFloat32::Response> res);
    void onTfStatic(std::shared_ptr<tf2_msgs::msg::TFMessage> msg);
    std::shared_ptr<rclcpp::Publisher<tf2_msgs::msg::TFMessage>> tfStaticPub;
    std::shared_ptr<rclcpp::Subscription<tf2_msgs::msg::TFMessage>> tfStaticSub;
    std::shared_ptr<tf2_msgs::msg::TFMessage> lastTfStaticMsg = nullptr;
};

void CameraSensorPrivate::srvSetPoseZ(std::shared_ptr<phntm_interfaces::srv::SetFloat32::Request> req, std::shared_ptr<phntm_interfaces::srv::SetFloat32::Response> res) {
  this->pose_to_set.SetZ(req->data);
  this->pose_dirty = true;
  res->data = this->pose_to_set.Z(); // confirm the new value
  res->success = true;
}

void CameraSensorPrivate::srvGetPoseZ(std::shared_ptr<phntm_interfaces::srv::GetFloat32::Request>, std::shared_ptr<phntm_interfaces::srv::GetFloat32::Response> res) {
  res->data = this->pose_to_set.Z();
}

void CameraSensorPrivate::onTfStatic(std::shared_ptr<tf2_msgs::msg::TFMessage> msg) {
  std::cout <<  "Camera got TF static data" << std::endl;
  this->lastTfStaticMsg = msg;
}

//////////////////////////////////////////////////
bool CameraSensor::CreateCamera()
{
  sdf::Camera *cameraSdf = this->dataPtr->sdfSensor.CameraSensor();
  if (!cameraSdf)
  {
    gzerr << "Unable to access camera SDF element.\n";
    return false;
  }

  unsigned int width = cameraSdf->ImageWidth();
  unsigned int height = cameraSdf->ImageHeight();

  if (width == 0u || height == 0u)
  {
    gzerr << "Unable to create a camera sensor with 0 width or height."
          << std::endl;
    return false;
  }

  this->dataPtr->camera = this->Scene()->CreateCamera(this->Name());
  this->dataPtr->camera->SetImageWidth(width);
  this->dataPtr->camera->SetImageHeight(height);
  this->dataPtr->camera->SetNearClipPlane(cameraSdf->NearClip());
  this->dataPtr->camera->SetFarClipPlane(cameraSdf->FarClip());
  this->dataPtr->camera->SetVisibilityMask(cameraSdf->VisibilityMask());
  this->dataPtr->camera->SetLocalPose(this->Pose());
  this->AddSensor(this->dataPtr->camera);

  this->dataPtr->frame_data_size = width * height * 3;
  // unsigned char *data = new unsigned char[data_size];
  //std::vector<unsigned char> pixels(data_size);
  for (uint i = 0; i < this->dataPtr->num_pixel_buffers; i++) {
    std::vector<unsigned char> pixel_buffer(this->dataPtr->frame_data_size);
    this->dataPtr->pixel_buffers.push_back(pixel_buffer);
  }

  const std::map<SensorNoiseType, sdf::Noise> noises = {
    {CAMERA_NOISE, cameraSdf->ImageNoise()},
  };

  for (const auto & [noiseType, noiseSdf] : noises)
  {
    // Add gaussian noise to camera sensor
    if (noiseSdf.Type() == sdf::NoiseType::GAUSSIAN)
    {
      // Skip applying noise if mean and stddev are 0 - this avoids
      // doing an extra render pass in gz-rendering
      // Note ImageGaussianNoiseModel only uses mean and stddev and does not
      // use bias parameters.
      if (!math::equal(noiseSdf.Mean(), 0.0) ||
          !math::equal(noiseSdf.StdDev(), 0.0))
      {
        this->dataPtr->noises[noiseType] =
          ImageNoiseFactory::NewNoiseModel(noiseSdf, "camera");

        std::dynamic_pointer_cast<ImageGaussianNoiseModel>(
             this->dataPtr->noises[noiseType])->SetCamera(
               this->dataPtr->camera);
      }
    }
    else if (noiseSdf.Type() != sdf::NoiseType::NONE)
    {
      gzwarn << "The camera sensor only supports Gaussian noise. "
       << "The supplied noise type[" << static_cast<int>(noiseSdf.Type())
       << "] is not supported." << std::endl;
    }
  }

  // \todo(nkoeng) these parameters via sdf
  this->dataPtr->camera->SetAntiAliasing(cameraSdf->AntiAliasingValue());

  math::Angle angle = cameraSdf->HorizontalFov();
  if (angle < 0.001 || angle > GZ_PI*2)
  {
    gzerr << "Invalid horizontal field of view [" << angle << "]\n";

    return false;
  }
  this->dataPtr->camera->SetAspectRatio(static_cast<double>(width)/height);
  this->dataPtr->camera->SetHFOV(angle);

  if (cameraSdf->Element() != nullptr &&
      cameraSdf->Element()->HasElement("distortion"))
  {
    // Skip distortion of all coefficients are 0s
    if (!math::equal(cameraSdf->DistortionK1(), 0.0) ||
        !math::equal(cameraSdf->DistortionK2(), 0.0) ||
        !math::equal(cameraSdf->DistortionK3(), 0.0) ||
        !math::equal(cameraSdf->DistortionP1(), 0.0) ||
        !math::equal(cameraSdf->DistortionP2(), 0.0))
    {
      this->dataPtr->distortion =
          ImageDistortionFactory::NewDistortionModel(*cameraSdf, "camera");
      this->dataPtr->distortion->Load(*cameraSdf);

      std::dynamic_pointer_cast<ImageBrownDistortionModel>(
          this->dataPtr->distortion)->SetCamera(this->dataPtr->camera);
    }
  }

  sdf::PixelFormatType pixelFormat = cameraSdf->PixelFormat();
  switch (pixelFormat)
  {
    case sdf::PixelFormatType::RGB_INT8:
      this->dataPtr->camera->SetImageFormat(rendering::PF_R8G8B8);
      break;
    case sdf::PixelFormatType::BGR_INT8:
      this->dataPtr->camera->SetImageFormat(rendering::PF_B8G8R8);
      break;
    case sdf::PixelFormatType::L_INT8:
      this->dataPtr->camera->SetImageFormat(rendering::PF_L8);
      break;
    case sdf::PixelFormatType::L_INT16:
      this->dataPtr->camera->SetImageFormat(rendering::PF_L16);
      break;
    case sdf::PixelFormatType::BAYER_RGGB8:
      this->dataPtr->camera->SetImageFormat(rendering::PF_BAYER_RGGB8);
      break;
    case sdf::PixelFormatType::BAYER_BGGR8:
      this->dataPtr->camera->SetImageFormat(rendering::PF_BAYER_BGGR8);
      break;
    case sdf::PixelFormatType::BAYER_GBRG8:
      this->dataPtr->camera->SetImageFormat(rendering::PF_BAYER_GBRG8);
      break;
    case sdf::PixelFormatType::BAYER_GRBG8:
      
      break;
    default: {
      auto fmt_name = phntm::FFmpegEncoder::GetGZPixelFormatName(pixelFormat);
      RCLCPP_ERROR(this->dataPtr->directRosNode->get_logger(), "Rendering camera %s doesn't support selected pixel format: %s", this->Name().c_str(), fmt_name.c_str());
      break;
    }
  }

  this->UpdateLensIntrinsicsAndProjection(this->dataPtr->camera,
      *cameraSdf);

  this->Scene()->RootVisual()->AddChild(this->dataPtr->camera);

  // Create the directory to store frames
  if (cameraSdf->SaveFrames())
  {
    this->dataPtr->saveImagePath = cameraSdf->SaveFramesPath();
    this->dataPtr->saveImagePrefix = this->Name() + "_";
    this->dataPtr->saveImage = true;
  }

  // Populate camera info topic
  this->PopulateInfo(cameraSdf);

  this->dataPtr->postRenderThreadRunning = true;
  this->dataPtr->postRenderThread = std::thread(&CameraSensor::postRenderWorker, this);
  this->dataPtr->postRenderThread.detach();

  return true;
}

//////////////////////////////////////////////////
CameraSensor::CameraSensor()
  : dataPtr(new CameraSensorPrivate())
{
}

//////////////////////////////////////////////////
CameraSensor::~CameraSensor()
{
  if (this->dataPtr->directRosNode != nullptr) {
    this->dataPtr->imagePub.reset();
    this->dataPtr->h264Pub.reset();
    DirectRosNode::ReleaseDirectROSNode(this->dataPtr->directRosNodeName, this->dataPtr.get());
    this->dataPtr->directRosNode.reset();
    this->dataPtr->encoder.reset();
  }
  for (uint i = 0; i < this->dataPtr->num_pixel_buffers; i++) {
    this->dataPtr->pixel_buffers[i].clear();
  }
  this->dataPtr->pixel_buffers.clear();

  this->dataPtr->postRenderThreadRunning = false;
  this->dataPtr->postRenderCV.notify_one();

  if (this->Scene() && this->dataPtr->camera)
  {
    this->Scene()->DestroySensor(this->dataPtr->camera);
  }
}

//////////////////////////////////////////////////
bool CameraSensor::Init()
{
  return this->Sensor::Init();
}

//////////////////////////////////////////////////
bool CameraSensor::Load(const sdf::Sensor &_sdf)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  if (!Sensor::Load(_sdf))
  {
    return false;
  }

  // Check if this is the right type
  if (_sdf.Type() != sdf::SensorType::CAMERA)
  {
    gzerr << "Attempting to a load a Camera sensor, but received "
      << "a " << _sdf.TypeStr() << std::endl;
  }

  if (_sdf.CameraSensor() == nullptr)
  {
    gzerr << "Attempting to a load a Camera sensor, but received "
      << "a null sensor." << std::endl;
    return false;
  }

  this->dataPtr->sdfSensor = _sdf;
  
  // if (this->Topic().empty())
  //   this->SetTopic("/camera");

  auto sdf_camera = _sdf.Element()->GetElement("camera");
  
  std::cout << "Camera [" << this->Name() << "] getting direct ROS node" << std::endl;
  this->dataPtr->directRosNode = DirectRosNode::GetDirectROSNode(this->dataPtr->directRosNodeName, this->dataPtr.get());
  if (this->dataPtr->directRosNode == nullptr) {
     gzerr << "Failed creating direct ROS node for Camera sensor [" << this->Name() << "]" << std::endl;
    return false;
  }

  // direct uncompressed output
  if (!this->Topic().empty()) {
    rclcpp::QoS qos(1);
    // qos.best_effort();
    // qos.transient_local();
    this->dataPtr->imagePub = this->dataPtr->directRosNode->create_publisher<sensor_msgs::msg::Image>(this->Topic(), qos);
  }

  // direct h264 compressed output
  this->dataPtr->h264Topic = sdf_camera->HasElement("camera_h264_topic") ? sdf_camera->GetElement("camera_h264_topic")->GetValue()->GetAsString() : "";
  if (!this->dataPtr->h264Topic.empty()) {
    rclcpp::QoS qos(10);
    // qos.best_effort();
    this->dataPtr->h264Pub = this->dataPtr->directRosNode->create_publisher<ffmpeg_image_transport_msgs::msg::FFMPEGPacket>(this->dataPtr->h264Topic, qos);
  }

  if (sdf_camera->HasElement("encoder_hw_device")) {
    this->dataPtr->encoderHwDevice = sdf_camera->GetElement("encoder_hw_device")->GetValue()->GetAsString();
    gzdbg << "Camera [" << this->Name() << "] setting encoder_hw_device to '" << this->dataPtr->encoderHwDevice << "'" << std::endl;
    if (this->dataPtr->encoderHwDevice == "sw")
      this->dataPtr->encoderHwDevice = "";
  }

  if (sdf_camera->HasElement("encoder_thread_count")) {
    this->dataPtr->encoderThreadCount = std::stoi(sdf_camera->GetElement("encoder_thread_count")->GetValue()->GetAsString());
    gzdbg << "Camera [" << this->Name() << "] setting encoder_thread_count to '" << this->dataPtr->encoderThreadCount << "'" << std::endl;
  }
  
  if (sdf_camera->HasElement("encoder_bit_rate")) {
    this->dataPtr->encoderBitRate = std::stoi(sdf_camera->GetElement("encoder_bit_rate")->GetValue()->GetAsString());
    gzdbg << "Camera [" << this->Name() << "] setting encoder_bit_rate to '" << this->dataPtr->encoderBitRate << "'" << std::endl;
  }

  if (sdf_camera->HasElement("set_pose_srv_z") && !sdf_camera->GetElement("set_pose_srv_z")->GetValue()->GetAsString().empty()) {
    this->dataPtr->pose_to_set = this->Pose();
    this->dataPtr->pose_dirty = true; // produce /tf on start
    auto srv_name = "/" + this->dataPtr->directRosNodeName + "/" + sdf_camera->GetElement("set_pose_srv_z")->GetValue()->GetAsString();
    this->dataPtr->set_pose_z_srv = this->dataPtr->directRosNode->create_service<phntm_interfaces::srv::SetFloat32>(srv_name,
                                                                                   std::bind(&CameraSensorPrivate::srvSetPoseZ, this->dataPtr.get(), std::placeholders::_1, std::placeholders::_2));
  }

  if (sdf_camera->HasElement("get_pose_srv_z") && !sdf_camera->GetElement("get_pose_srv_z")->GetValue()->GetAsString().empty()) {
    auto srv_name = "/" + this->dataPtr->directRosNodeName + "/" + sdf_camera->GetElement("get_pose_srv_z")->GetValue()->GetAsString();
    this->dataPtr->get_pose_z_srv = this->dataPtr->directRosNode->create_service<phntm_interfaces::srv::GetFloat32>(srv_name,
                                                                                   std::bind(&CameraSensorPrivate::srvGetPoseZ, this->dataPtr.get(), std::placeholders::_1, std::placeholders::_2));
  }

  if (sdf_camera->HasElement("tf_static_topic") && !sdf_camera->GetElement("tf_static_topic")->GetValue()->GetAsString().empty()) {
    rclcpp::QoS qos(1);
    qos.reliable();
    qos.transient_local();
    this->dataPtr->tfStaticPub = this->dataPtr->directRosNode->create_publisher<tf2_msgs::msg::TFMessage>(sdf_camera->GetElement("tf_static_topic")->GetValue()->GetAsString(), qos);
    this->dataPtr->tfStaticSub = this->dataPtr->directRosNode->create_subscription<tf2_msgs::msg::TFMessage>(sdf_camera->GetElement("tf_static_topic")->GetValue()->GetAsString(), qos,
                                                                std::bind(&CameraSensorPrivate::onTfStatic, this->dataPtr.get(), std::placeholders::_1));
  }

  if (sdf_camera->HasElement("encoder_input_pixel_format") && sdf_camera->GetElement("encoder_input_pixel_format")->GetValue()
      && !sdf_camera->GetElement("encoder_input_pixel_format")->GetValue()->GetAsString().empty()) {
    auto str_val = sdf_camera->GetElement("encoder_input_pixel_format")->GetValue()->GetAsString();
    std::unordered_map<std::string, AVPixelFormat> map = {
      {"yuv420p", AVPixelFormat::AV_PIX_FMT_YUV420P},
      {"yuvj420p", AVPixelFormat::AV_PIX_FMT_YUVJ420P},
      {"yuv422p", AVPixelFormat::AV_PIX_FMT_YUV422P},
      {"yuvj422p", AVPixelFormat::AV_PIX_FMT_YUVJ422P},
      {"yuv444p", AVPixelFormat::AV_PIX_FMT_YUV444P},
      {"yuvj444p", AVPixelFormat::AV_PIX_FMT_YUVJ444P},
      {"nv12", AVPixelFormat::AV_PIX_FMT_NV12},
      {"nv16", AVPixelFormat::AV_PIX_FMT_NV16},
      {"nv21", AVPixelFormat::AV_PIX_FMT_NV21},
      {"yuv420p10le", AVPixelFormat::AV_PIX_FMT_YUV420P10LE},
      {"yuv422p10le", AVPixelFormat::AV_PIX_FMT_YUV422P10LE},
      {"yuv444p10le", AVPixelFormat::AV_PIX_FMT_YUV444P10LE},
      {"nv20le", AVPixelFormat::AV_PIX_FMT_NV20LE},
      {"gray8", AVPixelFormat::AV_PIX_FMT_GRAY8},
      {"gray10le", AVPixelFormat::AV_PIX_FMT_GRAY10LE},
      {"gray16", AVPixelFormat::AV_PIX_FMT_GRAY16},
      {"rgb0", AVPixelFormat::AV_PIX_FMT_RGB0},
      {"bgr0", AVPixelFormat::AV_PIX_FMT_BGR0},
      {"vaapi", AVPixelFormat::AV_PIX_FMT_VAAPI},
    };

    if (map.find(str_val) != map.end()) {
      gzdbg << "Camera [" << this->Name() << "] setting encoder_force_input_pixel_format to '" << str_val << "'" << std::endl;
      this->dataPtr->encoderForceInputPixelFormat = map.at(str_val);
    } else {
      gzdbg << "Camera [" << this->Name() << "] invalid encoder_force_input_pixel_format provided: '" << str_val << "', supported values are: " << std::endl;
      for (const auto& pair : map) {
        gzdbg << pair.first << std::endl;
      }
      gzdbg << std::endl;
    }
  }

  // via gz-ros-bridge
  if (!_sdf.CameraSensor()->CameraInfoTopic().empty())
  {
    this->dataPtr->infoTopic = _sdf.CameraSensor()->CameraInfoTopic();
  }

 
  gzdbg << "Camera images for [" << this->Name() << "] advertised on ROS topic ["
         << this->Topic() << "]" << std::endl;

  if (_sdf.CameraSensor()->Triggered())
  {
    std::string triggerTopic = _sdf.CameraSensor()->TriggerTopic();
    if (triggerTopic.empty())
    {
      triggerTopic = transport::TopicUtils::AsValidTopic(this->Topic() +
                                                         "/trigger");
    }
    this->SetTriggered(true, triggerTopic);
  }

  if (!this->AdvertiseInfo())
    return false;


  if (this->Scene())
    this->CreateCamera();

  this->dataPtr->sceneChangeConnection =
      RenderingEvents::ConnectSceneChangeCallback(
      std::bind(&CameraSensor::SetScene, this, std::placeholders::_1));

  this->dataPtr->initialized = true;
  return true;
}

//////////////////////////////////////////////////
bool CameraSensor::Load(sdf::ElementPtr _sdf)
{
  sdf::Sensor sdfSensor;
  sdfSensor.Load(_sdf);
  return this->Load(sdfSensor);
}

/////////////////////////////////////////////////
gz::common::ConnectionPtr CameraSensor::ConnectImageCallback(
    std::function<void(const gz::msgs::Image &)> _callback)
{
  return this->dataPtr->imageEvent.Connect(_callback);
}

/////////////////////////////////////////////////
void CameraSensor::SetScene(gz::rendering::ScenePtr _scene)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  // APIs make it possible for the scene pointer to change
  if (this->Scene() != _scene)
  {
    // TODO(anyone) Remove camera from scene
    this->dataPtr->camera = nullptr;
    RenderingSensor::SetScene(_scene);
    if (this->dataPtr->initialized)
      this->CreateCamera();
  }
}

std::string getThreadId() {
    std::ostringstream oss;
    oss << std::this_thread::get_id();
    return oss.str();
}

//////////////////////////////////////////////////
bool CameraSensor::Update(const std::chrono::steady_clock::duration &_now)
{
  GZ_PROFILE("CameraSensor::Update");
  if (!this->dataPtr->initialized)
  {
    gzerr << "Not initialized, update ignored.\n";
    return false;
  }

  if (!this->dataPtr->camera)
  {
    gzerr << "Camera doesn't exist.\n";
    return false;
  }

  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  if (this->HasInfoConnections())
  {
    // publish the camera info message
    this->PublishInfo(_now);
  }

  this->dataPtr->hasImageConnections = this->HasImageConnections();
  this->dataPtr->hasH264Connections = this->HasH264Connections();
  this->dataPtr->now = _now;

  if (!this->dataPtr->hasImageConnections && !this->dataPtr->hasH264Connections)
  {
    if (this->dataPtr->generatingData)
    {
      gzdbg << "Disabling camera sensor: '" << this->Name()
            << "' data generation. " << std::endl;
      this->dataPtr->generatingData = false;
    }
    if (!this->dataPtr->pose_dirty) // only exit here if we're not updating camera pose
      return true;
  }
  else
  {
    if (!this->dataPtr->generatingData)
    {
      gzdbg << "Enabling camera sensor: '" << this->Name()
            << "' data generation." << std::endl;
      this->dataPtr->generatingData = true;
    }
  }

  bool generate_pos_update = false;
  if (this->dataPtr->pose_dirty) {
    std::cout << "Setting camera " << this->Name() << " pose to " << this->dataPtr->pose_to_set << std::endl;
    this->dataPtr->camera->SetLocalPose(this->dataPtr->pose_to_set);
    generate_pos_update = true;
  }

  if (this->dataPtr->hasImageConnections || this->dataPtr->hasH264Connections)
  {

    {
      std::lock_guard<std::mutex> render_lock(this->dataPtr->postRenderMutex);
      this->Render();
      // this->dataPtr->camera->Copy(*image_buffer); // copy here

      uint gl_id = this->RenderingCamera()->RenderTextureGLId();

      // get context and display from ogre2
      if (this->dataPtr->eglWorkerCtx == nullptr) {
        this->dataPtr->eglCtx = eglGetCurrentContext();
        if (this->dataPtr->eglCtx == EGL_NO_CONTEXT) {
            std::cout << this->Name() <<  " Error getting EGL context" << std::endl;
            return false;
        }
        this->dataPtr->eglDisplay = eglGetCurrentDisplay();
        if (this->dataPtr->eglDisplay == EGL_NO_DISPLAY) {
            std::cout << this->Name() << " Error getting EGL display" << std::endl;
            return false;
        }       
        this->dataPtr->eglSurface = eglGetCurrentSurface(EGL_DRAW);

        std::cout << this->Name() << " Creating worker shared ctx" << std::endl;
        EGLConfig config;
        EGLint numConfigs = 0;
        
        EGLint cfg_attribs[] = {
          EGL_RENDERABLE_TYPE, EGL_OPENGL_ES3_BIT, // or _ES2_BIT if only ES2 supported
          EGL_SURFACE_TYPE, EGL_PBUFFER_BIT,       // allow off‑screen surfaces
          EGL_RED_SIZE,   8,
          EGL_GREEN_SIZE, 8,
          EGL_BLUE_SIZE,  8,
          EGL_ALPHA_SIZE, 8,
          EGL_NONE
        };
        eglChooseConfig(this->dataPtr->eglDisplay, cfg_attribs, &config, 1, &numConfigs);
        if (numConfigs < 1) {
            std::cout << this->Name() + " No matching EGLConfig found" << std::endl;
            return false;
        }
        EGLint ctx_attribs[] = {
          EGL_CONTEXT_CLIENT_VERSION, 3,
          EGL_NONE
        };
        this->dataPtr->eglWorkerCtx = eglCreateContext(this->dataPtr->eglDisplay, config, this->dataPtr->eglCtx, ctx_attribs);
        if (this->dataPtr->eglWorkerCtx == EGL_NO_CONTEXT) {
            std::cout << this->Name() + " Error creating worker EGL context" << std::endl;
            EGLint err = eglGetError();
            std::cout << "eglCreateContext failed with 0x" << std::hex << err << std::endl;
            return false;
        }
        
        //this->dataPtr->eglWorkerSurface = eglCreatePbufferSurface(this->dataPtr->eglDisplay, config, {});
      }
      this->dataPtr->postRenderQueue.push(gl_id);  
    }

    this->dataPtr->postRenderCV.notify_one();
  }

  // using /tf_static (reliable), but need to receive one first, then we only produce an update
  // the received tf message needs to include this camera's link
  if (generate_pos_update && this->dataPtr->lastTfStaticMsg && this->dataPtr->tfStaticPub) {

      tf2_msgs::msg::TFMessage *msg = this->dataPtr->lastTfStaticMsg.get();

      bool update = false;
      for (size_t i = 0; i < msg->transforms.size(); i++) {

          geometry_msgs::msg::TransformStamped *t = &msg->transforms[i];
          if (t->child_frame_id == this->OpticalFrameId()) {
            DirectRosNode::SetCurrentStamp(&t->header.stamp, this->dataPtr->now);
            t->transform.translation.z = this->dataPtr->pose_to_set.Pos().Z();
            update = true;
          }
      }
      
      if (update) {
        std::cout << "Updating camera " << this->Name() << " pose to into tf_static " << std::endl;
        this->dataPtr->pose_dirty = false;
        if (rclcpp::ok()) 
          this->dataPtr->tfStaticPub->publish(*msg);
      }
  }

  return true;
}

void CameraSensor::postRenderWorker() {
  std::cout << this->Name() << " POST-RENDER WORKER RUNNING" << std::endl;
  
  while (this->dataPtr->postRenderThreadRunning) {

    std::unique_lock<std::mutex> render_lock(this->dataPtr->postRenderMutex);
    this->dataPtr->postRenderCV.wait(render_lock, [this] { return !this->dataPtr->postRenderQueue.empty() || !this->dataPtr->postRenderThreadRunning; });

    if (!this->dataPtr->postRenderThreadRunning || this->dataPtr->postRenderQueue.empty() || this->dataPtr->eglWorkerCtx == nullptr || this->dataPtr->eglWorkerCtx == EGL_NO_CONTEXT) {
      render_lock.unlock();
      break;
    }
      
    uint gl_texture_id = this->dataPtr->postRenderQueue.front();
    this->dataPtr->postRenderQueue.pop();

    if (!this->dataPtr->eglWorkerCtxSet) {
      std::cout << this->Name() << " POST-RENDER Setting worker ctx" << std::endl;
      eglMakeCurrent(this->dataPtr->eglDisplay, EGL_NO_SURFACE, EGL_NO_SURFACE, this->dataPtr->eglWorkerCtx);
      this->dataPtr->eglWorkerCtxSet = true;
    }

    GLboolean is_texture = glIsTexture(gl_texture_id);
    if (!is_texture) {
      std::cout << this->Name() << " Invalid texture ID: " << gl_texture_id << std::endl;
      render_lock.unlock();
      continue;
    }

    // glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT | GL_TEXTURE_FETCH_BARRIER_BIT);
    // glFinish(); // ensure complete

    // Read back the texture data
    GLuint fbo;
    glGenFramebuffers(1, &fbo);
    glBindFramebuffer(GL_FRAMEBUFFER, fbo);

    // Attach Y texture to framebuffer
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, gl_texture_id, 0);

    // Check framebuffer status
    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
        std::cout << this->Name() <<  " Framebuffer not complete for texture readback" << std::endl;
        glDeleteFramebuffers(1, &fbo);
        render_lock.unlock();
        continue;
    }

    unsigned int width = this->dataPtr->camera->ImageWidth();
    unsigned int height = this->dataPtr->camera->ImageHeight();

    // Set viewport
    glViewport(0, 0, width, height);

    // Read pixels
    auto pixel_buffer = &this->dataPtr->pixel_buffers[this->dataPtr->current_pixel_buffer];
    this->dataPtr->current_pixel_buffer++;
    if (this->dataPtr->current_pixel_buffer == this->dataPtr->num_pixel_buffers) {
      this->dataPtr->current_pixel_buffer = 0;
    }
    glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, pixel_buffer->data());

    // Clean up
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glDeleteFramebuffers(1, &fbo);

    render_lock.unlock();

    std::string camera_image_format = "";
    AVPixelFormat opencv_format = AV_PIX_FMT_NONE;
    AVPixelFormat codec_input_format = AV_PIX_FMT_NV12;
    switch (this->dataPtr->camera->ImageFormat())
    {
      case rendering::PF_R8G8B8:
        camera_image_format = "rgb8";
        opencv_format = AVPixelFormat::AV_PIX_FMT_RGB24;
        codec_input_format = AVPixelFormat::AV_PIX_FMT_NV12;
        break;
      case rendering::PF_B8G8R8:
        camera_image_format = "bgr8";
        opencv_format = AVPixelFormat::AV_PIX_FMT_BGR24;
        codec_input_format = AVPixelFormat::AV_PIX_FMT_NV12;
        break;
      case rendering::PF_L8:
        camera_image_format = "mono8";
        opencv_format = AVPixelFormat::AV_PIX_FMT_GRAY8;
        codec_input_format = AVPixelFormat::AV_PIX_FMT_GRAY8;
        break;
      case rendering::PF_L16:
        camera_image_format = "mono16";
        opencv_format = AVPixelFormat::AV_PIX_FMT_GRAY16;
        codec_input_format = AVPixelFormat::AV_PIX_FMT_GRAY16;
        break;
      case rendering::PF_BAYER_RGGB8:
        camera_image_format = "rggb8";
        break;
      case rendering::PF_BAYER_BGGR8:
        camera_image_format = "bggr8";
        break;
      case rendering::PF_BAYER_GBRG8:
        camera_image_format = "gbrg8";
        break;
      case rendering::PF_BAYER_GRBG8:
        camera_image_format = "grbg8";
        break;
      default:
        break;
    }
    if (camera_image_format.empty()) {
      gzerr << "Unsupported pixel format [" << this->dataPtr->camera->ImageFormat() << "]" << " \n";
      continue;
    }
    if (this->dataPtr->encoderForceInputPixelFormat != AVPixelFormat::AV_PIX_FMT_NONE) {
      codec_input_format = this->dataPtr->encoderForceInputPixelFormat;
    }

    if (this->dataPtr->hasH264Connections) {

       // make encoder
        if (this->dataPtr->encoder.get() == nullptr && !this->dataPtr->encoderError) {

          std::cout << "Camera [" << this->Name() << "] output image format = " << camera_image_format << std::endl;

          RCLCPP_INFO(this->dataPtr->directRosNode->get_logger(), "Making encoder %dx%d for %s with hw_device=%s",
                      width, height, this->H264Topic().c_str(), this->dataPtr->encoderHwDevice.c_str());
          try {
              this->dataPtr->encoder = std::make_shared<phntm::FFmpegEncoder>(width, height,
                                              camera_image_format, opencv_format, codec_input_format,
                                              this->OpticalFrameId(), this->H264Topic(), this->dataPtr->directRosNode,
                                              this->dataPtr->encoderHwDevice,
                                              this->dataPtr->encoderThreadCount,
                                              this->dataPtr->encoderGOPSize,
                                              this->dataPtr->encoderBitRate,
                                              std::bind(&CameraSensor::onEncodedFrame, this, std::placeholders::_1));
          } catch (const std::runtime_error & ex) {
              this->dataPtr->encoder.reset();
              // this->encoder_error = true;
              std::cout << "Error making encoder" << std::endl;
              RCLCPP_ERROR(this->dataPtr->directRosNode->get_logger(), "%s", ex.what());
              this->dataPtr->encoderError = true;
          }
        }
        if (this->dataPtr->encoder.get() == nullptr) {
          this->dataPtr->encoderError = true;
        } else {
          cv::Mat frame;
          switch (this->dataPtr->camera->ImageFormat())
          {
            case rendering::PF_R8G8B8:
              frame = cv::Mat(height, width, CV_8UC3, pixel_buffer->data()); // no copy
              break;
            case rendering::PF_B8G8R8:
              frame = cv::Mat(height, width, CV_8UC3, pixel_buffer->data()); // no copy
              break;
            case rendering::PF_L8: 
              frame = cv::Mat(height, width, CV_8UC1, pixel_buffer->data()); // no copy
              break;
            case rendering::PF_L16: {
              frame = cv::Mat(height, width, CV_16UC1, pixel_buffer->data()); // no copy
              break;
            }
            default:
              RCLCPP_ERROR(this->dataPtr->directRosNode->get_logger(), "[%s] Received unsupported pixel format from camera %d", this->Name().c_str(), this->dataPtr->camera->ImageFormat());
              break;
          }

          std_msgs::msg::Header header;
          header = std_msgs::msg::Header();
          header.frame_id = this->dataPtr->opticalFrameId;
          DirectRosNode::SetCurrentStamp(&header.stamp, this->dataPtr->now);
          this->dataPtr->encoder->encodeFrame(frame, header);
        }
    }

    if (this->dataPtr->hasImageConnections) {
      // create ROS raw message
      sensor_msgs::msg::Image msg;
      {
        GZ_PROFILE("CameraSensor::Update Message");
        msg.header = std_msgs::msg::Header();
        msg.header.frame_id = this->dataPtr->opticalFrameId;
        DirectRosNode::SetCurrentStamp(&msg.header.stamp, this->dataPtr->now);
        msg.encoding = camera_image_format;
        msg.width = width;
        msg.height = height;
        msg.data.assign(pixel_buffer->data(), pixel_buffer->data() + this->dataPtr->frame_data_size);
       
        GZ_PROFILE("CameraSensor::Update Publish");
        if (rclcpp::ok()) 
          this->dataPtr->imagePub->publish(msg);
      }
    }


  }
  std::cout << this->Name() << " POST-RENDER WORKER DONE" << std::endl;
}

// on subscriber thread
void CameraSensor::onEncodedFrame(const std::shared_ptr<ffmpeg_image_transport_msgs::msg::FFMPEGPacket> msg) {
    // ffmpeg_image_transport_msgs::msg::FFMPEGPacket msg_out = 
    // std::cout << this->Name() << " publishing encoded frame" << std::endl;
    this->dataPtr->h264Pub->publish(*msg.get());
}

//////////////////////////////////////////////////
bool CameraSensorPrivate::SaveImage(const unsigned char *_data,
    unsigned int _width, unsigned int _height,
    gz::common::Image::PixelFormatType _format)
{
  // Attempt to create the directory if it doesn't exist
  if (!gz::common::isDirectory(this->saveImagePath))
  {
    if (!gz::common::createDirectories(this->saveImagePath))
      return false;
  }

  std::string filename = this->saveImagePrefix +
                         std::to_string(this->saveImageCounter) + ".png";
  ++this->saveImageCounter;

  gz::common::Image localImage;
  localImage.SetFromData(_data, _width, _height, _format);

  localImage.SavePNG(
      gz::common::joinPaths(this->saveImagePath, filename));
  return true;
}

//////////////////////////////////////////////////
unsigned int CameraSensor::ImageWidth() const
{
  if (this->dataPtr->camera)
    return this->dataPtr->camera->ImageWidth();
  return 0;
}

//////////////////////////////////////////////////
unsigned int CameraSensor::ImageHeight() const
{
  if (this->dataPtr->camera)
    return this->dataPtr->camera->ImageHeight();
  return 0;
}

//////////////////////////////////////////////////
rendering::CameraPtr CameraSensor::RenderingCamera() const
{
  return this->dataPtr->camera;
}

//////////////////////////////////////////////////
std::string CameraSensor::InfoTopic() const
{
  return this->dataPtr->infoTopic;
}

//////////////////////////////////////////////////
std::string CameraSensor::H264Topic() const
{
  return this->dataPtr->h264Topic;
}

//////////////////////////////////////////////////
bool CameraSensor::AdvertiseInfo()
{
  if (this->dataPtr->infoTopic.empty())
  {
    auto parts = common::Split(this->Topic(), '/');
    parts.pop_back();
    for (const auto &part : parts)
    {
      if (!part.empty())
        this->dataPtr->infoTopic += "/" + part;
    }
    this->dataPtr->infoTopic += "/camera_info";
  }

  return this->AdvertiseInfo(this->dataPtr->infoTopic);
}

//////////////////////////////////////////////////
bool CameraSensor::AdvertiseInfo(const std::string &_topic)
{
  this->dataPtr->infoTopic = _topic;

  this->dataPtr->infoPub =
      this->dataPtr->node.Advertise<gz::msgs::CameraInfo>(this->dataPtr->infoTopic);
  if (!this->dataPtr->infoPub)
  {
    gzerr << "Unable to create publisher on topic ["
      << this->dataPtr->infoTopic << "].\n";
  }
  else
  {
    gzdbg << "Camera info for [" << this->Name() << "] advertised on ["
           << this->dataPtr->infoTopic << "]" << std::endl;
  }

  return this->dataPtr->infoPub;
}

//////////////////////////////////////////////////
void CameraSensor::PublishInfo(
  const std::chrono::steady_clock::duration &_now)
{
  *this->dataPtr->infoMsg.mutable_header()->mutable_stamp() =
    msgs::Convert(_now);
  this->dataPtr->infoPub.Publish(this->dataPtr->infoMsg);
}

//////////////////////////////////////////////////
void CameraSensor::PopulateInfo(const sdf::Camera *_cameraSdf)
{
  unsigned int width = _cameraSdf->ImageWidth();
  unsigned int height = _cameraSdf->ImageHeight();

  msgs::CameraInfo::Distortion *distortion =
    this->dataPtr->infoMsg.mutable_distortion();

  distortion->set_model(msgs::CameraInfo::Distortion::PLUMB_BOB);
  distortion->add_k(_cameraSdf->DistortionK1());
  distortion->add_k(_cameraSdf->DistortionK2());
  distortion->add_k(_cameraSdf->DistortionP1());
  distortion->add_k(_cameraSdf->DistortionP2());
  distortion->add_k(_cameraSdf->DistortionK3());

  msgs::CameraInfo::Intrinsics *intrinsics =
    this->dataPtr->infoMsg.mutable_intrinsics();

  intrinsics->add_k(_cameraSdf->LensIntrinsicsFx());
  intrinsics->add_k(0.0);
  intrinsics->add_k(_cameraSdf->LensIntrinsicsCx());

  intrinsics->add_k(0.0);
  intrinsics->add_k(_cameraSdf->LensIntrinsicsFy());
  intrinsics->add_k(_cameraSdf->LensIntrinsicsCy());

  intrinsics->add_k(0.0);
  intrinsics->add_k(0.0);
  intrinsics->add_k(1.0);

  msgs::CameraInfo::Projection *proj =
    this->dataPtr->infoMsg.mutable_projection();

  proj->add_p(_cameraSdf->LensProjectionFx());
  proj->add_p(0.0);
  proj->add_p(_cameraSdf->LensProjectionCx());
  proj->add_p(_cameraSdf->LensProjectionTx());

  proj->add_p(0.0);
  proj->add_p(_cameraSdf->LensProjectionFy());
  proj->add_p(_cameraSdf->LensProjectionCy());
  proj->add_p(_cameraSdf->LensProjectionTy());

  proj->add_p(0.0);
  proj->add_p(0.0);
  proj->add_p(1.0);
  proj->add_p(0.0);

  // Set the rectification matrix to identity
  this->dataPtr->infoMsg.add_rectification_matrix(1.0);
  this->dataPtr->infoMsg.add_rectification_matrix(0.0);
  this->dataPtr->infoMsg.add_rectification_matrix(0.0);

  this->dataPtr->infoMsg.add_rectification_matrix(0.0);
  this->dataPtr->infoMsg.add_rectification_matrix(1.0);
  this->dataPtr->infoMsg.add_rectification_matrix(0.0);

  this->dataPtr->infoMsg.add_rectification_matrix(0.0);
  this->dataPtr->infoMsg.add_rectification_matrix(0.0);
  this->dataPtr->infoMsg.add_rectification_matrix(1.0);

  // Note: while Gazebo interprets the camera frame to be looking towards +X,
  // other tools, such as ROS, may interpret this frame as looking towards +Z.
  // To make this configurable the user has the option to set an optical frame.
  // If the user has set <optical_frame_id> in the cameraSdf use it,
  // otherwise fall back to the sensor frame.
  if (_cameraSdf->OpticalFrameId().empty())
  {
   this->dataPtr->opticalFrameId = this->FrameId();
  }
  else
  {
   this->dataPtr->opticalFrameId = _cameraSdf->OpticalFrameId();
  }
  auto infoFrame = this->dataPtr->infoMsg.mutable_header()->add_data();
  infoFrame->set_key("frame_id");
  infoFrame->add_value(this->dataPtr->opticalFrameId);

  this->dataPtr->infoMsg.set_width(width);
  this->dataPtr->infoMsg.set_height(height);
}

//////////////////////////////////////////////////
void CameraSensor::SetBaseline(double _baseline)
{
  this->dataPtr->baseline = _baseline;

  // Also update message
  if (this->dataPtr->infoMsg.has_projection() &&
      this->dataPtr->infoMsg.projection().p_size() == 12)
  {
    auto fx = this->dataPtr->infoMsg.projection().p(0);
    this->dataPtr->infoMsg.mutable_projection()->set_p(3, -fx * _baseline);
  }
}

//////////////////////////////////////////////////
double CameraSensor::Baseline() const
{
  return this->dataPtr->baseline;
}

//////////////////////////////////////////////////
bool CameraSensor::HasConnections() const
{
  return this->HasImageConnections() || this->HasH264Connections() || this->HasInfoConnections();
}

//////////////////////////////////////////////////
//////////////////////////////////////////////////
bool CameraSensor::HasImageConnections() const
{
  return (this->dataPtr->imagePub && this->dataPtr->imagePub->get_subscription_count() > 0);
}

//////////////////////////////////////////////////
bool CameraSensor::HasH264Connections() const
{
  return (this->dataPtr->h264Pub && this->dataPtr->h264Pub->get_subscription_count() > 0);
}

//////////////////////////////////////////////////
bool CameraSensor::HasInfoConnections() const
{
  return this->dataPtr->infoPub && this->dataPtr->infoPub.HasConnections();
}

//////////////////////////////////////////////////
const std::string& CameraSensor::OpticalFrameId() const
{
  return this->dataPtr->opticalFrameId;
}

//////////////////////////////////////////////////
void CameraSensor::UpdateLensIntrinsicsAndProjection(
  rendering::CameraPtr _camera, sdf::Camera &_cameraSdf)
{
  // Update the DOM object intrinsics to have consistent
  // intrinsics between ogre camera and camera_info msg
  if(!_cameraSdf.HasLensIntrinsics())
  {
    auto intrinsicMatrix =
      gz::rendering::projectionToCameraIntrinsic(
        _camera->ProjectionMatrix(),
        _camera->ImageWidth(),
        _camera->ImageHeight()
      );

    _cameraSdf.SetLensIntrinsicsFx(intrinsicMatrix(0, 0));
    _cameraSdf.SetLensIntrinsicsFy(intrinsicMatrix(1, 1));
    _cameraSdf.SetLensIntrinsicsCx(intrinsicMatrix(0, 2));
    _cameraSdf.SetLensIntrinsicsCy(intrinsicMatrix(1, 2));
  }
  // set custom projection matrix based on intrinsics param specified in sdf
  else
  {
    double fx = _cameraSdf.LensIntrinsicsFx();
    double fy = _cameraSdf.LensIntrinsicsFy();
    double cx = _cameraSdf.LensIntrinsicsCx();
    double cy = _cameraSdf.LensIntrinsicsCy();
    double s = _cameraSdf.LensIntrinsicsSkew();
    auto projectionMatrix = CameraSensorPrivate::BuildProjectionMatrix(
        _camera->ImageWidth(),
        _camera->ImageHeight(),
        fx, fy, cx, cy, s,
        _camera->NearClipPlane(),
        _camera->FarClipPlane());
    _camera->SetProjectionMatrix(projectionMatrix);
  }

  // Update the DOM object intrinsics to have consistent
  // projection matrix values between ogre camera and camera_info msg
  // If these values are not defined in the SDF then we need to update
  // these values to something reasonable. The projection matrix is
  // the cumulative effect of intrinsic and extrinsic parameters
  if(!_cameraSdf.HasLensProjection())
  {
    // Note that the matrix from Ogre via camera->ProjectionMatrix() has a
    // different format than the projection matrix used in SDFormat.
    // This is why they are converted using projectionToCameraIntrinsic.
    // The resulting matrix is the intrinsic matrix, but since the user has
    // not overridden the values, this is also equal to the projection matrix.
    auto intrinsicMatrix =
      gz::rendering::projectionToCameraIntrinsic(
        _camera->ProjectionMatrix(),
        _camera->ImageWidth(),
        _camera->ImageHeight()
      );
    _cameraSdf.SetLensProjectionFx(intrinsicMatrix(0, 0));
    _cameraSdf.SetLensProjectionFy(intrinsicMatrix(1, 1));
    _cameraSdf.SetLensProjectionCx(intrinsicMatrix(0, 2));
    _cameraSdf.SetLensProjectionCy(intrinsicMatrix(1, 2));
  }
  // set custom projection matrix based on projection param specified in sdf
  else
  {
    // tx and ty are not used
    double fx = _cameraSdf.LensProjectionFx();
    double fy = _cameraSdf.LensProjectionFy();
    double cx = _cameraSdf.LensProjectionCx();
    double cy = _cameraSdf.LensProjectionCy();
    double s = 0;

    auto projectionMatrix = CameraSensorPrivate::BuildProjectionMatrix(
        _camera->ImageWidth(),
        _camera->ImageHeight(),
        fx, fy, cx, cy, s,
        _camera->NearClipPlane(),
        _camera->FarClipPlane());
    _camera->SetProjectionMatrix(projectionMatrix);
  }
}

//////////////////////////////////////////////////
math::Matrix4d CameraSensorPrivate::BuildProjectionMatrix(
    double _imageWidth, double _imageHeight,
    double _intrinsicsFx, double _intrinsicsFy,
    double _intrinsicsCx, double _intrinsicsCy,
    double _intrinsicsS,
    double _clipNear, double _clipFar)
{
  return CameraSensorPrivate::BuildNDCMatrix(
           0, _imageWidth, 0, _imageHeight, _clipNear, _clipFar) *
           CameraSensorPrivate::BuildPerspectiveMatrix(
             _intrinsicsFx, _intrinsicsFy,
             _intrinsicsCx, _imageHeight - _intrinsicsCy,
             _intrinsicsS, _clipNear, _clipFar);
}

//////////////////////////////////////////////////
math::Matrix4d CameraSensorPrivate::BuildNDCMatrix(
    double _left, double _right,
    double _bottom, double _top,
    double _near, double _far)
{
  double inverseWidth = 1.0 / (_right - _left);
  double inverseHeight = 1.0 / (_top - _bottom);
  double inverseDistance = 1.0 / (_far - _near);

  return math::Matrix4d(
           2.0 * inverseWidth,
           0.0,
           0.0,
           -(_right + _left) * inverseWidth,
           0.0,
           2.0 * inverseHeight,
           0.0,
           -(_top + _bottom) * inverseHeight,
           0.0,
           0.0,
           -2.0 * inverseDistance,
           -(_far + _near) * inverseDistance,
           0.0,
           0.0,
           0.0,
           1.0);
}

//////////////////////////////////////////////////
math::Matrix4d CameraSensorPrivate::BuildPerspectiveMatrix(
    double _intrinsicsFx, double _intrinsicsFy,
    double _intrinsicsCx, double _intrinsicsCy,
    double _intrinsicsS,
    double _clipNear, double _clipFar)
{
  return math::Matrix4d(
           _intrinsicsFx,
           _intrinsicsS,
           -_intrinsicsCx,
           0.0,
           0.0,
           _intrinsicsFy,
           -_intrinsicsCy,
           0.0,
           0.0,
           0.0,
           _clipNear + _clipFar,
           _clipNear * _clipFar,
           0.0,
           0.0,
           -1.0,
           0.0);
}
