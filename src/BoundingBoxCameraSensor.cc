/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include <gz/math/Quaternion.hh>
#include <mutex>
#include <ostream>
#include <string>

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/image.pb.h>
#include <gz/msgs/annotated_axis_aligned_2d_box.pb.h>
#include <gz/msgs/annotated_axis_aligned_2d_box_v.pb.h>
#include <gz/msgs/annotated_oriented_3d_box.pb.h>
#include <gz/msgs/annotated_oriented_3d_box_v.pb.h>

#include <gz/common/Console.hh>
#include <gz/common/Image.hh>
#include <gz/common/Profiler.hh>
#include <gz/common/Util.hh>
#include <gz/msgs/Utility.hh>
#include <gz/rendering/BoundingBoxCamera.hh>
#include <gz/transport/Node.hh>
#include <gz/transport/Publisher.hh>
#include <gz/transport/TopicUtils.hh>

#include "gz/sensors/BoundingBoxCameraSensor.hh"
#include "gz/sensors/RenderingEvents.hh"
#include "gz/sensors/SensorFactory.hh"

#include "gz/sensors/DirectRosNode.hh"
#include "std_msgs/msg/header.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"
#include "vision_msgs/msg/detection3_d_array.hpp"
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>

using namespace gz;
using namespace sensors;

class gz::sensors::BoundingBoxCameraSensorPrivate
{
  /// \brief Save an image of rgb camera
  public: void SaveImage();

  /// \brief Save the bounding boxes
  public: void SaveBoxes();

  /// \brief SDF Sensor DOM Object
  public: sdf::Sensor sdfSensor;

  /// \brief True if Load() has been called and was successful
  public: bool initialized{false};

  /// \brief Rendering BoundingBox Camera
  public: rendering::BoundingBoxCameraPtr boundingboxCamera{nullptr};

  /// \brief Rendering RGB Camera to draw boxes on it and publish
  /// its image (just for visualization)
  public: rendering::CameraPtr rgbCamera{nullptr};

  /// \brief Node to create publisher
  public: transport::Node node;

  /// \brief Publisher to publish Image msg with drawn boxes
  // public: transport::Node::Publisher imagePublisher;

  /// \brief Publisher to publish BoundingBoxes msg
  // public: transport::Node::Publisher boxesPublisher;

  /// \brief Vector to receive boxes from the rendering camera
  public: std::vector<rendering::BoundingBox> boundingBoxes2d;
  public: std::vector<rendering::BoundingBox> boundingBoxes3d;

  /// \brief RGB Image to draw boxes on it
  public: rendering::Image image;

  /// \brief Buffer contains the image data to be saved
  public: unsigned char *saveImageBuffer{nullptr};

  /// \brief Connection to the new BoundingBox frames data
  public: common::ConnectionPtr newBoundingBoxConnection2d;
  public: common::ConnectionPtr newBoundingBoxConnection3d;

  /// \brief Connection to the Manager's scene change event.
  public: common::ConnectionPtr sceneChangeConnection;

  /// \brief Just a mutex for thread safety
  public: std::mutex mutex;

  /// \brief BoundingBoxes type
  public: rendering::BoundingBoxType type2d
    {rendering::BoundingBoxType::BBT_VISIBLEBOX2D};

  public: rendering::BoundingBoxType type3d
    {rendering::BoundingBoxType::BBT_BOX3D};

  /// \brief True to save images & boxes
  public: bool saveSample{false};

  /// \brief path directory to where images & boxes are saved
  public: std::string savePath{"./"};

  /// \brief Folder to save the image
  public: std::string saveImageFolder{"/images"};

  /// \brief Folder to save the bounding boxes
  public: std::string saveBoxesFolder{"/boxes"};

  /// \brief counter used to set the sample filename
  public: std::uint64_t saveCounter{0};

  public:
    std::shared_ptr<rclcpp::Node> directRosNode;
    std::string directRosNodeName = "gz_cameras_direct";
    std::shared_ptr<rclcpp::Publisher<vision_msgs::msg::Detection2DArray>> boxes2dPub;
    std::shared_ptr<rclcpp::Publisher<vision_msgs::msg::Detection3DArray>> boxes3dPub;
    std::chrono::steady_clock::duration now;
    std::string frameId;
};

//////////////////////////////////////////////////
BoundingBoxCameraSensor::BoundingBoxCameraSensor()
  : CameraSensor(), dataPtr(std::make_unique<BoundingBoxCameraSensorPrivate>())
{
}

/////////////////////////////////////////////////
BoundingBoxCameraSensor::~BoundingBoxCameraSensor()
{
  if (this->dataPtr->directRosNode != nullptr) {
    this->dataPtr->boxes2dPub.reset();
    this->dataPtr->boxes3dPub.reset();
    DirectRosNode::ReleaseDirectROSNode(this->dataPtr->directRosNodeName, this->dataPtr.get());
    this->dataPtr->directRosNode.reset();
  }
  //this->dataPtr->image_buffers.clear();
}

/////////////////////////////////////////////////
bool BoundingBoxCameraSensor::Init()
{
  return CameraSensor::Init();
}

/////////////////////////////////////////////////
bool BoundingBoxCameraSensor::Load(sdf::ElementPtr _sdf)
{
  sdf::Sensor sdfSensor;
  sdfSensor.Load(_sdf);
  return this->Load(sdfSensor);
}

/////////////////////////////////////////////////
bool BoundingBoxCameraSensor::Load(const sdf::Sensor &_sdf)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  auto sdfCamera = _sdf.CameraSensor();
  if (!sdfCamera)
    return false;

  // BoundingBox Type
  if (sdfCamera->HasBoundingBoxType())
  {
    std::string type = sdfCamera->BoundingBoxType();

    if (type.find("full_2d") != std::string::npos || type.find("full_box_2d") != std::string::npos)
      this->dataPtr->type2d = rendering::BoundingBoxType::BBT_FULLBOX2D;
    else if (type.find("2d") != std::string::npos || type.find("visible_2d") != std::string::npos || type.find("visible_box_2d") != std::string::npos)
      this->dataPtr->type2d = rendering::BoundingBoxType::BBT_VISIBLEBOX2D;
    else
      this->dataPtr->type2d = rendering::BoundingBoxType::BBT_NONE;

    if (type.find("3d") != std::string::npos)
      this->dataPtr->type3d = rendering::BoundingBoxType::BBT_BOX3D;
    else
      this->dataPtr->type3d = rendering::BoundingBoxType::BBT_NONE;
  }

  if (!Sensor::Load(_sdf))
  {
    return false;
  }

  this->dataPtr->frameId = this->FrameId();

  // Check if this is the right type
  if (_sdf.Type() != sdf::SensorType::BOUNDINGBOX_CAMERA)
  {
    gzerr << "Attempting to a load a BoundingBox Camera sensor, but received "
      << "a " << _sdf.TypeStr() << std::endl;
    return false;
  }

  if (_sdf.CameraSensor() == nullptr)
  {
    gzerr << "Attempting to a load a BoundingBox Camera sensor, but received "
      << "a null sensor." << std::endl;
    return false;
  }

  this->dataPtr->sdfSensor = _sdf;

  auto sdf_camera = _sdf.Element()->GetElement("camera");

  auto topicBoundingBoxes2d = sdf_camera->HasElement("topic2d") ? sdf_camera->GetElement("topic2d")->GetValue()->GetAsString() : "";
  auto topicBoundingBoxes3d = sdf_camera->HasElement("topic3d") ? sdf_camera->GetElement("topic3d")->GetValue()->GetAsString() : "";
  // auto topicImage = this->Topic() + "_image";

  // this->dataPtr->imagePublisher =
  //   this->dataPtr->node.Advertise<msgs::Image>(topicImage);

  // if (!this->dataPtr->imagePublisher)
  // {
  //   gzerr << "Unable to create publisher on topic ["
  //     << topicImage << "].\n";
  //   return false;
  // }

  // gzdbg << "Camera images for [" << this->Name() << "] advertised on ["
  //   << topicImage << "]" << std::endl;

  std::cout << "BB Camera [" << this->Name() << "] getting direct ROS node" << std::endl;
  this->dataPtr->directRosNode = DirectRosNode::GetDirectROSNode(this->dataPtr->directRosNodeName, this->dataPtr.get());
  if (this->dataPtr->directRosNode == nullptr) {
     gzerr << "Failed creating direct ROS node for BB Camera sensor [" << this->Name() << "]" << std::endl;
    return false;
  }

  if (this->dataPtr->type2d != rendering::BoundingBoxType::BBT_NONE && !topicBoundingBoxes2d.empty())
  {
    rclcpp::QoS qos(10);
    this->dataPtr->boxes2dPub = this->dataPtr->directRosNode->create_publisher<vision_msgs::msg::Detection2DArray>(topicBoundingBoxes2d, qos);
    gzdbg << "Bounding boxes 2d for [" << this->Name() << "] advertised on ["  << topicBoundingBoxes2d << std::endl;
  }

  if (this->dataPtr->type3d != rendering::BoundingBoxType::BBT_NONE && !topicBoundingBoxes3d.empty())
  {
    rclcpp::QoS qos(10);
    this->dataPtr->boxes3dPub = this->dataPtr->directRosNode->create_publisher<vision_msgs::msg::Detection3DArray>(topicBoundingBoxes3d, qos);
    gzdbg << "Bounding boxes 3d for [" << this->Name() << "] advertised on ["  << topicBoundingBoxes3d << std::endl;
  }

  // if (!this->dataPtr->boxesPublisher)
  // {
  //   gzerr << "Unable to create publisher on topic ["
  //     << topicBoundingBoxes << "].\n";
  //   return false;
  // }


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
  {
    this->CreateCamera();
  }

  this->dataPtr->sceneChangeConnection =
    RenderingEvents::ConnectSceneChangeCallback(
    std::bind(&BoundingBoxCameraSensor::SetScene, this,
    std::placeholders::_1));

  this->dataPtr->initialized = true;

  gzdbg << "Bounding boxes camera for [" << this->Name() << "] initialized" << std::endl;

  return true;
}

/////////////////////////////////////////////////
void BoundingBoxCameraSensor::SetScene(
  rendering::ScenePtr _scene)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  // APIs make it possible for the scene pointer to change
  if (this->Scene() != _scene)
  {
    this->dataPtr->boundingboxCamera = nullptr;
    this->dataPtr->rgbCamera = nullptr;
    RenderingSensor::SetScene(_scene);

    if (this->dataPtr->initialized)
      this->CreateCamera();
  }
}

/////////////////////////////////////////////////
bool BoundingBoxCameraSensor::CreateCamera()
{
  auto sdfCamera = this->dataPtr->sdfSensor.CameraSensor();
  if (!sdfCamera)
  {
    gzerr << "Unable to access camera SDF element\n";
    return false;
  }

  std::cout << "BB Cam [" << this->Name() << "] creating camera" << std::endl;

  if (!this->dataPtr->boundingboxCamera)
  {
    // Create rendering camera
    this->dataPtr->boundingboxCamera =
      this->Scene()->CreateBoundingBoxCamera(this->Name());

    // this->dataPtr->rgbCamera = this->Scene()->CreateCamera(
    //   this->Name() + "_rgbCamera");
  }

  auto width = sdfCamera->ImageWidth();
  auto height = sdfCamera->ImageHeight();

  std::cout << "BB Cam [" << this->Name() << "] creating camera w " << width << "x" << height << std::endl;

  if (width == 0u || height == 0u)
  {
    gzerr << "Unable to create a bounding box camera sensor with 0 width or height. " << std::endl;
    return false;
  }

  // Set Camera Properties
  // this->dataPtr->rgbCamera->SetImageFormat(rendering::PF_R8G8B8);
  // this->dataPtr->rgbCamera->SetImageWidth(width);
  // this->dataPtr->rgbCamera->SetImageHeight(height);
  // this->dataPtr->rgbCamera->SetVisibilityMask(sdfCamera->VisibilityMask());
  // this->dataPtr->rgbCamera->SetNearClipPlane(sdfCamera->NearClip());
  // this->dataPtr->rgbCamera->SetFarClipPlane(sdfCamera->FarClip());
  math::Angle angle = sdfCamera->HorizontalFov();
  if (angle < 0.01 || angle > GZ_PI*2)
  {
    gzerr << "Invalid horizontal field of view [" << angle << "]\n";
    return false;
  }
  double aspectRatio = static_cast<double>(width)/height;
  // this->dataPtr->rgbCamera->SetAspectRatio(aspectRatio);
  // this->dataPtr->rgbCamera->SetHFOV(angle);

  this->dataPtr->boundingboxCamera->SetImageWidth(width);
  this->dataPtr->boundingboxCamera->SetImageHeight(height);
  this->dataPtr->boundingboxCamera->SetNearClipPlane(sdfCamera->NearClip());
  this->dataPtr->boundingboxCamera->SetFarClipPlane(sdfCamera->FarClip());
  this->dataPtr->boundingboxCamera->SetImageFormat(rendering::PixelFormat::PF_R8G8B8);
  this->dataPtr->boundingboxCamera->SetAspectRatio(aspectRatio);
  this->dataPtr->boundingboxCamera->SetHFOV(angle);
  this->dataPtr->boundingboxCamera->SetVisibilityMask(sdfCamera->VisibilityMask());
  this->dataPtr->boundingboxCamera->SetBoundingBoxType(this->dataPtr->type2d, this->dataPtr->type3d);
  this->dataPtr->boundingboxCamera->SetLocalPose(this->Pose());

  // Add the camera to the scene
  //this->Scene()->RootVisual()->AddChild(this->dataPtr->rgbCamera);
  this->Scene()->RootVisual()->AddChild(this->dataPtr->boundingboxCamera);

  // Add the rendering sensors to handle its render
  this->AddSensor(this->dataPtr->boundingboxCamera);
  //this->AddSensor(this->dataPtr->rgbCamera);

  // use a copy so we do not modify the original sdfCamera
  // when updating bounding box camera
  auto sdfCameraCopy = *sdfCamera;
  // this->UpdateLensIntrinsicsAndProjection(this->dataPtr->rgbCamera,
  //     sdfCameraCopy);
  this->UpdateLensIntrinsicsAndProjection(this->dataPtr->boundingboxCamera,
      *sdfCamera);
  // Camera Info Msg
  this->PopulateInfo(sdfCamera);

  // Create the directory to store frames
  if (sdfCamera->SaveFrames())
  {
    this->dataPtr->savePath = sdfCamera->SaveFramesPath();
    this->dataPtr->saveImageFolder =
      this->dataPtr->savePath + this->dataPtr->saveImageFolder;
    this->dataPtr->saveBoxesFolder =
      this->dataPtr->savePath + this->dataPtr->saveBoxesFolder;
    this->dataPtr->saveSample = true;

    // Set the save counter to be equal number of images in the folder + 1
    // to continue adding to the images in the folder (multi scene datasets)
    if (common::isDirectory(this->dataPtr->saveImageFolder))
    {
      common::DirIter endIter;
      for (common::DirIter dirIter(this->dataPtr->saveImageFolder);
        dirIter != endIter; ++dirIter)
      {
        this->dataPtr->saveCounter++;
      }
    }
  }

  std::cout << "BB Cam [" << this->Name() << "] setting callbacks" << std::endl;

  // Connection to receive the BoundingBox buffer
  this->dataPtr->newBoundingBoxConnection2d =
    this->dataPtr->boundingboxCamera->ConnectNewBoundingBoxes2D(
      std::bind(&BoundingBoxCameraSensor::OnNewBoundingBoxes2D, this,
        std::placeholders::_1));
  
  this->dataPtr->newBoundingBoxConnection3d =
    this->dataPtr->boundingboxCamera->ConnectNewBoundingBoxes3D(
      std::bind(&BoundingBoxCameraSensor::OnNewBoundingBoxes3D, this,
        std::placeholders::_1));

  //this->dataPtr->image = this->dataPtr->rgbCamera->CreateImage();

  std::cout << "BB Cam [" << this->Name() << "] created ok" << std::endl;

  return true;
}

/////////////////////////////////////////////////
rendering::BoundingBoxCameraPtr
  BoundingBoxCameraSensor::BoundingBoxCamera() const
{
  return this->dataPtr->boundingboxCamera;
}

/////////////////////////////////////////////////
void BoundingBoxCameraSensor::OnNewBoundingBoxes2D(
  const std::vector<rendering::BoundingBox> &_boxes)
{
  GZ_PROFILE("BoundingBoxCameraSensor::OnNewBoundingBoxes");
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->dataPtr->boundingBoxes2d = _boxes;
  // std::cout << "[BB " << this->Name() << "] 2D: " << _boxes.size() << std::endl;

  if (!this->Has2DConnections())
    return;

  vision_msgs::msg::Detection2DArray msg;
  msg.header = std_msgs::msg::Header();
  msg.header.frame_id = this->dataPtr->frameId;
  DirectRosNode::SetCurrentStamp(&msg.header.stamp, this->dataPtr->now);

  for (const auto &box : _boxes)
  {
    vision_msgs::msg::Detection2D det;

    det.bbox.center.position.x = box.Center().X();
    det.bbox.center.position.y = box.Center().Y();
    det.bbox.size_x = box.Size().X();
    det.bbox.size_y = box.Size().Y();

    vision_msgs::msg::ObjectHypothesisWithPose res;
    res.hypothesis.class_id = std::to_string(box.Label());
    res.hypothesis.score = 1.0;
    det.results.push_back(res);

    msg.detections.push_back(det);
  }

  this->dataPtr->boxes2dPub->publish(msg);
}

/////////////////////////////////////////////////
void BoundingBoxCameraSensor::OnNewBoundingBoxes3D(
  const std::vector<rendering::BoundingBox> &_boxes)
{
  GZ_PROFILE("BoundingBoxCameraSensor::OnNewBoundingBoxes");
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->dataPtr->boundingBoxes3d = _boxes;
  // std::cout << "[BB " << this->Name() << "] 3D: " << _boxes.size() << std::endl;

  if (!this->Has3DConnections())
    return;

  vision_msgs::msg::Detection3DArray msg;
  msg.header = std_msgs::msg::Header();
  msg.header.frame_id = this->dataPtr->frameId;
  DirectRosNode::SetCurrentStamp(&msg.header.stamp, this->dataPtr->now);

  for (const auto &box : _boxes)
  {
    vision_msgs::msg::Detection3D det;

    det.bbox.size.x = box.Size().X();
    det.bbox.size.y = box.Size().Y();
    det.bbox.size.z = box.Size().Z();

    vision_msgs::msg::ObjectHypothesisWithPose res;
    res.hypothesis.class_id = std::to_string(box.Label());
    res.hypothesis.score = 1.0;
    res.pose.pose.position.x = box.Center().X();
    res.pose.pose.position.y = box.Center().Y();
    res.pose.pose.position.z = box.Center().Z();
    
    res.pose.pose.orientation.x = box.Orientation().X();
    res.pose.pose.orientation.y = box.Orientation().Y();
    res.pose.pose.orientation.z = box.Orientation().Z();
    res.pose.pose.orientation.w = box.Orientation().W();
    det.results.push_back(res);

    msg.detections.push_back(det);
  }

  this->dataPtr->boxes3dPub->publish(msg);
}

//////////////////////////////////////////////////
bool BoundingBoxCameraSensor::Update(const std::chrono::steady_clock::duration &_now)
{
  GZ_PROFILE("BoundingBoxCameraSensor::Update");
  if (!this->dataPtr->initialized)
  {
    gzerr << "Not initialized, update ignored.\n";
    return false;
  }

  if (!this->dataPtr->boundingboxCamera/*|| !this->dataPtr->rgbCamera*/)
  {
    gzerr << "Camera doesn't exist.\n";
    return false;
  }

  if (this->HasInfoConnections())
  {
    // publish the camera info message
    this->PublishInfo(_now);
  }

  auto has2DConnections = this->Has2DConnections();
  auto has3DConnections = this->Has3DConnections();

  // don't render if there are no subscribers nor saving
  if (!has2DConnections && !has3DConnections)
  {
    return false;
  }

  this->dataPtr->now = _now;

  // The sensor updates only the bounding box camera with its pose
  // as it has the same name, so make rgb camera with the same pose
  // this->dataPtr->rgbCamera->SetWorldPose(
  //   this->dataPtr->boundingboxCamera->WorldPose());

  // Render the bounding box camera
  this->Render();

 // std::cout << "Detecting " << this->dataPtr->boundingBoxes.size() << std::endl;


  // Render the rgb camera
  // this->dataPtr->rgbCamera->Copy(this->dataPtr->image);

  // auto imageBuffer = this->dataPtr->image.Data<unsigned char>();

  // if (this->dataPtr->saveSample)
  // {
  //   auto bufferSize = this->dataPtr->image.MemorySize();
  //   if (!this->dataPtr->saveImageBuffer)
  //     this->dataPtr->saveImageBuffer = new uint8_t[bufferSize];

  //   memcpy(this->dataPtr->saveImageBuffer, imageBuffer,
  //     bufferSize);
  // }

  // auto width = this->dataPtr->rgbCamera->ImageWidth();
  // auto height = this->dataPtr->rgbCamera->ImageHeight();

  // Create Image message
//  if (false) {

//     // Draw bounding boxes
//     for (const auto &box : this->dataPtr->boundingBoxes)
//     {
//       this->dataPtr->boundingboxCamera->DrawBoundingBox(
//         imageBuffer, math::Color::Green, box);
//     }

//     msgs::Image imageMsg;
//     imageMsg.set_width(width);
//     imageMsg.set_height(height);
//     // Format
//     imageMsg.set_step(
//       width * rendering::PixelUtil::BytesPerPixel(rendering::PF_R8G8B8));
//     imageMsg.set_pixel_format_type(
//       msgs::PixelFormatType::RGB_INT8);
//     // Time stamp
//     auto stamp = imageMsg.mutable_header()->mutable_stamp();
//     *stamp = msgs::Convert(_now);
//     auto frame = imageMsg.mutable_header()->add_data();
//     frame->set_key("frame_id");
//     frame->add_value(this->Name());
//     // Image data
//     imageMsg.set_data(imageBuffer,
//         rendering::PixelUtil::MemorySize(rendering::PF_R8G8B8,
//         width, height));

//     // Publish
//     this->AddSequence(imageMsg.mutable_header(), "rgbImage");
//     this->dataPtr->imagePublisher.Publish(imageMsg);
//   }

  // msgs::AnnotatedAxisAligned2DBox_V boxes2DMsg;
  // msgs::AnnotatedOriented3DBox_V boxes3DMsg;

  // if (this->dataPtr->type3d != rendering::BoundingBoxType::BBT_NONE)
  // {
  //   // Create 3D boxes message
  //   for (const auto &box : this->dataPtr->boundingBoxes3d)
  //   {
  //     // box data
  //     auto annotatedBox = boxes3DMsg.add_annotated_box();
  //     annotatedBox->set_label(box.Label());

  //     auto oriented3DBox = annotatedBox->mutable_box();
  //     msgs::Set(oriented3DBox->mutable_center(), box.Center());
  //     msgs::Set(oriented3DBox->mutable_boxsize(), box.Size());
  //     msgs::Set(oriented3DBox->mutable_orientation(), box.Orientation());
  //   }
  //   // time stamp
  //   auto stampBoxes =
  //     boxes3DMsg.mutable_header()->mutable_stamp();
  //   *stampBoxes = msgs::Convert(_now);
  //   auto frameBoxes = boxes3DMsg.mutable_header()->add_data();
  //   frameBoxes->set_key("frame_id");
  //   frameBoxes->add_value(this->Name());
  // }
  
  // if (this->dataPtr->type2d != rendering::BoundingBoxType::BBT_NONE) {
  //   // Create 2D boxes message
  //   for (const auto &box : this->dataPtr->boundingBoxes2d)
  //   {
  //     // box data
  //     auto annotatedBox = boxes2DMsg.add_annotated_box();
  //     annotatedBox->set_label(box.Label());

  //     auto minCorner = box.Center() - box.Size() * 0.5;
  //     auto maxCorner = box.Center() + box.Size() * 0.5;

  //     auto axisAlignedBox = annotatedBox->mutable_box();
  //     msgs::Set(axisAlignedBox->mutable_min_corner(),
  //         {minCorner.X(), minCorner.Y()});
  //     msgs::Set(axisAlignedBox->mutable_max_corner(),
  //         {maxCorner.X(), maxCorner.Y()});
  //   }
  //   // time stamp
  //   auto stampBoxes = boxes2DMsg.mutable_header()->mutable_stamp();
  //   *stampBoxes = msgs::Convert(_now);
  //   auto frameBoxes = boxes2DMsg.mutable_header()->add_data();
  //   frameBoxes->set_key("frame_id");
  //   frameBoxes->add_value(this->Name());
  // }

  //std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  // Publish
  // if (this->dataPtr->type == rendering::BoundingBoxType::BBT_BOX3D)
  // {
  //   this->AddSequence(boxes3DMsg.mutable_header(), "boundingboxes");
  //   this->dataPtr->boxesPublisher.Publish(boxes3DMsg);
  // }
  // else
  // {
  //   this->AddSequence(boxes2DMsg.mutable_header(), "boundingboxes");
  //   this->dataPtr->boxesPublisher.Publish(boxes2DMsg);
  // }

  // Save a sample (image & its bounding boxes)
  // if (this->dataPtr->saveSample)
  // {
  //   this->dataPtr->SaveImage();
  //   this->dataPtr->SaveBoxes();
  //   ++this->dataPtr->saveCounter;
  // }

  return true;
}

/////////////////////////////////////////////////
unsigned int BoundingBoxCameraSensor::ImageHeight() const
{
  return this->dataPtr->boundingboxCamera->ImageHeight();
}

/////////////////////////////////////////////////
unsigned int BoundingBoxCameraSensor::ImageWidth() const
{
  return this->dataPtr->boundingboxCamera->ImageWidth();
}

//////////////////////////////////////////////////
void BoundingBoxCameraSensorPrivate::SaveImage()
{
  // Attempt to create the save directory if it doesn't exist
  if (!common::isDirectory(this->savePath))
  {
    if (!common::createDirectories(this->savePath))
    {
      gzerr << "Failed to create directory [" << this->savePath << "]"
             << std::endl;
      return;
    }
  }
  // Attempt to create the image directory if it doesn't exist
  if (!common::isDirectory(this->saveImageFolder))
  {
    if (!common::createDirectories(this->saveImageFolder))
    {
      gzerr << "Failed to create directory [" << this->saveImageFolder << "]"
             << std::endl;
      return;
    }
  }

  auto width = this->rgbCamera->ImageWidth();
  auto height = this->rgbCamera->ImageHeight();
  if (width == 0 || height == 0)
    return;

  common::Image localImage;

  // Save the images in format of 0000001, 0000002 .. etc
  // Useful in sorting them in python
  std::stringstream ss;
  ss << std::setw(7) << std::setfill('0') << this->saveCounter;
  std::string saveCounterString = ss.str();

  std::string filename = "image_" + saveCounterString + ".png";

  localImage.SetFromData(this->saveImageBuffer, width, height,
      common::Image::RGB_INT8);
  localImage.SavePNG(
      common::joinPaths(this->saveImageFolder, filename));
}

//////////////////////////////////////////////////
void BoundingBoxCameraSensorPrivate::SaveBoxes()
{
  // Attempt to create the save directory if it doesn't exist
  if (!common::isDirectory(this->savePath))
  {
    if (!common::createDirectories(this->savePath))
    {
      gzerr << "Failed to create directory [" << this->savePath << "]"
             << std::endl;
      return;
    }
  }
  // Attempt to create the boxes directory if it doesn't exist
  if (!common::isDirectory(this->saveBoxesFolder))
  {
    if (!common::createDirectories(this->saveBoxesFolder))
    {
      gzerr << "Failed to create directory [" << this->saveBoxesFolder << "]"
             << std::endl;
      return;
    }
  }

  // Save the images in format of 0000001, 0000002 .. etc
  // Useful in sorting them in python
  std::stringstream ss;
  ss << std::setw(7) << std::setfill('0') << this->saveCounter;
  std::string saveCounterString = ss.str();

  std::string filename = this->saveBoxesFolder + "/boxes_" +
    saveCounterString + ".csv";
  std::ofstream file(filename);

  // if (this->type == rendering::BoundingBoxType::BBT_BOX3D)
  // {
  //   file << "label,x,y,z,w,h,l,roll,pitch,yaw\n";
  //   for (const auto &box : this->boundingBoxes)
  //   {
  //     auto label = std::to_string(box.Label());

  //     auto x = std::to_string(box.Center().X());
  //     auto y = std::to_string(box.Center().Y());
  //     auto z = std::to_string(box.Center().Z());

  //     auto w = std::to_string(box.Size().X());
  //     auto h = std::to_string(box.Size().Y());
  //     auto l = std::to_string(box.Size().Z());

  //     auto roll = std::to_string(box.Orientation().Roll());
  //     auto pitch = std::to_string(box.Orientation().Pitch());
  //     auto yaw = std::to_string(box.Orientation().Yaw());

  //     // label x y z w h l roll pitch yaw
  //     std::string sep = ",";
  //     std::string boxString = label + sep + x + sep + y + sep + z + sep +
  //       w + sep + h + sep + l + sep + roll + sep + pitch + sep + yaw;

  //     file << boxString + '\n';
  //   }
  // }
  // else
  // {
  //   file << "label,x_center,y_center,width,height\n";
  //   for (const auto &box : this->boundingBoxes)
  //   {
  //     auto label = std::to_string(box.Label());

  //     auto x = std::to_string(box.Center().X());
  //     auto y = std::to_string(box.Center().Y());
  //     auto width = std::to_string(box.Size().X());
  //     auto height = std::to_string(box.Size().Y());

  //     // label x y width height
  //     std::string sep = ",";
  //     std::string boxString = label + sep +
  //       x + sep + y + sep + width + sep + height;

  //     file << boxString + '\n';
  //   }
  // }
  file.close();
}

//////////////////////////////////////////////////
bool BoundingBoxCameraSensor::HasConnections() const
{
  return this->Has2DConnections() || this->Has3DConnections();
}

//////////////////////////////////////////////////
bool BoundingBoxCameraSensor::Has2DConnections() const
{
  return (this->dataPtr->boxes2dPub && this->dataPtr->boxes2dPub->get_subscription_count() > 0);
}

//////////////////////////////////////////////////
bool BoundingBoxCameraSensor::Has3DConnections() const
{
  return (this->dataPtr->boxes3dPub && this->dataPtr->boxes3dPub->get_subscription_count() > 0);
}
