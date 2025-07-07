#include "gz/sensors/DirectRosNode.hh"

using namespace gz;
using namespace sensors;

std::map<std::string, DirectRosNode::NodeRef> DirectRosNode::directROSNodes;
std::mutex DirectRosNode::mutex;
// bool DirectRosNode::rclcpp_intiated = false;


std::shared_ptr<rclcpp::Node>  gz::sensors::DirectRosNode::GetDirectROSNode(std::string node_name, void* owner_ptr) {

  std::lock_guard<std::mutex> lock(mutex);

  // if (!DirectRosNode::rclcpp_intiated) {
  //   DirectRosNode::rclcpp_intiated = true;
  //   rclcpp::init(0, nullptr);
  // }

  NodeRef *node_ref = nullptr;
  if (DirectRosNode::directROSNodes.find(node_name) != DirectRosNode::directROSNodes.end()) {
    std::cout << "Using direct ROS node " << node_name << std::endl;
    node_ref = &DirectRosNode::directROSNodes.at(node_name);
  } else {
    std::cout << "Making direct ROS node " << node_name << std::endl;
    NodeRef new_node_ref;
    new_node_ref.node = std::make_shared<rclcpp::Node>(node_name);
    DirectRosNode::directROSNodes.emplace(node_name, new_node_ref);
    node_ref = &DirectRosNode::directROSNodes.at(node_name);
  }

  if (std::find(node_ref->owners.begin(), node_ref->owners.end(), owner_ptr) == node_ref->owners.end()) {
    node_ref->owners.push_back(owner_ptr);
  }

  return node_ref->node;
}


void gz::sensors::DirectRosNode::ReleaseDirectROSNode(std::string node_name, void* owner_ptr) {

  std::lock_guard<std::mutex> lock(mutex);

  if (DirectRosNode::directROSNodes.find(node_name) == DirectRosNode::directROSNodes.end())
    return;

  auto node_ref = &DirectRosNode::directROSNodes.at(node_name);
  auto pos = std::find(node_ref->owners.begin(), node_ref->owners.end(), owner_ptr);
  if (pos != node_ref->owners.end()) {
    std::cout << "Releasing direct ROS node " << node_name << std::endl;
    node_ref->owners.erase(pos);
  }

  if (node_ref->owners.size() == 0) {
    std::cout << "Destroying direct ROS node " << node_name << std::endl;
    DirectRosNode::directROSNodes.erase(node_name);
  }

  // if (DirectRosNode::directROSNodes.empty()) {
  //   rclcpp::shutdown();
  //   DirectRosNode::rclcpp_intiated = false;
  // }
  
}