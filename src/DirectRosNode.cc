#include "gz/sensors/DirectRosNode.hh"
#include <chrono>
#include <rclcpp/executors.hpp>

using namespace gz;
using namespace sensors;

std::map<std::string, DirectRosNode::NodeRef> DirectRosNode::directROSNodes;
std::mutex DirectRosNode::mutex;
bool DirectRosNode::spinning = false;
std::thread DirectRosNode::spinner_thread;
// bool DirectRosNode::rclcpp_intiated = false;


std::shared_ptr<rclcpp::Node>  gz::sensors::DirectRosNode::GetDirectROSNode(std::string node_name, void* owner_ptr) {

  std::lock_guard<std::mutex> lock(DirectRosNode::mutex);

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

  std::cout << "Adding owner ref for " << node_name << std::endl;
  if (std::find(node_ref->owners.begin(), node_ref->owners.end(), owner_ptr) == node_ref->owners.end()) {
    node_ref->owners.push_back(owner_ptr);
  }

  if (!DirectRosNode::spinning) {
    DirectRosNode::spinning = true;
    DirectRosNode::spinner_thread = std::thread(&DirectRosNode::SpinNodes);
    DirectRosNode::spinner_thread.detach();
  }

  return node_ref->node;
}

void DirectRosNode::SpinNodes() {
    std::cout << "Spinning direct ROS Nodes every second... " << std::endl;
    while (DirectRosNode::spinning) {
      for (const auto& pair : DirectRosNode::directROSNodes) {
        //std::cout << "spinning " << pair.first << std::endl;
        rclcpp::spin_some(pair.second.node);
      }
      std::this_thread::sleep_for(std::chrono::seconds(1)); // spin all once a sex
    }
    std::cout << "Stopped spinning direct ROS nodes." << std::endl;
}


void gz::sensors::DirectRosNode::ReleaseDirectROSNode(std::string node_name, void* owner_ptr) {

  std::lock_guard<std::mutex> lock(DirectRosNode::mutex);

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

  if (DirectRosNode::directROSNodes.empty()) {
    DirectRosNode::spinning = false; // kill spinner
  }
  
}