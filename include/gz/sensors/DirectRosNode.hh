#ifndef GZ_DIRECT_ROS_NODE_HH_
#define GZ_DIRECT_ROS_NODE_HH_

#include <memory>
#include <string>
#include <rclcpp/node.hpp>
#include <vector>
#include <mutex>

namespace gz
{
  namespace sensors
  {

    // const int NS_TO_SEC = 1000000000;

    class DirectRosNode {

        public:
            static std::shared_ptr<rclcpp::Node> GetDirectROSNode(std::string node_name, void* owner_ptr);
            static void ReleaseDirectROSNode(std::string node_name, void* owner_ptr);
            static std::mutex mutex;
            static void SetCurrentStamp(builtin_interfaces::msg::Time *stamp,  std::chrono::steady_clock::duration timestamp);

        private:

            struct NodeRef {
                std::shared_ptr<rclcpp::Node> node;
                std::vector<void *> owners;
            };

            static std::map<std::string, NodeRef> directROSNodes;
            static bool spinning;
            static void SpinNodes();
            static std::thread spinner_thread;
    };

  }
}

#endif