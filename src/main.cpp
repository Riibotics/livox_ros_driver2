#include "livox_ros_driver2.cpp"

#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<livox_ros::DriverNode>(options);

  // Auto-transition: unconfigured → configured → active
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  CallbackReturn rc;

  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE, rc);
  if (rc != CallbackReturn::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "Failed to configure lifecycle node.");
    rclcpp::shutdown();
    return 1;
  }

  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE, rc);
  if (rc != CallbackReturn::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "Failed to activate lifecycle node.");
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
