#include <controller/controller_cascading_pid.hpp>

#include <memory>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<roscopter::ControllerCascadingPID>();
  RCLCPP_INFO_ONCE(node->get_logger(), "Using default (cascading PID) controller");
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
