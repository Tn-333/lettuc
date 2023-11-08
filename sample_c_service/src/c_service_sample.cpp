#include "rclcpp/rclcpp.hpp"
#include "xarm_msgs/srv/plan_pose.hpp"
#include <xarm_msgs/srv/set_digital_io.hpp>


#include <chrono>
#include <memory>
#include <cstdio>

using namespace std::chrono_literals;



int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("c_service_sample");
  rclcpp::Client<xarm_msgs::srv::SetDigitalIO>::SharedPtr client = node->create_client<xarm_msgs::srv::SetDigitalIO>("/xarm/set_tgpio_digital");

  auto request = std::make_shared<xarm_msgs::srv::SetDigitalIO::Request>();
  request->ionum = 0;
  request->value = 1;

  while(!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto result = client->async_send_request(request);
  //wait for reault

  rclcpp::shutdown();
  return 0;
}
