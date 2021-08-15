//
// Created by jd on 2021/8/15.
//

#include "rclcpp/rclcpp.hpp"
#include <aubo_messages/srv/not_switch.hpp>

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("switch_state_client");
  rclcpp::Client<aubo_messages::srv::NotSwitch>::SharedPtr client =
          node->create_client<aubo_messages::srv::NotSwitch>("switch_state_feedback");

  auto request = std::make_shared<aubo_messages::srv::NotSwitch::Request>();
  request->request = true;

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto result = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
      rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "left is open: %d ; right is open: %d",
            result.get()->left_open, result.get()->right_open);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service switch_state_feedback");
  }

  rclcpp::shutdown();
  return 0;
}