//
// Created by jd on 2021/8/15.
//

#include "rclcpp/rclcpp.hpp"
#include <aubo_messages/srv/not_switch.hpp>

#include <memory>

void state_callback(const std::shared_ptr<aubo_messages::srv::NotSwitch::Request> request,
                    std::shared_ptr<aubo_messages::srv::NotSwitch::Response>      response)
{
  response->left_open = true;
  response->right_open = true;

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"request: %d", request->request);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("switch_state_server");

  rclcpp::Service<aubo_messages::srv::NotSwitch>::SharedPtr service =
          node->create_service<aubo_messages::srv::NotSwitch>("switch_state_feedback", &state_callback);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to receive switch state requests");

  rclcpp::spin(node);
  rclcpp::shutdown();
}