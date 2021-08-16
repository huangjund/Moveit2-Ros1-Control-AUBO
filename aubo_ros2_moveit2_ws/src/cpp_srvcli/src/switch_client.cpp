//
// Created by jd on 2021/8/15.
//


#include "cpp_srvcli/switch_client.h"



using namespace std::chrono_literals;

auboi5::switch_client::switch_client(): rclcpp::Node("switch_state_client"), client_(
        this->create_client<aubo_messages::srv::NotSwitch>("switch_state_feedback")){
}

void auboi5::switch_client::open_node() {
  while (!client_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }
}


void auboi5::switch_client::send_request() {
  auto request = std::make_shared<aubo_messages::srv::NotSwitch::Request>();
  request->request = true;

  result_ = client_->async_send_request(request);
}


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto client = std::make_shared<auboi5::switch_client>();
  client->open_node();
  client->send_request();

  // Wait for the result.
  if (rclcpp::spin_until_future_complete(client, client->result_) ==
      rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "left is open: %d ; right is open: %d",
                client->result_.get()->left_open, client->result_.get()->right_open);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service switch_state_feedback");
  }

  rclcpp::shutdown();
  return 0;
}