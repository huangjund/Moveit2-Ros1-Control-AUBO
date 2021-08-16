//
// Created by miniload on 8/16/21.
//

#ifndef CPP_SRVCLI_INCLUDE_CPP_SRVCLI_SWITCH_CLIENT_H_
#define CPP_SRVCLI_INCLUDE_CPP_SRVCLI_SWITCH_CLIENT_H_

#include "rclcpp/rclcpp.hpp"
#include <aubo_messages/srv/not_switch.hpp>

#include <chrono>
#include <cstdlib>
#include <memory>

namespace auboi5{
class switch_client: public rclcpp::Node{
 public:
  switch_client();
  ~switch_client() override = default;

  // no copy, no assign, no move
  switch_client(const switch_client&) = delete;
  switch_client(const switch_client&&) = delete;
  switch_client& operator=(const switch_client&) = delete;

  void send_request();
  void open_node();

 private:
  rclcpp::Client<aubo_messages::srv::NotSwitch>::SharedPtr client_;
};
} // namespace auboi5

#endif //CPP_SRVCLI_INCLUDE_CPP_SRVCLI_SWITCH_CLIENT_H_
