#pragma once
#include <rclcpp/rclcpp.hpp>

namespace pl_utils
{

template<typename ServiceT>
class SimpleServiceClient
{
public:
  SimpleServiceClient(rclcpp::Node* node, const std::string& name,
                      std::chrono::seconds wait = std::chrono::seconds(5))
    : node_(node)
  {
    client_ = node_->create_client<ServiceT>(name);
    if (!client_->wait_for_service(wait))
    {
      RCLCPP_FATAL(node_->get_logger(),
                   "[ServiceClient] service '%s' not available", name.c_str());
      rclcpp::shutdown();
    }
  }

  template<typename RequestT>
  bool call_and_wait(const std::shared_ptr<RequestT>& req,
                     std::chrono::seconds timeout = std::chrono::seconds(10))
  {
    auto future = client_->async_send_request(req);
    auto rc = rclcpp::spin_until_future_complete(
        node_->get_node_base_interface(), future, timeout);
    if (rc != rclcpp::FutureReturnCode::SUCCESS)
      return false;
    last_response_ = future.get();
    return true;
  }

  auto last_response() const { return last_response_; }

private:
  rclcpp::Node* node_;
  typename rclcpp::Client<ServiceT>::SharedPtr client_;
  typename ServiceT::Response::SharedPtr       last_response_;
};

} // namespace pl_utils
