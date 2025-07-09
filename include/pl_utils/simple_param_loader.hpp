#pragma once
#include <rclcpp/rclcpp.hpp>

namespace pl_utils
{

class SimpleParamLoader
{
public:
  explicit SimpleParamLoader(rclcpp::Node* node) : node_(node) {}

  // вернёт параметр либо аварийно завершит узел
  template<typename T>
  T load_required(const std::string& name)
  {
    // если параметр ещё не объявлен – объявим его с "пустым" дефолт-значением
    if (!node_->has_parameter(name))
      node_->declare_parameter<T>(name, T{});   // ← добавили default value

    T value;
    if (!node_->get_parameter(name, value))
    {
      RCLCPP_FATAL(node_->get_logger(),
                  "[ParamLoader] Required param '%s' not set!", name.c_str());
      rclcpp::shutdown();
    }
    return value;
  }
 
private:
  rclcpp::Node* node_;
};

} // namespace pl_utils
