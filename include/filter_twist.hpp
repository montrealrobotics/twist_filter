// filter_twist.hpp
#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rcl_interfaces/msg/parameter_type.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "component_filters.hpp"
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"

class TwistFilter
{
public:
  TwistFilter(rclcpp::Node *node,
              std::shared_ptr<TwistFilterObjectBase> linear_filter,
              std::shared_ptr<TwistFilterObjectBase> angular_filter);

  ~TwistFilter() = default;

private:
  void timer_callback();
  void declare_parameters();
  rcl_interfaces::msg::SetParametersResult parameters_callback(const std::vector<rclcpp::Parameter> &params);
  void update_twist(const geometry_msgs::msg::Twist::SharedPtr data);
  void pub_cmd();
  geometry_msgs::msg::Twist filter_twist(const geometry_msgs::msg::Twist &data);
  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr cb_;

  geometry_msgs::msg::Twist saturate_vel(const geometry_msgs::msg::Twist &v, double l_max, double a_max);
  double get_magnitude(const geometry_msgs::msg::Vector3 &v_comp);
  geometry_msgs::msg::Twist saturate_acc(const geometry_msgs::msg::Twist &v, double l_max, double a_max, double time_delta);
  geometry_msgs::msg::Twist get_acc(const geometry_msgs::msg::Twist &v, double time_delta);
  double get_slope(double current, double prev, double step);

  rclcpp::Node *node_;
  std::shared_ptr<TwistFilterObjectBase> linear_filter_;
  std::shared_ptr<TwistFilterObjectBase> angular_filter_;

  double linear_vel_max_;
  double linear_acc_max_;
  double angular_vel_max_;
  double angular_acc_max_;
  double timeout_;
  double last_val_;
  std::mutex lock_;

  rclcpp::Time time_prev_;
  geometry_msgs::msg::Twist twist_prev_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_in_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_out_;

  bool stopped_;
  geometry_msgs::msg::Twist cmd_;
  rclcpp::Time prev_time_;

  rclcpp::TimerBase::SharedPtr timer_;
};
