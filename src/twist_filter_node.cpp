// combo_twist_filter_node.cpp
#include <memory>
#include <map>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "component_filters.hpp"
#include "filter_twist.hpp"

class ComboFilterNode : public rclcpp::Node
{
public:
  ComboFilterNode() : Node("twist_filter")
  {
    declare_parameter("filter_linear", "avg");
    declare_parameter("filter_angular", "lp");
    declare_parameter("active_filters.linear.x", true);
    declare_parameter("active_filters.linear.y", false);
    declare_parameter("active_filters.linear.z", false);
    declare_parameter("active_filters.angular.x", false);
    declare_parameter("active_filters.angular.y", false);
    declare_parameter("active_filters.angular.z", true);

    // LP filter parameters
    declare_parameter("filter_linear.tau", 0.2);
    declare_parameter("filter_linear.damping", 0.8);
    declare_parameter("filter_angular.tau", 0.2);
    declare_parameter("filter_angular.damping", 0.8);

    // FIR/IIR filter parameters
    declare_parameter("filter_linear.num_samples", 2);
    declare_parameter("filter_linear.weights", "[]");
    declare_parameter("filter_linear.num_out_samples", 2);
    declare_parameter("filter_linear.out_weights", "[]");
    declare_parameter("filter_angular.num_samples", 2);
    declare_parameter("filter_angular.weights", "[]");
    declare_parameter("filter_angular.num_out_samples", 2);
    declare_parameter("filter_angular.out_weights", "[]");

    // Create active filters configuration
    std::map<std::string, std::map<std::string, bool>> active_filters;
    active_filters["linear"]["x"] = get_parameter("active_filters.linear.x").as_bool();
    active_filters["linear"]["y"] = get_parameter("active_filters.linear.y").as_bool();
    active_filters["linear"]["z"] = get_parameter("active_filters.linear.z").as_bool();
    active_filters["angular"]["x"] = get_parameter("active_filters.angular.x").as_bool();
    active_filters["angular"]["y"] = get_parameter("active_filters.angular.y").as_bool();
    active_filters["angular"]["z"] = get_parameter("active_filters.angular.z").as_bool();

    // Get filter types
    std::string filter_linear = get_parameter("filter_linear").as_string();
    std::string filter_angular = get_parameter("filter_angular").as_string();

    // Create filter configurations for LP
    std::map<std::string, double> config_lp_linear;
    config_lp_linear["tau"] = get_parameter("filter_linear.tau").as_double();
    config_lp_linear["damping"] = get_parameter("filter_linear.damping").as_double();

    std::map<std::string, double> config_lp_angular;
    config_lp_angular["tau"] = get_parameter("filter_angular.tau").as_double();
    config_lp_angular["damping"] = get_parameter("filter_angular.damping").as_double();

    // Create filter configurations for FIR/IIR
    std::map<std::string, std::string> config_fir_iir_linear;
    config_fir_iir_linear["num_samples"] = std::to_string(get_parameter("filter_linear.num_samples").as_int());
    config_fir_iir_linear["weights"] = get_parameter("filter_linear.weights").as_string();
    config_fir_iir_linear["num_out_samples"] = std::to_string(get_parameter("filter_linear.num_out_samples").as_int());
    config_fir_iir_linear["out_weights"] = get_parameter("filter_linear.out_weights").as_string();

    std::map<std::string, std::string> config_fir_iir_angular;
    config_fir_iir_angular["num_samples"] = std::to_string(get_parameter("filter_angular.num_samples").as_int());
    config_fir_iir_angular["weights"] = get_parameter("filter_angular.weights").as_string();
    config_fir_iir_angular["num_out_samples"] = std::to_string(get_parameter("filter_angular.num_out_samples").as_int());
    config_fir_iir_angular["out_weights"] = get_parameter("filter_angular.out_weights").as_string();

    RCLCPP_INFO(get_logger(), "Active filters: linear.x=%s, linear.y=%s, linear.z=%s, angular.x=%s, angular.y=%s, angular.z=%s",
                active_filters["linear"]["x"] ? "true" : "false",
                active_filters["linear"]["y"] ? "true" : "false",
                active_filters["linear"]["z"] ? "true" : "false",
                active_filters["angular"]["x"] ? "true" : "false",
                active_filters["angular"]["y"] ? "true" : "false",
                active_filters["angular"]["z"] ? "true" : "false");

    std::shared_ptr<TwistFilterObjectBase> linear_filter;
    if (filter_linear == "lp")
    {
      linear_filter = std::static_pointer_cast<TwistFilterObjectBase>(
          std::make_shared<LPTwistFilterObject>(active_filters, config_lp_linear));
      RCLCPP_INFO(get_logger(), "Using LP filter for linear velocity");
    }
    else if (filter_linear == "fir" || filter_linear == "avg")
    {
      linear_filter = std::static_pointer_cast<TwistFilterObjectBase>(
          std::make_shared<FIRTwistFilterObject>(active_filters, config_fir_iir_linear));
      RCLCPP_INFO(get_logger(), "Using FIR/AVG filter for linear velocity");
    }
    else if (filter_linear == "iir")
    {
      linear_filter = std::static_pointer_cast<TwistFilterObjectBase>(
          std::make_shared<IIRTwistFilterObject>(active_filters, config_fir_iir_linear));
      RCLCPP_INFO(get_logger(), "Using IIR filter for linear velocity");
    }
    else
    {
      linear_filter = std::static_pointer_cast<TwistFilterObjectBase>(
          std::make_shared<FIRTwistFilterObject>(active_filters, config_fir_iir_linear));
      RCLCPP_WARN(get_logger(), "Unknown filter type: %s, using FIR filter for linear velocity", filter_linear.c_str());
    }

    std::shared_ptr<TwistFilterObjectBase> angular_filter;
    if (filter_angular == "lp")
    {
      angular_filter = std::static_pointer_cast<TwistFilterObjectBase>(
          std::make_shared<LPTwistFilterObject>(active_filters, config_lp_angular));
      RCLCPP_INFO(get_logger(), "Using LP filter for angular velocity");
    }
    else if (filter_angular == "fir" || filter_angular == "avg")
    {
      angular_filter = std::static_pointer_cast<TwistFilterObjectBase>(
          std::make_shared<FIRTwistFilterObject>(active_filters, config_fir_iir_angular));
      RCLCPP_INFO(get_logger(), "Using FIR/AVG filter for angular velocity");
    }
    else if (filter_angular == "iir")
    {
      angular_filter = std::static_pointer_cast<TwistFilterObjectBase>(
          std::make_shared<IIRTwistFilterObject>(active_filters, config_fir_iir_angular));
      RCLCPP_INFO(get_logger(), "Using IIR filter for angular velocity");
    }
    else
    {
      angular_filter = std::static_pointer_cast<TwistFilterObjectBase>(
          std::make_shared<LPTwistFilterObject>(active_filters, config_lp_angular));
      RCLCPP_WARN(get_logger(), "Unknown filter type: %s, using LP filter for angular velocity", filter_angular.c_str());
    }
    twist_filter_ = std::make_shared<TwistFilter>(this, linear_filter, angular_filter);
  }

private:
  std::shared_ptr<TwistFilter> twist_filter_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ComboFilterNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}