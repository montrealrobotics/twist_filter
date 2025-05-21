// filter_twist.cpp (Refactored & Optimized)
#include "filter_twist.hpp"
#include <cmath>
#include <algorithm>

TwistFilter::TwistFilter(rclcpp::Node *node,
                         std::shared_ptr<TwistFilterObjectBase> linear_filter,
                         std::shared_ptr<TwistFilterObjectBase> angular_filter)
    : node_(node),
      linear_filter_(linear_filter),
      angular_filter_(angular_filter),
      linear_vel_max_(1.0),
      linear_acc_max_(1.0),
      angular_vel_max_(1.0),
      angular_acc_max_(1.0),
      timeout_(0.25),
      last_val_(0.0),
      stopped_(false)
{
    _declare_parameters();

    cb_ = node_->add_on_set_parameters_callback(
        std::bind(&TwistFilter::_parameters_callback, this, std::placeholders::_1));

    time_prev_ = node_->get_clock()->now();

    auto qos = rclcpp::QoS(1).reliable().durability_volatile();

    sub_cmd_in_ = node_->create_subscription<geometry_msgs::msg::Twist>(
        "filter_in",
        qos,
        std::bind(&TwistFilter::update_twist, this, std::placeholders::_1));

    pub_cmd_out_ = node_->create_publisher<geometry_msgs::msg::Twist>("filter_out", qos);

    prev_time_ = node_->get_clock()->now();

    timer_ = node_->create_wall_timer(
        std::chrono::duration<double>(0.1),
        std::bind(&TwistFilter::timer_callback, this));
}

rcl_interfaces::msg::SetParametersResult TwistFilter::_parameters_callback(
    const std::vector<rclcpp::Parameter> &params)
{
    for (const auto &param : params)
    {
        const std::string &name = param.get_name();
        if (name == "linear_vel_max") linear_vel_max_ = param.as_double();
        else if (name == "linear_acc_max") linear_acc_max_ = param.as_double();
        else if (name == "angular_vel_max") angular_vel_max_ = param.as_double();
        else if (name == "angular_acc_max") angular_acc_max_ = param.as_double();
        else if (name == "timeout") timeout_ = param.as_double();
    }

    bool success = linear_filter_->update_filters(params) || angular_filter_->update_filters(params);

    RCLCPP_INFO(node_->get_logger(),
                "Filter Reconfigure: Linear vel max: %f, Linear acc max: %f, Angular vel max: %f, Angular acc max: %f, Timeout: %f",
                linear_vel_max_, linear_acc_max_, angular_vel_max_, angular_acc_max_, timeout_);

    rcl_interfaces::msg::SetParametersResult result;
    result.successful = success;
    return result;
}

geometry_msgs::msg::Twist TwistFilter::filter_twist(const geometry_msgs::msg::Twist &data)
{
    geometry_msgs::msg::Twist cmd_out = data;
    int64_t time_filter = node_->get_clock()->now().nanoseconds();

    linear_filter_->filter_vector(data.linear, cmd_out.linear, time_filter);
    angular_filter_->filter_vector(data.angular, cmd_out.angular, time_filter);

    rclcpp::Time time_now = node_->get_clock()->now();
    double time_delta = (time_now - time_prev_).seconds();

    if (linear_vel_max_ > 0 || angular_vel_max_ > 0)
        cmd_out = _saturate_vel(cmd_out, linear_vel_max_, angular_vel_max_);

    if ((linear_acc_max_ > 0 || angular_acc_max_ > 0) && time_delta > 1e-5)
        cmd_out = _saturate_acc(cmd_out, linear_acc_max_, angular_acc_max_, time_delta);

    twist_prev_ = cmd_out;
    time_prev_ = time_now;

    return cmd_out;
}

void TwistFilter::timer_callback() { pub_cmd(); }

void TwistFilter::_declare_parameters()
{
    rcl_interfaces::msg::ParameterDescriptor d;
    d.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;

    auto declare = [&](const std::string &name, double def, double &store) {
        node_->declare_parameter(name, def, d);
        store = node_->get_parameter(name).as_double();
    };

    declare("linear_vel_max", 1.0, linear_vel_max_);
    declare("linear_acc_max", 1.0, linear_acc_max_);
    declare("angular_vel_max", 1.0, angular_vel_max_);
    declare("angular_acc_max", 1.0, angular_acc_max_);
    declare("timeout", 0.25, timeout_);
}

void TwistFilter::update_twist(const geometry_msgs::msg::Twist::SharedPtr data)
{
    std::lock_guard<std::mutex> lock(lock_);
    cmd_ = *data;
    prev_time_ = node_->get_clock()->now();
}

void TwistFilter::pub_cmd()
{
    std::lock_guard<std::mutex> lock(lock_);
    auto current_time = node_->get_clock()->now();
    double elapsed = (current_time - prev_time_).seconds();

    bool is_zero = _get_mag(cmd_.linear) == 0.0 && _get_mag(cmd_.angular) == 0.0;
    if (is_zero)
    {
        geometry_msgs::msg::Twist cmd = geometry_msgs::msg::Twist();
        cmd.linear.x = 0.0;
        cmd.linear.y = 0.0;
        cmd.linear.z = 0.0;
        cmd.angular.x = 0.0;
        cmd.angular.y = 0.0;
        cmd.angular.z = 0.0;
        pub_cmd_out_->publish(cmd);
        linear_filter_->reset_state();
        angular_filter_->reset_state();
        return;
    }
    geometry_msgs::msg::Twist cmd = (elapsed > timeout_) ? geometry_msgs::msg::Twist() : cmd_;
    cmd = filter_twist(cmd);

    if (!is_zero || !stopped_)
    {
        pub_cmd_out_->publish(cmd);
        stopped_ = is_zero;
    }
}

double TwistFilter::_get_mag(const geometry_msgs::msg::Vector3 &v)
{
    return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

geometry_msgs::msg::Twist TwistFilter::_saturate_vel(const geometry_msgs::msg::Twist &v, double l_max, double a_max)
{
    geometry_msgs::msg::Twist sat = v;
    double mag = _get_mag(sat.linear);
    if (mag > l_max)
    {
        double r = l_max / mag;
        sat.linear.x *= r; sat.linear.y *= r; sat.linear.z *= r;
    }
    mag = _get_mag(sat.angular);
    if (mag > a_max)
    {
        double r = a_max / mag;
        sat.angular.x *= r; sat.angular.y *= r; sat.angular.z *= r;
    }
    return sat;
}

geometry_msgs::msg::Twist TwistFilter::_saturate_acc(const geometry_msgs::msg::Twist &v, double l_max, double a_max, double dt)
{
    geometry_msgs::msg::Twist acc = _get_acc(v, dt);
    geometry_msgs::msg::Twist sat;

    double mag = _get_mag(acc.linear);
    if (mag > l_max)
    {
        double r = l_max / mag;
        acc.linear.x *= r; acc.linear.y *= r; acc.linear.z *= r;
    }
    mag = _get_mag(acc.angular);
    if (mag > a_max)
    {
        double r = a_max / mag;
        acc.angular.x *= r; acc.angular.y *= r; acc.angular.z *= r;
    }

    sat.linear.x = acc.linear.x * dt + twist_prev_.linear.x;
    sat.linear.y = acc.linear.y * dt + twist_prev_.linear.y;
    sat.linear.z = acc.linear.z * dt + twist_prev_.linear.z;
    sat.angular.x = acc.angular.x * dt + twist_prev_.angular.x;
    sat.angular.y = acc.angular.y * dt + twist_prev_.angular.y;
    sat.angular.z = acc.angular.z * dt + twist_prev_.angular.z;

    return sat;
}

geometry_msgs::msg::Twist TwistFilter::_get_acc(const geometry_msgs::msg::Twist &v, double dt)
{
    geometry_msgs::msg::Twist a;
    a.linear.x = _get_slope(v.linear.x, twist_prev_.linear.x, dt);
    a.linear.y = _get_slope(v.linear.y, twist_prev_.linear.y, dt);
    a.linear.z = _get_slope(v.linear.z, twist_prev_.linear.z, dt);
    a.angular.x = _get_slope(v.angular.x, twist_prev_.angular.x, dt);
    a.angular.y = _get_slope(v.angular.y, twist_prev_.angular.y, dt);
    a.angular.z = _get_slope(v.angular.z, twist_prev_.angular.z, dt);
    return a;
}

double TwistFilter::_get_slope(double current, double prev, double dt)
{
    return (current - prev) / dt;
}
