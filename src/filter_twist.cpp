// filter_twist.cpp
#include "filter_twist.hpp"
#include <cmath>

// filter_twist.cpp - constructor and filter_twist method
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

    node_->add_on_set_parameters_callback(
        std::bind(&TwistFilter::_parameters_callback, this, std::placeholders::_1));

    time_prev_ = node_->get_clock()->now();

    auto qos = rclcpp::QoS(1)
                   .reliability(rclcpp::ReliabilityPolicy::Reliable)
                   .durability(rclcpp::DurabilityPolicy::Volatile);

    sub_cmd_in_ = node_->create_subscription<geometry_msgs::msg::Twist>(
        "filter_in",
        1,
        std::bind(&TwistFilter::update_twist, this, std::placeholders::_1));

    pub_cmd_out_ = node_->create_publisher<geometry_msgs::msg::Twist>(
        "filter_out",
        qos);

    prev_time_ = node_->get_clock()->now();

    double timer_period = 0.1; // seconds
    timer_ = node_->create_wall_timer(
        std::chrono::duration<double>(timer_period),
        std::bind(&TwistFilter::timer_callback, this));
}

// Also update _parameters_callback to update both filters
rcl_interfaces::msg::SetParametersResult TwistFilter::_parameters_callback(
    const std::vector<rclcpp::Parameter> &params)
{
    for (const auto &param : params)
    {
        if (param.get_name() == "linear_vel_max")
        {
            linear_vel_max_ = param.as_double();
        }
        else if (param.get_name() == "linear_acc_max")
        {
            linear_acc_max_ = param.as_double();
        }
        else if (param.get_name() == "angular_vel_max")
        {
            angular_vel_max_ = param.as_double();
        }
        else if (param.get_name() == "angular_acc_max")
        {
            angular_acc_max_ = param.as_double();
        }
        else if (param.get_name() == "timeout")
        {
            timeout_ = param.as_double();
        }
    }

    // Update both filters
    bool linear_result = linear_filter_->update_filters(params);
    bool angular_result = angular_filter_->update_filters(params);

    RCLCPP_INFO(node_->get_logger(),
                "Filter Reconfigure: Linear vel max: %f, Linear acc max: %f, Angular vel max: %f, Angular acc max: %f, Timeout: %f",
                linear_vel_max_, linear_acc_max_, angular_vel_max_, angular_acc_max_, timeout_);

    rcl_interfaces::msg::SetParametersResult set_result;
    set_result.successful = linear_result && angular_result;
    return set_result;
}

// Update filter_twist to use both filters
geometry_msgs::msg::Twist TwistFilter::filter_twist(const geometry_msgs::msg::Twist &data)
{
    geometry_msgs::msg::Twist cmd_out = data;
    int64_t time_filter = node_->get_clock()->now().nanoseconds();

    // Process linear components with linear filter
    auto lp_linear = dynamic_cast<LPTwistFilterObject *>(linear_filter_.get());
    auto fir_linear = dynamic_cast<FIRTwistFilterObject *>(linear_filter_.get());
    auto iir_linear = dynamic_cast<IIRTwistFilterObject *>(linear_filter_.get());

    // Process angular components with angular filter
    auto lp_angular = dynamic_cast<LPTwistFilterObject *>(angular_filter_.get());
    auto fir_angular = dynamic_cast<FIRTwistFilterObject *>(angular_filter_.get());
    auto iir_angular = dynamic_cast<IIRTwistFilterObject *>(angular_filter_.get());

    // Apply linear filter based on its type
    if (lp_linear)
    {
        for (const auto &[key, filter] : lp_linear->linear)
        {
            double input_val = 0.0;
            if (key == "x")
                input_val = data.linear.x;
            else if (key == "y")
                input_val = data.linear.y;
            else if (key == "z")
                input_val = data.linear.z;

            double filtered_val = filter->filter_signal(input_val, time_filter);

            if (key == "x")
                cmd_out.linear.x = filtered_val;
            else if (key == "y")
                cmd_out.linear.y = filtered_val;
            else if (key == "z")
                cmd_out.linear.z = filtered_val;
        }
    }
    else if (fir_linear)
    {
        for (const auto &[key, filter] : fir_linear->linear)
        {
            double input_val = 0.0;
            if (key == "x")
                input_val = data.linear.x;
            else if (key == "y")
                input_val = data.linear.y;
            else if (key == "z")
                input_val = data.linear.z;

            double filtered_val = filter->filter_signal(input_val, time_filter);

            if (key == "x")
                cmd_out.linear.x = filtered_val;
            else if (key == "y")
                cmd_out.linear.y = filtered_val;
            else if (key == "z")
                cmd_out.linear.z = filtered_val;
        }
    }
    else if (iir_linear)
    {
        for (const auto &[key, filter] : iir_linear->linear)
        {
            double input_val = 0.0;
            if (key == "x")
                input_val = data.linear.x;
            else if (key == "y")
                input_val = data.linear.y;
            else if (key == "z")
                input_val = data.linear.z;

            double filtered_val = filter->filter_signal(input_val, time_filter);

            if (key == "x")
                cmd_out.linear.x = filtered_val;
            else if (key == "y")
                cmd_out.linear.y = filtered_val;
            else if (key == "z")
                cmd_out.linear.z = filtered_val;
        }
    }

    // Apply angular filter based on its type
    if (lp_angular)
    {
        for (const auto &[key, filter] : lp_angular->angular)
        {
            double input_val = 0.0;
            if (key == "x")
                input_val = data.angular.x;
            else if (key == "y")
                input_val = data.angular.y;
            else if (key == "z")
                input_val = data.angular.z;

            double filtered_val = filter->filter_signal(input_val, time_filter);

            if (key == "x")
                cmd_out.angular.x = filtered_val;
            else if (key == "y")
                cmd_out.angular.y = filtered_val;
            else if (key == "z")
                cmd_out.angular.z = filtered_val;
        }
    }
    else if (fir_angular)
    {
        for (const auto &[key, filter] : fir_angular->angular)
        {
            double input_val = 0.0;
            if (key == "x")
                input_val = data.angular.x;
            else if (key == "y")
                input_val = data.angular.y;
            else if (key == "z")
                input_val = data.angular.z;

            double filtered_val = filter->filter_signal(input_val, time_filter);

            if (key == "x")
                cmd_out.angular.x = filtered_val;
            else if (key == "y")
                cmd_out.angular.y = filtered_val;
            else if (key == "z")
                cmd_out.angular.z = filtered_val;
        }
    }
    else if (iir_angular)
    {
        for (const auto &[key, filter] : iir_angular->angular)
        {
            double input_val = 0.0;
            if (key == "x")
                input_val = data.angular.x;
            else if (key == "y")
                input_val = data.angular.y;
            else if (key == "z")
                input_val = data.angular.z;

            double filtered_val = filter->filter_signal(input_val, time_filter);

            if (key == "x")
                cmd_out.angular.x = filtered_val;
            else if (key == "y")
                cmd_out.angular.y = filtered_val;
            else if (key == "z")
                cmd_out.angular.z = filtered_val;
        }
    }

    // Apply velocity and acceleration limits
    rclcpp::Time time_now = node_->get_clock()->now();
    double time_delta = (time_now.nanoseconds() - time_prev_.nanoseconds()) / 1e9;

    if (linear_vel_max_ > 0 || angular_vel_max_ > 0)
    {
        cmd_out = _saturate_vel(cmd_out, linear_vel_max_, angular_vel_max_);
    }

    if ((linear_acc_max_ > 0 || angular_acc_max_ > 0) && time_delta > 0.00001)
    {
        cmd_out = _saturate_acc(cmd_out, linear_acc_max_, angular_acc_max_, time_delta);
    }

    twist_prev_ = cmd_out;
    time_prev_ = time_now;

    return cmd_out;
}
void TwistFilter::timer_callback()
{
    pub_cmd();
}

void TwistFilter::_declare_parameters()
{
    rcl_interfaces::msg::ParameterDescriptor double_descriptor;
    double_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;

    node_->declare_parameter("linear_vel_max", 1.0, double_descriptor);
    node_->declare_parameter("linear_acc_max", 1.0, double_descriptor);
    node_->declare_parameter("angular_vel_max", 1.0, double_descriptor);
    node_->declare_parameter("angular_acc_max", 1.0, double_descriptor);
    node_->declare_parameter("timeout", 0.25, double_descriptor);

    linear_vel_max_ = node_->get_parameter("linear_vel_max").as_double();
    linear_acc_max_ = node_->get_parameter("linear_acc_max").as_double();
    angular_vel_max_ = node_->get_parameter("angular_vel_max").as_double();
    angular_acc_max_ = node_->get_parameter("angular_acc_max").as_double();
    timeout_ = node_->get_parameter("timeout").as_double();
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
    rclcpp::Time current_time = node_->get_clock()->now();
    double elapsed = (current_time.nanoseconds() - prev_time_.nanoseconds()) / 1e9;

    geometry_msgs::msg::Twist cmd;
    if (elapsed > timeout_)
    {
        cmd = filter_twist(geometry_msgs::msg::Twist());
    }
    else
    {
        cmd = filter_twist(cmd_);
    }

    bool is_zero_twist = (cmd.linear.x == 0.0 && cmd.linear.y == 0.0 && cmd.linear.z == 0.0 &&
                          cmd.angular.x == 0.0 && cmd.angular.y == 0.0 && cmd.angular.z == 0.0);

    if (is_zero_twist)
    {
        if (!stopped_)
        {
            pub_cmd_out_->publish(cmd);
            stopped_ = true;
        }
    }
    else
    {
        stopped_ = false;
        pub_cmd_out_->publish(cmd);
    }
}

std::pair<double, double> TwistFilter::_get_twist_mag(const geometry_msgs::msg::Twist &v)
{
    double lin = std::sqrt(std::pow(v.linear.x, 2) + std::pow(v.linear.y, 2) + std::pow(v.linear.z, 2));
    double ang = std::sqrt(std::pow(v.angular.x, 2) + std::pow(v.angular.y, 2) + std::pow(v.angular.z, 2));

    return {lin, ang};
}

std::pair<double, double> TwistFilter::_get_max_ratios(double l_mag, double a_mag, double l_max, double a_max)
{
    double l_r = (l_mag <= l_max) ? 1.0 : l_max / l_mag;
    double a_r = (a_mag <= a_max) ? 1.0 : a_max / a_mag;

    return {l_r, a_r};
}

std::vector<double> TwistFilter::_get_scaling_order(const std::vector<double> &ratios)
{
    std::vector<double> valid;
    for (const auto &r : ratios)
    {
        if (r <= 1.0)
        {
            valid.push_back(r);
        }
    }

    if (valid.size() > 1)
    {
        std::sort(valid.begin(), valid.end());
    }

    return valid;
}

geometry_msgs::msg::Twist TwistFilter::_saturate_vel(const geometry_msgs::msg::Twist &v, double l_max, double a_max)
{
    geometry_msgs::msg::Twist sat_twist = v;

    double mag = _get_mag(sat_twist.linear);
    if (mag > l_max)
    {
        double ratio = l_max / mag;
        sat_twist.linear.x *= ratio;
        sat_twist.linear.y *= ratio;
        sat_twist.linear.z *= ratio;
    }

    mag = _get_mag(sat_twist.angular);
    if (mag > a_max)
    {
        double ratio = a_max / mag;
        sat_twist.angular.x *= ratio;
        sat_twist.angular.y *= ratio;
        sat_twist.angular.z *= ratio;
    }

    return sat_twist;
}

double TwistFilter::_get_mag(const geometry_msgs::msg::Vector3 &v_comp)
{
    return std::sqrt(std::pow(v_comp.x, 2) + std::pow(v_comp.y, 2) + std::pow(v_comp.z, 2));
}

geometry_msgs::msg::Twist TwistFilter::_saturate_acc(const geometry_msgs::msg::Twist &v, double l_max, double a_max, double time_delta)
{
    geometry_msgs::msg::Twist sat_twist = v;

    geometry_msgs::msg::Twist acc = _get_acc(sat_twist, time_delta);

    double mag = _get_mag(acc.linear);
    if (mag > l_max)
    {
        double ratio = l_max / mag;
        acc.linear.x *= ratio;
        acc.linear.y *= ratio;
        acc.linear.z *= ratio;
    }

    mag = _get_mag(acc.angular);
    if (mag > a_max)
    {
        double ratio = a_max / mag;
        acc.angular.x *= ratio;
        acc.angular.y *= ratio;
        acc.angular.z *= ratio;
    }

    sat_twist.linear.x = (acc.linear.x * time_delta) + twist_prev_.linear.x;
    sat_twist.linear.y = (acc.linear.y * time_delta) + twist_prev_.linear.y;
    sat_twist.linear.z = (acc.linear.z * time_delta) + twist_prev_.linear.z;
    sat_twist.angular.x = (acc.angular.x * time_delta) + twist_prev_.angular.x;
    sat_twist.angular.y = (acc.angular.y * time_delta) + twist_prev_.angular.y;
    sat_twist.angular.z = (acc.angular.z * time_delta) + twist_prev_.angular.z;

    return sat_twist;
}

geometry_msgs::msg::Twist TwistFilter::_get_acc(const geometry_msgs::msg::Twist &v, double time_delta)
{
    geometry_msgs::msg::Twist acc;

    acc.linear.x = _get_slope(v.linear.x, twist_prev_.linear.x, time_delta);
    acc.linear.y = _get_slope(v.linear.y, twist_prev_.linear.y, time_delta);
    acc.linear.z = _get_slope(v.linear.z, twist_prev_.linear.z, time_delta);
    acc.angular.x = _get_slope(v.angular.x, twist_prev_.angular.x, time_delta);
    acc.angular.y = _get_slope(v.angular.y, twist_prev_.angular.y, time_delta);
    acc.angular.z = _get_slope(v.angular.z, twist_prev_.angular.z, time_delta);

    return acc;
}

double TwistFilter::_get_slope(double current, double prev, double step)
{
    return (current - prev) / step;
}
