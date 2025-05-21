// component_filters.cpp
#include "component_filters.hpp"
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <algorithm>

using json = nlohmann::json;

FilterBase::FilterBase(int num_samples) : num_samples_(num_samples), last_sent_vel_(0.0)
{
    samples_.resize(num_samples_, 0.0);
}

void FilterBase::update_samples(double data)
{
    for (int i = num_samples_ - 2; i >= 0; --i)
        samples_[i + 1] = samples_[i];
    samples_[0] = data;
}

void FilterBase::reset(int num_samples, const std::vector<double> &weights)
{
    num_samples_ = num_samples;
    samples_.assign(num_samples_, 0.0);
}

void FilterBase::reset_state()
{
    last_sent_vel_ = 0.0;
}

double FilterBase::filter_signal(double data, int64_t time)
{
    update_samples(data);
    return get_result();
}

FIRFilter::FIRFilter(int num_samples, const std::vector<double> &weights)
    : FilterBase(num_samples), weights_(weights) {}

double FIRFilter::get_result()
{
    double result = 0.0;
    for (size_t i = 0; i < samples_.size(); ++i)
        result += samples_[i] * (i < weights_.size() ? weights_[i] : 1.0);
    return weights_.empty() ? result / samples_.size() : result;
}

void FIRFilter::reset(int num_samples, const std::vector<double> &weights)
{
    RCLCPP_INFO(rclcpp::get_logger("twist_filter"), "Resetting filter with %d samples", num_samples);
    FilterBase::reset(num_samples, weights);
    weights_ = weights;
}

IIRFilter::IIRFilter(int num_samples, const std::vector<double> &weights,
                     int num_out_samples, const std::vector<double> &out_weights)
    : FilterBase(num_samples), num_out_samples_(num_out_samples), weights_(weights), out_weights_(out_weights)
{
    out_samples_.assign(num_out_samples_, 0.0);
}

void IIRFilter::update_feedback(double data)
{
    for (int i = num_out_samples_ - 2; i >= 0; --i)
        out_samples_[i + 1] = out_samples_[i];
    out_samples_[0] = data;
}

double IIRFilter::get_result()
{
    double input_response = 0.0, feedback_response = 0.0;
    for (size_t i = 0; i < samples_.size(); ++i)
        input_response += samples_[i] * (i < weights_.size() ? weights_[i] : 1.0);
    for (size_t i = 0; i < out_samples_.size(); ++i)
        feedback_response += out_samples_[i] * (i < out_weights_.size() ? out_weights_[i] : 1.0);
    return input_response - feedback_response;
}

double IIRFilter::filter_signal(double data, int64_t time)
{
    update_samples(data);
    double result = get_result();
    if (std::abs(result) < 0.001) result = 0.0;
    update_feedback(result);
    return result;
}

void IIRFilter::reset(int num_samples, const std::vector<double> &weights,
                      int num_out_samples, const std::vector<double> &out_weights)
{
    RCLCPP_INFO(rclcpp::get_logger("twist_filter"), "Resetting filter with %d samples", num_samples);
    RCLCPP_INFO(rclcpp::get_logger("twist_filter"), "Out samples: %d", num_out_samples);
    FilterBase::reset(num_samples, weights);
    weights_ = weights;
    num_out_samples_ = num_out_samples;
    out_weights_ = out_weights;
    out_samples_.assign(num_out_samples_, 0.0);
}

LPFilter::LPFilter(double tau, double damping)
    : tau_(tau), damping_factor_(damping), prev_acceleration_(0.0), prev_time_(0), last_sent_velocity_(0.0) {}

double LPFilter::low_pass_filter(double acceleration, double dt)
{
    double alpha = 1.0 - std::exp(-dt / tau_);
    return prev_acceleration_ + alpha * (acceleration - prev_acceleration_);
}

double LPFilter::filter_signal(double data, int64_t current_time)
{
    double dt = (current_time - prev_time_) / 1e9;
    if (dt <= 0) return last_sent_velocity_;

    double acc = (data - last_sent_velocity_) / dt;
    double filtered_acc = low_pass_filter(acc, dt);
    double smoothed_vel = last_sent_velocity_ + filtered_acc * dt;

    prev_acceleration_ = filtered_acc;
    prev_time_ = current_time;
    last_sent_velocity_ = damping_factor_ * last_sent_velocity_ + (1 - damping_factor_) * smoothed_vel;

    return last_sent_velocity_;
}

void LPFilter::reset(double tau, double damping)
{
    RCLCPP_INFO(rclcpp::get_logger("twist_filter"), "Resetting filter with tau %f", tau);
    RCLCPP_INFO(rclcpp::get_logger("twist_filter"), "Damping factor %f", damping);
    tau_ = tau;
    damping_factor_ = damping;
}

void LPFilter::reset_state()
{
    prev_acceleration_ = 0.0;
    prev_time_ = 0;
    last_sent_velocity_ = 0.0;
}

FIRTwistFilterObject::FIRTwistFilterObject(
    const std::map<std::string, std::map<std::string, bool>> &active_filters,
    const std::map<std::string, std::string> &config)
{
    num_samples = std::stoi(config.at("num_samples"));
    weights = json::parse(config.at("weights")).get<std::vector<double>>();

    for (const auto &[axis, enabled] : active_filters.at("linear"))
        if (enabled) linear[axis] = std::make_shared<FIRFilter>(num_samples, weights);

    for (const auto &[axis, enabled] : active_filters.at("angular"))
        if (enabled) angular[axis] = std::make_shared<FIRFilter>(num_samples, weights);
}

void FIRTwistFilterObject::reset_state()
{
    for (auto &[_, filter] : linear) filter->reset_state();
    for (auto &[_, filter] : angular) filter->reset_state();
}

bool FIRTwistFilterObject::update_filters(const std::vector<rclcpp::Parameter> &parameters)
{
    int new_num_samples = -1;
    std::string weights_str;

    for (const auto &param : parameters)
    {
        if (param.get_name().find("num_samples") != std::string::npos)
            new_num_samples = param.as_int();
        else if (param.get_name().find("weights") != std::string::npos)
            weights_str = param.as_string();
    }

    if (!weights_str.empty())
    {
        weights = json::parse(weights_str).get<std::vector<double>>();
        num_samples = weights.size();
    }
    else if (new_num_samples > 0)
    {
        weights.assign(new_num_samples, 1.0);
        num_samples = new_num_samples;
    }
    else
    {
        return false;
    }

    for (auto &[_, filter] : linear) filter->reset(num_samples, weights);
    for (auto &[_, filter] : angular) filter->reset(num_samples, weights);

    return true;
}

IIRTwistFilterObject::IIRTwistFilterObject(
    const std::map<std::string, std::map<std::string, bool>> &active_filters,
    const std::map<std::string, std::string> &config)
{
    num_samples = std::stoi(config.at("num_samples"));
    weights = json::parse(config.at("weights")).get<std::vector<double>>();

    num_out_samples = std::stoi(config.at("num_out_samples"));
    out_weights = json::parse(config.at("out_weights")).get<std::vector<double>>();


    for (const auto &[axis, enabled] : active_filters.at("linear"))
        if (enabled) linear[axis] = std::make_shared<IIRFilter>(num_samples, weights, num_out_samples, out_weights);

    for (const auto &[axis, enabled] : active_filters.at("angular"))
        if (enabled) angular[axis] = std::make_shared<IIRFilter>(num_samples, weights, num_out_samples, out_weights);
}

void IIRTwistFilterObject::reset_state()
{
    for (auto &[_, filter] : linear) filter->reset_state();
    for (auto &[_, filter] : angular) filter->reset_state();
}

bool IIRTwistFilterObject::update_filters(const std::vector<rclcpp::Parameter> &parameters)
{
    std::string weights_str, out_weights_str;
    int new_num_samples = -1, new_out_samples = -1;

    for (const auto &param : parameters)
    {
        if (param.get_name().find("num_samples") != std::string::npos)
            new_num_samples = param.as_int();
        else if (param.get_name().find("weights") != std::string::npos)
            weights_str = param.as_string();
        else if (param.get_name().find("num_out_samples") != std::string::npos)
            new_out_samples = param.as_int();
        else if (param.get_name().find("out_weights") != std::string::npos)
            out_weights_str = param.as_string();
    }

    if (!weights_str.empty()) weights = json::parse(weights_str).get<std::vector<double>>();
    if (!out_weights_str.empty()) out_weights = json::parse(out_weights_str).get<std::vector<double>>();
    if (new_num_samples > 0) num_samples = new_num_samples;
    if (new_out_samples > 0) num_out_samples = new_out_samples;

    for (auto &[_, filter] : linear)
        filter->reset(num_samples, weights, num_out_samples, out_weights);
    for (auto &[_, filter] : angular)
        filter->reset(num_samples, weights, num_out_samples, out_weights);

    return true;
}

LPTwistFilterObject::LPTwistFilterObject(
    const std::map<std::string, std::map<std::string, bool>> &active_filters,
    const std::map<std::string, double> &config)
{
    tau_param_ = config.at("tau");
    damping_param_ = config.at("damping");

    for (const auto &[axis, enabled] : active_filters.at("linear"))
        if (enabled) linear[axis] = std::make_shared<LPFilter>(tau_param_, damping_param_);

    for (const auto &[axis, enabled] : active_filters.at("angular"))
        if (enabled) angular[axis] = std::make_shared<LPFilter>(tau_param_, damping_param_);
}

bool LPTwistFilterObject::update_filters(const std::vector<rclcpp::Parameter> &parameters)
{
    double tau = -1.0, damping = -1.0;

    for (const auto &param : parameters)
    {
        if (param.get_name().find("tau") != std::string::npos)
            tau = param.as_double();
        else if (param.get_name().find("damping") != std::string::npos)
            damping = param.as_double();
    }

    if (tau > 0.0) tau_param_ = tau;
    if (damping > 0.0) damping_param_ = damping;

    for (auto &[_, filter] : linear)
        filter->reset(tau_param_, damping_param_);
    for (auto &[_, filter] : angular)
        filter->reset(tau_param_, damping_param_);

    return true;
}

void LPTwistFilterObject::reset_state()
{
    for (auto &[_, filter] : linear) filter->reset_state();
    for (auto &[_, filter] : angular) filter->reset_state();
}

template <typename FilterT>
void filter_vector_helper(const std::map<std::string, std::shared_ptr<FilterT>> &filters,
                          const geometry_msgs::msg::Vector3 &input,
                          geometry_msgs::msg::Vector3 &output,
                          int64_t time_ns)
{
    for (const auto &[key, filter] : filters)
    {
        if (key == "x") output.x = filter->filter_signal(input.x, time_ns);
        else if (key == "y") output.y = filter->filter_signal(input.y, time_ns);
        else if (key == "z") output.z = filter->filter_signal(input.z, time_ns);
    }
}

void FIRTwistFilterObject::filter_vector(const geometry_msgs::msg::Vector3 &input,
                                         geometry_msgs::msg::Vector3 &output,
                                         int64_t time_ns)
{
    filter_vector_helper(linear, input, output, time_ns);
    filter_vector_helper(angular, input, output, time_ns);
}

void IIRTwistFilterObject::filter_vector(const geometry_msgs::msg::Vector3 &input,
                                         geometry_msgs::msg::Vector3 &output,
                                         int64_t time_ns)
{
    filter_vector_helper(linear, input, output, time_ns);
    filter_vector_helper(angular, input, output, time_ns);
}

void LPTwistFilterObject::filter_vector(const geometry_msgs::msg::Vector3 &input,
                                        geometry_msgs::msg::Vector3 &output,
                                        int64_t time_ns)
{
    filter_vector_helper(linear, input, output, time_ns);
    filter_vector_helper(angular, input, output, time_ns);
}

FIRTwistFilterObject::~FIRTwistFilterObject() = default;
IIRTwistFilterObject::~IIRTwistFilterObject() = default;
LPTwistFilterObject::~LPTwistFilterObject() = default;
