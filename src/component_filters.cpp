// component_filters.cpp
#include "component_filters.hpp"
#include <nlohmann/json.hpp>
#include <sstream>
#include <algorithm>

using json = nlohmann::json;

FilterBase::FilterBase(int num_samples)
    : num_samples_(num_samples), last_sent_vel_(0.0)
{
    samples_.resize(num_samples_, 0.0);
}

std::string FilterBase::to_string() const
{
    std::stringstream ss;
    ss << "[";
    for (size_t i = 0; i < samples_.size(); ++i)
    {
        ss << samples_[i];
        if (i < samples_.size() - 1)
        {
            ss << ", ";
        }
    }
    ss << "]";
    return ss.str();
}

void FilterBase::update_samples(double data)
{
    if (samples_.size() > 1)
    {
        for (int i = num_samples_ - 2; i >= 0; --i)
        {
            samples_[i + 1] = samples_[i];
        }
    }
    samples_[0] = data;
}

double FilterBase::get_result()
{
    double result = 0.0;
    for (const auto &sample : samples_)
    {
        result += sample;
    }
    return result / samples_.size();
}

double FilterBase::filter_signal(double data, int64_t time)
{
    update_samples(data);
    return get_result();
}

void FilterBase::reset(int num_samples, const std::vector<double> &weights)
{
    num_samples_ = num_samples;
    samples_.clear();
    samples_.resize(num_samples_, 0.0);
}

FIRFilter::FIRFilter(int num_samples, const std::vector<double> &weights)
    : FilterBase(num_samples), weights_(weights)
{
}

double FIRFilter::get_result()
{
    double result = 0.0;
    if (!weights_.empty())
    {
        for (size_t i = 0; i < samples_.size(); ++i)
        {
            result += samples_[i] * weights_[i];
        }
    }
    else
    {
        for (const auto &sample : samples_)
        {
            result += sample;
        }
        result = result / samples_.size();
    }
    return result;
}

void FIRFilter::reset(int num_samples, const std::vector<double> &weights)
{
    FilterBase::reset(num_samples, weights);
    weights_ = weights;
}

IIRFilter::IIRFilter(int num_samples, const std::vector<double> &weights,
                     int num_out_samples, const std::vector<double> &out_weights)
    : FilterBase(num_samples), num_out_samples_(num_out_samples),
      weights_(weights), out_weights_(out_weights)
{
    out_samples_.resize(num_out_samples_, 0.0);
}

void IIRFilter::update_feedback(double data)
{
    if (out_samples_.size() > 1)
    {
        for (int i = num_out_samples_ - 2; i >= 0; --i)
        {
            out_samples_[i + 1] = out_samples_[i];
        }
    }
    out_samples_[0] = data;
}

double IIRFilter::get_result()
{
    double input_response = 0.0;
    double feedback_response = 0.0;

    if (!weights_.empty())
    {
        for (size_t i = 0; i < samples_.size(); ++i)
        {
            input_response += samples_[i] * weights_[i];
        }
    }
    else
    {
        for (const auto &sample : samples_)
        {
            input_response += sample;
        }
        input_response = input_response / samples_.size();
    }

    if (!out_weights_.empty())
    {
        for (size_t i = 0; i < out_samples_.size(); ++i)
        {
            feedback_response += out_samples_[i] * out_weights_[i];
        }
    }
    else
    {
        for (const auto &sample : out_samples_)
        {
            feedback_response += sample;
        }
        feedback_response = feedback_response / out_samples_.size();
    }

    return input_response - feedback_response;
}

double IIRFilter::filter_signal(double data, int64_t time)
{
    update_samples(data);
    double result = get_result();

    if (std::abs(result) < 0.001)
    {
        result = 0.0;
    }

    update_feedback(result);
    return result;
}

void IIRFilter::reset(int num_samples, const std::vector<double> &weights,
                      int num_out_samples, const std::vector<double> &out_weights)
{
    FilterBase::reset(num_samples, weights);
    weights_ = weights;

    num_out_samples_ = num_out_samples;
    out_samples_.clear();
    out_samples_.resize(num_out_samples_, 0.0);
    out_weights_ = out_weights;
}

LPFilter::LPFilter(double tau, double damping_factor)
    : tau_(tau), damping_factor_(damping_factor), acceleration_(0.0),
      prev_acceleration_(0.0), prev_time_(0), last_sent_velocity_(0.0)
{
}

double LPFilter::filter_signal(double data, int64_t current_time)
{
    double dt = static_cast<double>(current_time - prev_time_) / 1e9;
    if (dt <= 0)
    {
        return last_sent_velocity_;
    }

    double current_acceleration = (data - last_sent_velocity_) / dt;
    double filtered_acceleration = low_pass_filter(current_acceleration, dt);
    prev_acceleration_ = filtered_acceleration;

    double smoothed_velocity = last_sent_velocity_ + filtered_acceleration * dt;
    prev_time_ = current_time;
    double damped_velocity = damping_factor_ * last_sent_velocity_ +
                             (1.0 - damping_factor_) * smoothed_velocity;

    last_sent_velocity_ = damped_velocity;
    return damped_velocity;
}

double LPFilter::low_pass_filter(double acceleration, double dt)
{
    double alpha = 1.0 - std::exp(-dt / tau_);
    return prev_acceleration_ + alpha * (acceleration - prev_acceleration_);
}

void LPFilter::reset(double tau, double damping)
{
    tau_ = tau;
    damping_factor_ = damping;
}

FIRTwistFilterObject::FIRTwistFilterObject(
    const std::map<std::string, std::map<std::string, bool>> &active_filters,
    const std::map<std::string, std::string> &config)
{
    auto logger = rclcpp::get_logger("FIRTwistFilterObject");

    num_samples = std::stoi(config.at("num_samples"));
    weights = json::parse(config.at("weights")).get<std::vector<double>>();

    const auto &lin_config = active_filters.at("linear");
    const auto &ang_config = active_filters.at("angular");

    for (const auto &[key, active] : lin_config)
    {
        if (active)
        {
            linear[key] = std::make_shared<FIRFilter>(num_samples, weights);
            RCLCPP_INFO(logger, "Created component filter for linear.%s", key.c_str());
        }
    }

    for (const auto &[key, active] : ang_config)
    {
        if (active)
        {
            angular[key] = std::make_shared<FIRFilter>(num_samples, weights);
            RCLCPP_INFO(logger, "Created component filter for angular.%s", key.c_str());
        }
    }
}

bool FIRTwistFilterObject::update_filters(const std::vector<rclcpp::Parameter> &parameters)
{
    auto logger = rclcpp::get_logger("FIRTwistFilterObject");
    int new_num_samples = -1;
    std::string weights_str;

    for (const auto &param : parameters)
    {
        RCLCPP_INFO(logger, "param: %s", param.get_name().c_str());
        if (param.get_name().find("num_samples") != std::string::npos)
        {
            new_num_samples = param.as_int();
            RCLCPP_INFO(logger, "Filter Reconfigure: Number of samples: %d", new_num_samples);
        }
        else if (param.get_name().find("weights") != std::string::npos)
        {
            weights_str = param.as_string();
            RCLCPP_INFO(logger, "Filter Reconfigure: Weights: %s", weights_str.c_str());
        }
    }

    std::vector<double> new_weights;
    if (new_num_samples == -1 && weights_str.empty())
    {
        RCLCPP_WARN(logger, "No updates made to FIR filter.");
        return true;
    }

    if (new_num_samples == -1)
    {
        if (!weights_str.empty())
        {
            new_weights = json::parse(weights_str).get<std::vector<double>>();
            new_num_samples = new_weights.size();
        }
    }

    if (new_weights.empty())
    {
        weights_str = "[]";
    }

    if (new_num_samples == static_cast<int>(new_weights.size()) || new_weights.empty())
    {
        num_samples = new_num_samples;
        weights = new_weights;
        reset_filters(num_samples, weights);
    }
    else
    {
        RCLCPP_WARN(logger, "Could not update filter. Make sure number of samples and respective weights match.");
    }

    return true;
}

void FIRTwistFilterObject::reset_filters(int num_samples, const std::vector<double> &weights)
{
    for (auto &[key, filter] : linear)
    {
        filter->reset(num_samples, weights);
    }
    for (auto &[key, filter] : angular)
    {
        filter->reset(num_samples, weights);
    }

    auto logger = rclcpp::get_logger("FIRTwistFilterObject");
    RCLCPP_INFO(logger, "Component filters reset, Sample number: %d", num_samples);
}

LPTwistFilterObject::LPTwistFilterObject(
    const std::map<std::string, std::map<std::string, bool>> &active_filters,
    const std::map<std::string, double> &config)
{
    auto logger = rclcpp::get_logger("LPTwistFilterObject");

    tau = config.at("tau");
    damping = config.at("damping");

    const auto &lin_config = active_filters.at("linear");
    const auto &ang_config = active_filters.at("angular");

    for (const auto &[key, active] : lin_config)
    {
        if (active)
        {
            linear[key] = std::make_shared<LPFilter>(tau, damping);
            RCLCPP_INFO(logger, "Created component filter for linear.%s", key.c_str());
        }
    }

    for (const auto &[key, active] : ang_config)
    {
        if (active)
        {
            angular[key] = std::make_shared<LPFilter>(tau, damping);
            RCLCPP_INFO(logger, "Created component filter for angular.%s", key.c_str());
        }
    }
}

bool LPTwistFilterObject::update_filters(const std::vector<rclcpp::Parameter> &parameters)
{
    auto logger = rclcpp::get_logger("LPTwistFilterObject");
    double new_tau = -1.0;
    double new_damping = -1.0;

    for (const auto &param : parameters)
    {
        if (param.get_name().find("tau") != std::string::npos)
        {
            if (param.as_double() <= 1.0)
            {
                new_tau = param.as_double();
                RCLCPP_INFO(logger, "Filter Reconfigure: Tau: %f", new_tau);
            }
        }
        else if (param.get_name().find("damping") != std::string::npos)
        {
            if (param.as_double() <= 1.0)
            {
                new_damping = param.as_double();
                RCLCPP_INFO(logger, "Filter Reconfigure: Damping: %f", new_damping);
            }
        }
    }

    if (new_tau > 0)
    {
        reset_filters(new_tau, damping);
    }
    else if (new_damping > 0)
    {
        reset_filters(tau, new_damping);
    }
    else
    {
        RCLCPP_WARN(logger, "No updates made to LP filter.");
    }

    return true;
}

void LPTwistFilterObject::reset_filters(double tau, double damping)
{
    for (auto &[key, filter] : linear)
    {
        filter->reset(tau, damping);
    }
    for (auto &[key, filter] : angular)
    {
        filter->reset(tau, damping);
    }

    auto logger = rclcpp::get_logger("LPTwistFilterObject");
    RCLCPP_INFO(logger, "Component filters reset, Tau: %f, damping: %f", tau, damping);
}

IIRTwistFilterObject::IIRTwistFilterObject(
    const std::map<std::string, std::map<std::string, bool>> &active_filters,
    const std::map<std::string, std::string> &config)
{
    auto logger = rclcpp::get_logger("IIRTwistFilterObject");

    num_samples = std::stoi(config.at("num_samples"));
    weights = json::parse(config.at("weights")).get<std::vector<double>>();
    num_out_samples = std::stoi(config.at("num_out_samples"));
    out_weights = json::parse(config.at("out_weights")).get<std::vector<double>>();

    const auto &lin_config = active_filters.at("linear");
    const auto &ang_config = active_filters.at("angular");

    for (const auto &[key, active] : lin_config)
    {
        if (active)
        {
            linear[key] = std::make_shared<IIRFilter>(num_samples, weights, num_out_samples, out_weights);
            RCLCPP_INFO(logger, "Created component filter for linear.%s", key.c_str());
        }
    }

    for (const auto &[key, active] : ang_config)
    {
        if (active)
        {
            angular[key] = std::make_shared<IIRFilter>(num_samples, weights, num_out_samples, out_weights);
            RCLCPP_INFO(logger, "Created component filter for angular.%s", key.c_str());
        }
    }
}

bool IIRTwistFilterObject::update_filters(const std::vector<rclcpp::Parameter> &parameters)
{
    auto logger = rclcpp::get_logger("IIRTwistFilterObject");
    int new_num_samples = -1;
    std::string weights_str;
    int new_num_out_samples = -1;
    std::string out_weights_str;

    for (const auto &param : parameters)
    {
        if (param.get_name().find("num_samples") != std::string::npos)
        {
            new_num_samples = param.as_int();
            RCLCPP_INFO(logger, "Filter Reconfigure: Number of samples: %d", new_num_samples);
        }
        else if (param.get_name().find("weights") != std::string::npos)
        {
            weights_str = param.as_string();
            RCLCPP_INFO(logger, "Filter Reconfigure: Weights: %s", weights_str.c_str());
        }
        else if (param.get_name().find("num_out_samples") != std::string::npos)
        {
            new_num_out_samples = param.as_int();
            RCLCPP_INFO(logger, "Filter Reconfigure: Number of output samples: %d", new_num_out_samples);
        }
        else if (param.get_name().find("out_weights") != std::string::npos)
        {
            out_weights_str = param.as_string();
            RCLCPP_INFO(logger, "Filter Reconfigure: Output weights: %s", out_weights_str.c_str());
        }
    }

    if (new_num_samples > 0 || !weights_str.empty())
    {
        std::vector<double> new_weights;
        if (new_num_samples == -1)
        {
            if (!weights_str.empty())
            {
                new_weights = json::parse(weights_str).get<std::vector<double>>();
                new_num_samples = new_weights.size();
            }
        }

        if (new_weights.empty())
        {
            weights_str = "[]";
        }

        if (new_num_samples == static_cast<int>(new_weights.size()) || new_weights.empty())
        {
            num_samples = new_num_samples;
            weights = new_weights;
        }
    }
    else if (new_num_out_samples > 0 || !out_weights_str.empty())
    {
        RCLCPP_INFO(logger, "else");
        std::vector<double> new_out_weights;
        if (new_num_out_samples == -1)
        {
            if (!out_weights_str.empty())
            {
                new_out_weights = json::parse(out_weights_str).get<std::vector<double>>();
                new_num_out_samples = new_out_weights.size();
            }
        }

        if (new_out_weights.empty())
        {
            out_weights_str = "[]";
        }

        if (new_num_out_samples == static_cast<int>(new_out_weights.size()) || new_out_weights.empty())
        {
            num_out_samples = new_num_out_samples;
            out_weights = new_out_weights;
        }
    }
    else
    {
        RCLCPP_WARN(logger, "No updates made to IIR filter.");
        return true;
    }

    RCLCPP_INFO(logger, "output %d, %s, %s, %d", num_samples, "weights", "out_weights", num_out_samples);

    reset_filters(num_samples, weights, num_out_samples, out_weights);
    return true;
}

void IIRTwistFilterObject::reset_filters(int samples, const std::vector<double> &weights,
                                         int out_samples, const std::vector<double> &out_weights)
{
    for (auto &[key, filter] : linear)
    {
        filter->reset(samples, weights, out_samples, out_weights);
    }
    for (auto &[key, filter] : angular)
    {
        filter->reset(samples, weights, out_samples, out_weights);
    }

    auto logger = rclcpp::get_logger("IIRTwistFilterObject");
    RCLCPP_INFO(logger, "Component filters reset, Sample number %d, out sample number %d",
                samples, out_samples);
}
