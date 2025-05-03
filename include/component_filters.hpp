#pragma once

#include <map>
#include <string>
#include <vector>
#include <memory>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter.hpp"
#include "geometry_msgs/msg/vector3.hpp"

class FilterBase;
class FIRFilter;
class IIRFilter;
class LPFilter;

class TwistFilterObjectBase
{
public:
  virtual ~TwistFilterObjectBase() = default;
  virtual bool update_filters(const std::vector<rclcpp::Parameter> &parameters) = 0;
  virtual void filter_vector(const geometry_msgs::msg::Vector3 &input,
                             geometry_msgs::msg::Vector3 &output,
                             int64_t time_ns) = 0;
};

class FilterBase
{
public:
  FilterBase(int num_samples);
  virtual ~FilterBase() = default;

  std::string to_string() const;
  void update_samples(double data);
  virtual double get_result() = 0;
  virtual double filter_signal(double data, int64_t time);
  virtual void reset(int num_samples, const std::vector<double> &weights);

protected:
  int num_samples_;
  std::vector<double> samples_;
  double last_sent_vel_;
};

class FIRFilter : public FilterBase
{
public:
  FIRFilter(int num_samples, const std::vector<double> &weights);
  double get_result() override;
  void reset(int num_samples, const std::vector<double> &weights) override;

private:
  std::vector<double> weights_;
};

class IIRFilter : public FilterBase
{
public:
  IIRFilter(int num_samples, const std::vector<double> &weights,
            int num_out_samples, const std::vector<double> &out_weights);

  void update_feedback(double data);
  double get_result() override;
  double filter_signal(double data, int64_t time) override;
  void reset(int num_samples, const std::vector<double> &weights,
             int num_out_samples, const std::vector<double> &out_weights);

private:
  int num_out_samples_;
  std::vector<double> out_samples_;
  std::vector<double> weights_;
  std::vector<double> out_weights_;
};

class LPFilter
{
public:
  LPFilter(double tau, double damping_factor);
  double filter_signal(double data, int64_t current_time);
  void reset(double tau, double damping);

private:
  double low_pass_filter(double acceleration, double dt);

  double tau_;
  double damping_factor_;
  double acceleration_;
  double prev_acceleration_;
  int64_t prev_time_;
  double last_sent_velocity_;
};

class FIRTwistFilterObject : public TwistFilterObjectBase
{
public:
  virtual ~FIRTwistFilterObject();
  FIRTwistFilterObject(const std::map<std::string, std::map<std::string, bool>> &active_filters,
                       const std::map<std::string, std::string> &config);

  bool update_filters(const std::vector<rclcpp::Parameter> &parameters) override;
  void reset_filters(int num_samples, const std::vector<double> &weights);
  void filter_vector(const geometry_msgs::msg::Vector3 &input,
                     geometry_msgs::msg::Vector3 &output,
                     int64_t time_ns) override;

  std::map<std::string, std::shared_ptr<FIRFilter>> linear;
  std::map<std::string, std::shared_ptr<FIRFilter>> angular;
  int num_samples;
  std::vector<double> weights;
};

class LPTwistFilterObject : public TwistFilterObjectBase
{
public:
  virtual ~LPTwistFilterObject();
  LPTwistFilterObject(const std::map<std::string, std::map<std::string, bool>> &active_filters,
                      const std::map<std::string, double> &config);

  bool update_filters(const std::vector<rclcpp::Parameter> &parameters) override;
  void reset_filters(double tau, double damping);
  void filter_vector(const geometry_msgs::msg::Vector3 &input,
                     geometry_msgs::msg::Vector3 &output,
                     int64_t time_ns) override;

  std::map<std::string, std::shared_ptr<LPFilter>> linear;
  std::map<std::string, std::shared_ptr<LPFilter>> angular;
  double tau_param_;
  double damping_param_;
};

class IIRTwistFilterObject : public TwistFilterObjectBase
{
public:
  virtual ~IIRTwistFilterObject();
  IIRTwistFilterObject(const std::map<std::string, std::map<std::string, bool>> &active_filters,
                       const std::map<std::string, std::string> &config);

  bool update_filters(const std::vector<rclcpp::Parameter> &parameters) override;
  void reset_filters(int samples, const std::vector<double> &weights,
                     int out_samples, const std::vector<double> &out_weights);
  void filter_vector(const geometry_msgs::msg::Vector3 &input,
                     geometry_msgs::msg::Vector3 &output,
                     int64_t time_ns) override;

  std::map<std::string, std::shared_ptr<IIRFilter>> linear;
  std::map<std::string, std::shared_ptr<IIRFilter>> angular;
  int num_samples;
  std::vector<double> weights;
  int num_out_samples;
  std::vector<double> out_weights;
};
