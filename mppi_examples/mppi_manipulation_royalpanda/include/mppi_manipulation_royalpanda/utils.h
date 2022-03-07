//
// Created by giuseppe on 17.08.21.
//

#pragma once
#include <regex>
#include <ros/ros.h>

// for pseudoinverse
#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/SVD>

#define NAMED_LOG_INFO(...) manipulation_royalpanda::namedLog(LogType::INFO, __PRETTY_FUNCTION__, __VA_ARGS__)
#define NAMED_LOG_WARN(...) manipulation_royalpanda::namedLog(LogType::WARN, __PRETTY_FUNCTION__, __VA_ARGS__)
#define NAMED_LOG_ERROR(...) manipulation_royalpanda::namedLog(LogType::ERROR, __PRETTY_FUNCTION__, __VA_ARGS__)

namespace manipulation_royalpanda{

enum LogType{
  INFO,
  WARN,
  ERROR,
};

template <typename T>
void append(std::ostringstream& os, const T& t){
  os << t;
}

  template <typename T, typename ...Args>
void append(std::ostringstream& os, const T& t, Args... args){
  os << t;
  append(os, args...);
}

template <typename ...Args>
void namedLog(LogType type, const char* prefix, Args&& ...args){
  static std::string delimiter{"::"};
  std::string function_name;
  std::string class_name;

  std::ostringstream  os;
  std::string prefix_str(prefix);

  size_t pos = 0;
  std::string token;
  int index = 0;
  if ((pos = prefix_str.rfind(delimiter)) != std::string::npos) {
    size_t pos2 =  prefix_str.find("(");
    function_name = prefix_str.substr(pos + delimiter.length(), pos2);
    prefix_str.erase(pos, std::string::npos);
  }

  if ((pos = prefix_str.rfind(delimiter)) != std::string::npos) {
    class_name = prefix_str.substr(pos + delimiter.length(), std::string::npos);
  }

  os << "[" << class_name << "::" << function_name << "] ";
  switch (type){
    case INFO:
      append(os, args...);
      ROS_INFO_STREAM(os.str());
    case WARN:
      append(os, args...);
      ROS_WARN_STREAM(os.str());
    case ERROR:
      append(os, args...);
      ROS_ERROR_STREAM(os.str());
  }
}

template<typename T>
bool are_equal(T lhs, T rhs){
  return std::abs(lhs - rhs) < 0.001;
}

template<typename T, typename ...Args>
bool are_equal(T lhs, T rhs, Args... args){
  return are_equal(lhs, rhs) * are_equal(lhs, args...);
}

inline void pseudoInverse(const Eigen::MatrixXd& M_, Eigen::MatrixXd& M_pinv_, bool damped = true) {
  double lambda_ = damped ? 0.2 : 0.0;

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(M_, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType sing_vals_ = svd.singularValues();
  Eigen::MatrixXd S_ = M_;  // copying the dimensions of M_, its content is not needed.
  S_.setZero();

  for (int i = 0; i < sing_vals_.size(); i++)
    S_(i, i) = (sing_vals_(i)) / (sing_vals_(i) * sing_vals_(i) + lambda_ * lambda_);

  M_pinv_ = Eigen::MatrixXd(svd.matrixV() * S_.transpose() * svd.matrixU().transpose());
}

}