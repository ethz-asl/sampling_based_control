//
// Created by giuseppe on 17.08.21.
//

#pragma once
#include <regex>
#include <ros/ros.h>
#define NAMED_LOG_INFO(...) manipulation_royalpanda::namedLog(LogType::INFO, __PRETTY_FUNCTION__, __VA_ARGS__)
#define NAMED_LOG_WARN(...) manipulation_royalpanda::namedLog(LogType::WARN, __PRETTY_FUNCTION__, __VA_ARGS__)
#define NAMED_LOG_ERROR(...) manipulation_royalpanda::namedLog(LogType::ERROR, __PRETTY_FUNCTION__, __VA_ARGS__)

namespace manipulation_royalpanda{

enum LogType{
  INFO,
  WARN,
  ERROR
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
  append(os, args...);
  switch (type){
    case INFO:
      ROS_INFO_STREAM(os.str());
    case WARN:
      ROS_WARN_STREAM(os.str());
    case ERROR:
      ROS_ERROR_STREAM(os.str());
    default:
      std::cout << os.str() << std::endl;
  }
}
}