#pragma once

#include <chrono>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <Eigen/Core>
#include <variant>
#include <type_traits>
#include <map>
#include <memory>
#include <ctime>

namespace logger{

struct DataStreamer{
  DataStreamer() = default;
  explicit DataStreamer(std::shared_ptr<std::ofstream>& file): file_(file){};

  template<typename T>
  void stream_array(const T& v){
    *file_ << "[";
    for (int i = 0; i < (int)v.size() - 1; i++) *file_ << v[i] << ",";
    *file_ << v[v.size()-1];
    *file_ << "]";
  }

  template<typename T>
  void operator()(const T& v){
    *file_ << v;
  }

 private:
  std::shared_ptr<std::ofstream>file_;
};

template<> void DataStreamer::operator()<Eigen::VectorXd>(const Eigen::VectorXd& v){
  stream_array(v);
}

template<> void DataStreamer::operator()<std::vector<double>>(const std::vector<double>& v){
  stream_array(v);
}

template<> void DataStreamer::operator()<std::vector<Eigen::VectorXd>>(const std::vector<Eigen::VectorXd>& v){
  *file_ << "[";

  for (size_t i = 0; i < v.size() - 1; i++) {
    stream_array(v[i]);
    *file_ << ",";
  }
  stream_array(v.back());
  *file_ << "]";
}

class DataLogger {
 private:

 public:
  DataLogger() = delete;
  ~DataLogger() { file_->close(); }

  DataLogger(DataLogger const&) = delete;
  DataLogger& operator=(DataLogger const&) = delete;

  explicit DataLogger(const std::string& root_folder, const std::string& experiment_id){

    file_path_ = root_folder + "/" + experiment_id  + "-"  + getCurrentDateTime("now") + "" + ".csv";
    file_ = std::make_shared<std::ofstream>(file_path_);
    if (!file_->is_open()) {
      throw std::runtime_error("Unable to open log file");
    }
    streamer_ = DataStreamer(file_);
    initialized_ = false;
  }

  void add_field(const std::string& field){
    header_.push_back(field);
  }

  void write_header(){
    for (size_t i = 0; i < header_.size() - 1; i++) *file_ << header_[i] << ";";
    *file_ << header_.back() << std::endl;
  }

  template<typename T>
  void set(const std::string& field_name, const T& value){
    if (!initialized_){
      write_header();
      initialized_ = true;
    }
    if (std::find(header_.begin(), header_.end(), field_name) == header_.end()){
      std::cout << "Could not find field name [" << field_name << "]. Not writing." << std::endl;
      return;
    }
    collection_[field_name] = value;
  }

  void write_line(){
    for (size_t i = 0; i < header_.size() - 1; i++) {
      std::visit(streamer_, collection_[header_[i]]);
      *file_ << ";";
    };
    std::visit(streamer_, collection_[header_.back()]);
    *file_ << std::endl;
    counter++;
    if (counter == flush_period_){
      counter = 0;
      file_->flush();
    }
  }

  static std::string getCurrentDateTime(const std::string& s ){
    time_t now = time(0);
    struct tm  tstruct;
    char  buf[80];
    tstruct = *localtime(&now);
    if(s=="now")
      strftime(buf, sizeof(buf), "%Y-%m-%d-%H-%M", &tstruct);
    else if(s=="date")
      strftime(buf, sizeof(buf), "%Y-%m-%d", &tstruct);
    return std::string(buf);
  };


 private:
  bool initialized_;
  size_t counter = 0;
  size_t flush_period_ = 100;
  DataStreamer streamer_;
  std::string file_path_;
  std::shared_ptr<std::ofstream> file_;
  std::vector<std::string> header_;
  std::map<std::string, std::variant<int, double, float, Eigen::VectorXd, std::vector<double>, std::vector<Eigen::VectorXd>>> collection_;
};
}