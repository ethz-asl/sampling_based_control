//
// Created by etienne on 03.11.20.
//

#pragma once

#include <chrono>
#include <fstream>
#include <iostream>
#include <stdexcept>

class DataLogger {
 public:
  // DataLogger cannot exist without file.
  DataLogger() = delete;

  // Cleanup.
  ~DataLogger() { log_file.close(); }

  // Disable copy constructor  since std::ofstream is not copyable.
  DataLogger(DataLogger const&) = delete;

  // Constructor
  explicit DataLogger(std::string f_path) : log_file(f_path) {
    if (!log_file.is_open()) {
      throw std::runtime_error("Unable to open log file");
    }
    f_path_ = f_path;
    //		log_file << std::ios_base::unitbuf;
    std::cout << "Logger linked to file in constructor: " << f_path_
              << std::endl;
  }

  // Disable copy.
  DataLogger& operator=(DataLogger const&) = delete;

  // Write a single value into log file stream.
  template <typename T>
  void write(T const& v) {
    log_file << v << ",";
  }

  void write_endl() { log_file << std::endl; }

  // Write multiple values.
  template <typename Arg, typename... Args>
  void write(Arg const& arg, Args const&... args) {
    write(arg);
    // here we recursively pass the rest values to the function.
    write(args...);
  }

 private:
  std::ofstream log_file;
  std::string f_path_;
};
