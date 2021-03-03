/**********************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2016, Philipp Leemann
 * All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Robotic Systems Lab nor ETH Zurich
 *     nor the names of its contributors may be used to endorse or
 *     promote products derived from this software without specific
 *     prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/*!
 * @file	Worker.hpp
 * @author	Philipp Leemann
 * @date	July, 2016
 */

// modified by: Giuseppe Rizzi

#include <limits>

#include <pthread.h>
#include <cstring>  // strerror(..)
#include <ctime>

#include "mppi_ros/threading/Worker.hpp"
#include "mppi_ros/threading/log_messages.hpp"

namespace mppi::threading {

Worker::Worker(const std::string& name, const double timestep,
               const WorkerCallback& callback)
    : Worker(WorkerOptions(name, timestep, callback)) {}

Worker::Worker(const std::string& name, const double timestep,
               const WorkerCallback& callback,
               const WorkerCallbackFailureReaction& callbackFailureReaction)
    : Worker(WorkerOptions(name, timestep, callback, callbackFailureReaction)) {
}

Worker::Worker(const WorkerOptions& options)
    : options_(options),
      running_(false),
      done_(false),
      thread_(),
      rate_(options)  // NOLINT(cppcoreguidelines-slicing)
{}

Worker::Worker(Worker&& other)
    : options_(std::move(other.options_)),
      running_(other.running_.load()),
      done_(other.done_.load()),
      thread_(std::move(other.thread_)),
      rate_(std::move(other.rate_)) {}

Worker::~Worker() { stop(true); }

bool Worker::start(const int priority) {
  if (running_) {
    LOG_ERROR("Worker [" << options_.name_
                         << "] cannot be started, already/still running.");
    done_ = true;
    return false;
  }
  if (options_.timeStep_ < 0.0) {
    LOG_ERROR("Worker [" << options_.name_
                         << "] cannot be started, invalid timestep: "
                         << options_.timeStep_.load());
    done_ = true;
    return false;
  }

  running_ = true;
  done_ = false;

  thread_ = std::thread(&Worker::run, this);

  sched_param sched{};
  sched.sched_priority = 0;
  if (priority != 0) {
    sched.sched_priority = priority;
  } else if (options_.defaultPriority_ != 0) {
    sched.sched_priority = options_.defaultPriority_;
  }

  if (sched.sched_priority != 0) {
    if (pthread_setschedparam(thread_.native_handle(), SCHED_FIFO, &sched) !=
        0) {
      LOG_WARN("Failed to set thread priority for worker ["
               << options_.name_ << "]: " << strerror(errno));
    }
  }

  LOG_INFO("Worker [" << options_.name_ << "] started");
  return true;
}

void Worker::stop(const bool wait) {
  running_ = false;
  if (wait && thread_.joinable()) {
    thread_.join();
  }
}

void Worker::setTimestep(const double timeStep) {
  if (timeStep <= 0.0) {
    LOG_ERROR("Cannot change timestep of Worker ["
              << options_.name_ << "] to " << timeStep << ", invalid value.");
    return;
  }
  options_.timeStep_ = timeStep;
  if (!std::isinf(timeStep)) {
    // We will use the rate, so we set its parameters.
    rate_.getOptions().timeStep_ = timeStep;
  }
}

void Worker::setEnforceRate(const bool enforceRate) {
  options_.enforceRate_ = enforceRate;
  rate_.getOptions().enforceRate_ = enforceRate;
}

void Worker::run() {
  if (std::isinf(options_.timeStep_)) {
    // Run the callback once.
    static timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    if (!options_.callback_(WorkerEvent(options_.timeStep_, now))) {
      LOG_WARN(
          "Worker [" << options_.name_
                     << "] callback returned false. Calling failure reaction.");
      options_.callbackFailureReaction_();
    }
  } else {
    // Reset the rate step time.
    rate_.reset();

    // Run the callback repeatedly.
    do {
      if (!options_.callback_(
              WorkerEvent(options_.timeStep_, rate_.getSleepEndTime()))) {
        LOG_WARN("Worker ["
                 << options_.name_
                 << "] callback returned false. Calling failure reaction.");
        options_.callbackFailureReaction_();
      }

      rate_.sleep();

    } while (running_);
  }

  LOG_INFO("Worker [" << options_.name_ << "] terminated.");
  done_ = true;
}

}  // namespace mppi::threading
