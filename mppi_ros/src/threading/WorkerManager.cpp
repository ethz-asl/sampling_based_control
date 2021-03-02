/*!
 * @file	WorkerManager.cpp
 * @author	Philipp Leemann
 * @date	July, 2016
 */

// modified by: Giuseppe Rizzi

#include "mppi_ros/threading/WorkerManager.hpp"
#include "mppi_ros/threading/log_messages.hpp"

namespace mppi::threading {

WorkerManager::WorkerManager() : workers_(), mutexWorkers_() {}

WorkerManager::~WorkerManager() { cancelWorkers(); }

bool WorkerManager::addWorker(const WorkerOptions& options,
                              const bool autostart) {
  std::lock_guard<std::mutex> lock(mutexWorkers_);
  auto insertedElement = workers_.emplace(options.name_, Worker(options));
  if (!insertedElement.second) {
    LOG_ERROR("Failed to create worker [" << options.name_ << "]");
    return false;
  }
  if (autostart) {
    return insertedElement.first->second.start();
  }
  return true;
}

// bool WorkerManager::addWorker(Worker&& worker) {
//    std::lock_guard<std::mutex> lock(mutexWorkers_);
//    auto insertedElement = workers_.emplace( worker.getName(),
//    std::move(worker) ); if(!insertedElement.second) {
//        MELO_ERROR("Failed to move worker [%s]", worker.getName().c_str());
//        return false;
//    }
//    return true;
//}

void WorkerManager::startWorker(const std::string& name, const int priority) {
  std::lock_guard<std::mutex> lock(mutexWorkers_);
  auto worker = workers_.find(name);
  if (worker == workers_.end()) {
    LOG_ERROR("Cannot start worker [" << name << "], worker not found");
    return;
  }
  worker->second.start(priority);
}

void WorkerManager::startWorkers() {
  std::lock_guard<std::mutex> lock(mutexWorkers_);
  for (auto& worker : workers_) {
    worker.second.start();
  }
}

void WorkerManager::stopWorker(const std::string& name, const bool wait) {
  std::lock_guard<std::mutex> lock(mutexWorkers_);
  auto worker = workers_.find(name);
  if (worker == workers_.end()) {
    LOG_ERROR("Cannot stop worker [" << name << "], worker not found");
    return;
  }
  worker->second.stop(wait);
}

void WorkerManager::stopWorkers(const bool wait) {
  std::lock_guard<std::mutex> lock(mutexWorkers_);
  for (auto& worker : workers_) {
    worker.second.stop(wait);
  }
}

bool WorkerManager::hasWorker(const std::string& name) {
  std::lock_guard<std::mutex> lock(mutexWorkers_);
  return (workers_.find(name) != workers_.end());
}

void WorkerManager::cancelWorker(const std::string& name, const bool wait) {
  std::lock_guard<std::mutex> lock(mutexWorkers_);
  auto worker = workers_.find(name);
  if (worker == workers_.end()) {
    LOG_ERROR("Cannot stop worker [" << name << "], worker not found");
    return;
  }
  worker->second.stop(wait);
  workers_.erase(worker);
}

void WorkerManager::cancelWorkers(const bool wait) {
  std::lock_guard<std::mutex> lock(mutexWorkers_);

  // signal all workers to stop
  for (auto& worker : workers_) {
    worker.second.stop(wait);
  }

  // call destructors of all workers, which will join the underlying thread
  workers_.clear();
}

void WorkerManager::setWorkerTimestep(const std::string& name,
                                      const double timeStep) {
  std::lock_guard<std::mutex> lock(mutexWorkers_);
  auto worker = workers_.find(name);
  if (worker == workers_.end()) {
    LOG_ERROR("Cannot change timestep of worker [" << name
                                                   << "], worker not found");
    return;
  }
  worker->second.setTimestep(timeStep);
}

void WorkerManager::cleanDestructibleWorkers() {
  std::lock_guard<std::mutex> lock(mutexWorkers_);
  for (auto it = workers_.begin(); it != workers_.end();) {
    if (it->second.isDestructible()) {
      it = workers_.erase(it);
    } else {
      ++it;
    }
  }
}

}  // namespace mppi::threading
