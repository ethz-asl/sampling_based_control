//
// Created by giuseppe on 29.07.21.
//

#include <ros/ros.h>
#include "mppi_manipulation/reference_scheduler.h"

std::ostream& operator<<(std::ostream& os, const mppi::reference_trajectory_t& ref){
  if (ref.rr.empty()){
    os << "Trajectory is empty!" << std::endl;
  }
  else{
    os << "r=[" << ref.rr[0].transpose() << "]"<< std::endl;
    os << "t=" << ref.tt[0] << std::endl;
  }
  return os;
}

int main(int argc, char** argv){
  std::vector<std::string> program_args{};
  ::ros::removeROSArgs(argc, argv, program_args);
  if (program_args.size() <= 1) {
    throw std::runtime_error("No reference file specified. Aborting.");
  }
  std::string reference_file = std::string(program_args[1]);

  manipulation::ReferenceScheduler scheduler;
  scheduler.parse_from_file(reference_file);

  mppi::reference_trajectory_t ref;
  std::vector<double> times{-1, 1, 1.1, 2, 2.9, 3, 3.3};
  for (const auto& time : times){
    scheduler.set_reference(time, ref);
    std::cout << "Queried reference at time " << time << " is:" << std::endl;
    std::cout << ref;
  }

  return 0;
}
