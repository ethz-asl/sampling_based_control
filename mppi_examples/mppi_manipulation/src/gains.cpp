#include <mppi_manipulation/gains.h>

using namespace manipulation;

std::ostream& operator<<(std::ostream& os,
                         const manipulation::PandaRaisimGains& gains) {
  std::cout << "=================================" << std::endl;
  std::cout << "       Panda Raisim Gains" << std::endl;
  std::cout << "=================================" << std::endl;
  std::cout << "Base gains: " << std::endl;
  std::cout << " Kp: " << gains.base_gains.Kp.transpose() << std::endl;
  std::cout << " Kd: " << gains.base_gains.Kd.transpose() << std::endl;
  std::cout << "Arm gains: " << std::endl;
  std::cout << " Kp: " << gains.arm_gains.Kp.transpose() << std::endl;
  std::cout << " Kd: " << gains.arm_gains.Kd.transpose() << std::endl;
  std::cout << "Gripper gains: " << std::endl;
  std::cout << " Kp: " << gains.gripper_gains.Kp.transpose() << std::endl;
  std::cout << " Kd: " << gains.gripper_gains.Kd.transpose() << std::endl;
  std::cout << "=================================" << std::endl;
  return os;
}