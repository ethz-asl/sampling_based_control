#include <iostream>

#define LOG_INFO(...) std::cout << __VA_ARGS__ << std::endl
#define LOG_WARN(...) \
  std::cout << "\033[1;93m" << __VA_ARGS__ << "\033[0m" << std::endl
#define LOG_ERROR(...) \
  std::cout << "\033[1;31m" << __VA_ARGS__ << "\033[0m" << std::endl
