/*!
 * @file     savgol_filter.cpp
 * @author   Giuseppe Rizzi
 * @date     27.07.2020
 * @version  1.0
 * @brief    description
 */

#include <gram_savitzky_golay/gram_savitzky_golay.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Float32MultiArray.h>

rosbag::Bag bag;
bag.open("test.bag", rosbag::bagmode::Read);

std::vector<std::string> topics;
topics.push_back(std::string("chatter"));
topics.push_back(std::string("numbers"));

rosbag::View view(bag, rosbag::TopicQuery(topics));

for (rosbag::MessageInstance const m : rosbag::View(bag)) {
  std_msgs::String::ConstPtr s = m.instantiate<std_msgs::String>();
  if (s != NULL) std::cout << s->data << std::endl;

  std_msgs::Int32::ConstPtr i = m.instantiate<std_msgs::Int32>();
  if (i != NULL) std::cout << i->data << std::endl;
}

bag.close();

// Window size is 2*m+1
const size_t m = 3;
// Polynomial Order
const size_t n = 2;
// Initial Point Smoothing (ie evaluate polynomial at first point in the window)
// Points are defined in range [-m;m]
const size_t t = m;
// Derivation order? 0: no derivation, 1: first derivative, 2: second
// derivative...
const int d = 0;

// Real-time filter (filtering at latest data point)
gram_sg::SavitzkyGolayFilter filter(m, t, n, d);
// Filter some data
std::vector<double> data = {.1, .7, .9, .7, .8, .5, -.3};
double result = filter.filter(data);

// Real-time derivative filter (filtering at latest data point)
// Use first order derivative
// NOTE that the derivation timestep is assumed to be 1. If this is not the
// case, divide the filter result by the timestep to obtain the correctly scaled
// derivative See Issue #1
// d=1;
// gram_sg::SavitzkyGolayFilter first_derivative_filter(m, t, n, d);
//// Filter some data
// std::vector<double> values = {.1, .2, .3, .4, .5, .6, .7};
//// Should be =.1
// double derivative_result = first_derivative_filter.filter(values);