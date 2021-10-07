//
// Created by giuseppe on 27.07.21.
//

#include <mppi/utils/data_logger.h>

using namespace logger;
int main(){
  std::string file_path = __FILE__;
  std::string dir_path = file_path.substr(0, file_path.rfind("/"));
  DataLogger logger(dir_path, "test");
  logger.add_field("int");
  logger.add_field("double");
  logger.add_field("eigen_vector");
  logger.add_field("vector_eigen_vectors");

  for (int i=0; i<1000; i++){
    logger.set("int", 1);
    logger.set("double", 0.1);
    logger.set("eigen_vector", Eigen::VectorXd::Random(5));
    logger.set("vector_eigen_vectors", std::vector<Eigen::VectorXd>(3, Eigen::VectorXd::Random(5)));

    logger.write_line();
  }
  return 0;
}