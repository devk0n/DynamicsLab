#include <iostream>
#include <Eigen/Dense>

int main() {

  Eigen::MatrixXd A = Eigen::MatrixXd::Random(3, 3);

  std::cout << A << std::endl;

  return 0;
}