#include "gtest/gtest.h"

#include <iostream>
#include <Eigen/Dense>
#include "polytope.hpp"
#include "oblog.hpp"

using Eigen::MatrixXd;

TEST(TESTEigen, basicTest) {
  MatrixXd m(2,2);
  m(0,0) = 3;
  m(1,0) = 2.5;
  m(0,1) = -1;
  m(1,1) = m(1,0) + m(0,1);
  std::cout << m << std::endl;


  OB::Transform t;
  std::cout << "Size of isometry: " << sizeof(t) << std::endl;;
  OB::Matrix3 rot = t.rotation();
  OB::Vector trans = t.translation();
  std::cout << " rotation: " << rot << std::endl;
  std::cout << " translation: " << trans << std::endl;
  //spdlog::info(" translation: {}", trans);
  //spdlog::info(" rotation: {}", v);

  std::cout.precision(17);

  OB::Real valor = 7;
  OB::Real otro = 7;
  std::cout << "valor: "<< valor << " isApprox " << otro<< " ->  " << Eigen::internal::isApprox(valor, otro) << std::endl;
  otro = 7.00000000001;
  std::cout << "valor: "<< valor << " isApprox " << otro<< " ->  " << Eigen::internal::isApprox(valor, otro) << std::endl;
  otro = 7.000000000001;
  std::cout << "valor: "<< valor << " isApprox " << otro<< " ->  " << Eigen::internal::isApprox(valor, otro) << std::endl;
  otro = 6.99999999999;
  std::cout << "valor: "<< valor << " isApprox " << otro<< " ->  " << Eigen::internal::isApprox(valor, otro) << std::endl;
  otro = 6.999999999999;
  std::cout << "valor: "<< valor << " isApprox " << otro<< " ->  " << Eigen::internal::isApprox(valor, otro) << std::endl;

  // for is_zero
  valor = 0.0001;
  std::cout << "valor: " << valor << " is_zero -> " << Eigen::internal::isMuchSmallerThan(valor, static_cast<OB::Real>(1.0)) << std::endl;;
  valor = 1;
  std::cout << "valor: " << valor << " is_zero -> " << Eigen::internal::isMuchSmallerThan(valor, static_cast<OB::Real>(1.0)) << std::endl;;
  valor = 0;
  std::cout << "valor: " << valor << " is_zero -> " << Eigen::internal::isMuchSmallerThan(valor, static_cast<OB::Real>(1.0)) << std::endl;;
  valor = 0.00000000001;
  std::cout << "valor: " << valor << " is_zero -> " << Eigen::internal::isMuchSmallerThan(valor, static_cast<OB::Real>(1.0)) << std::endl;;
  valor = 0.000000000001;
  std::cout << "valor: " << valor << " is_zero -> " << Eigen::internal::isMuchSmallerThan(valor, static_cast<OB::Real>(1.0)) << std::endl;;


  valor = -0.00000000001;
  std::cout << "valor: " << valor << " is_zero -> " << Eigen::internal::isMuchSmallerThan(valor, static_cast<OB::Real>(1.0)) << std::endl;;
  valor = -0.000000000001;
  std::cout << "valor: " << valor << " is_zero -> " << Eigen::internal::isMuchSmallerThan(valor, static_cast<OB::Real>(1.0)) << std::endl;;

}


TEST(TESTEigen, matrixInizialization) {
  Eigen::Vector3d v1;
  v1 << 1,2,3;
  Eigen::Vector3d v2;
  v2 << 4,5,6;
  Eigen::Vector3d v3;
  v3 << 7,8,9;

  Eigen::Matrix3d matrix;
  matrix << v1, v2, v3;
  std::cout << "Matrix " << matrix << std::endl;
}
