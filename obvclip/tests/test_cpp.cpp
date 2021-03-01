#include "gtest/gtest.h"
#include <iostream>

#include "polytope.hpp"

TEST(TESTCpp, basicTest) {
  OB::FeatureType ft = OB::FeatureType::vertex;
  std::cout << "SIZE OF enum class FeatureType: " << sizeof(OB::FeatureType) << std::endl;
  std::cout << "SIZE OF long: " << sizeof(long) << std::endl;
  std::cout << "SIZE OF size_t: " << sizeof(size_t) << std::endl;
  std::cout << "SIZE OF int: " << sizeof(int) << std::endl;
  std::cout << "SIZE OF char: " << sizeof(char) << std::endl;
  std::cout << "SIZE OF feature: " << sizeof(OB::Feature) << std::endl;
  std::cout << "SIZE OF char*: " << sizeof(char*) << std::endl;
}
