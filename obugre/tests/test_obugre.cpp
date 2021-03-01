#include "gtest/gtest.h"


#include "system.hpp"

using namespace OB;

TEST(TESTObugre, basicTest) {
  OB::System system{};
  system.createEngine("ode", EngineType::Ode);

  Real mass = 1.0;
  system.createBox("box1", Vector{1,1,1}, mass);

  system.step();

}
