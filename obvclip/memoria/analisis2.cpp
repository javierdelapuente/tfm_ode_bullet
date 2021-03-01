#include <ostream>
#include <fstream>
#include "polytope.hpp"
#include "polytope_examples.hpp"
#include "ogrevclip_app.hpp"


using namespace OB;



int main (int argc, char *argv[])
{
  std::shared_ptr<OB::ConvexPolytope> tetrahedron = std::make_shared<OB::ConvexPolytope>(create_tetrahedron_polytope(2.0));
  std::shared_ptr<OB::ConvexPolytope> cube = std::make_shared<OB::ConvexPolytope>(create_cube_polytope());
  std::shared_ptr<OB::ConvexPolytope> pyramid4 = std::make_shared<OB::ConvexPolytope>(create_pyramid_polytope(4, 2, 1));
  std::shared_ptr<OB::ConvexPolytope> prism3 = std::make_shared<OB::ConvexPolytope>(create_prism_polytope(3, 2));
  std::shared_ptr<OB::ConvexPolytope> prism6 = std::make_shared<OB::ConvexPolytope>(create_prism_polytope(6, 2.0));
  std::shared_ptr<OB::ConvexPolytope> prism20 = std::make_shared<OB::ConvexPolytope>(create_prism_polytope(20, 1.0, 2.0));

  {
    std::ofstream ofs{"prism20.txt"};
    prism20->write(ofs);
  }
  // {
  //   std::ifstream ifs{"tetrahedron.txt"};
  //   tetrahedron = std::make_shared<OB::ConvexPolytope>(ConvexPolytope::create(ifs));
  // }
  // {
  //   std::ifstream ifs{"pyramid5.txt"};
  //   pyramid5 = std::make_shared<OB::ConvexPolytope>(ConvexPolytope::create(ifs));
  // }

  OB::Transform t = OB::Transform::Identity();

  OB::WorldConvexPolytope wcp0{"pyramid4", pyramid4};
  t = OB::Translation(0, 0 ,0) * OB::AngleAxis(OB::PI * 0.1, OB::Vector::UnitX().normalized());
  wcp0.set_pose(t);

  OB::WorldConvexPolytope wcp1{"cube", cube};

  //t = OB::Translation(5, 0 ,0);
  //t = OB::Translation(0, 5 ,0) * OB::AngleAxis(0.5*OB::PI, OB::Vector::UnitZ().normalized());
  t = OB::Translation(0, 2.5 ,0) * OB::AngleAxis(OB::PI, OB::Vector::UnitZ().normalized());
  t *= OB::AngleAxis(OB::PI * 0.5, OB::Vector::UnitY().normalized());
  wcp1.set_pose(t);



  OB::OgreVClipApp app;
  app.initApp();
  app.addWorldConvexPolytope(wcp0);
  app.addWorldConvexPolytope(wcp1);
  app.getRoot()->startRendering();
  app.closeApp();


}
