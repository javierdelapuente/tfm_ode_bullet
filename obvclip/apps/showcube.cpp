#include <ostream>

#include "polytope.hpp"
#include "ogrevclip_app.hpp"

using OB::Point;

int main (int argc, char *argv[])
{
  std::vector<OB::Point> points;
  points.push_back(Point(-1, -1, -1));
  points.push_back(Point(-1, -1, 1));
  points.push_back(Point(-1, 1, -1));
  points.push_back(Point(-1, 1, 1));
  points.push_back(Point(1, -1, -1));
  points.push_back(Point(1, -1, 1));
  points.push_back(Point(1, 1, -1));
  points.push_back(Point(1, 1, 1));

  std::shared_ptr<OB::ConvexPolytope> poly =
    std::make_shared<OB::ConvexPolytope>();

  poly->generate_from_vertices(points);

  OB::Transform t;
  OB::WorldConvexPolytope wcp0{"0,0,0",
                               poly};


  OB::WorldConvexPolytope wcp1{"5,0,0",
                               poly};
  t = OB::Translation(5, 0 ,0);
  wcp1.set_pose(t);


  OB::WorldConvexPolytope wcp2{"0,5,0 pi/4ejez",
                               poly};
  t =  OB::Translation(0, 5 ,0) * OB::AngleAxis(0.25*OB::PI, OB::Vector::UnitZ());
  wcp2.set_pose(t);


  OB::OgreVClipApp app;
  app.initApp();
  app.addWorldConvexPolytope(wcp0);
  app.addWorldConvexPolytope(wcp1);
  app.addWorldConvexPolytope(wcp2);
  app.getRoot()->startRendering();
  app.closeApp();

}
