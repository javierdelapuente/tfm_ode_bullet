#include <ostream>

#include "polytope.hpp"
#include "ogrevclip_app.hpp"
#include "polytope_examples.hpp"

using OB::Point;

std::shared_ptr<OB::ConvexPolytope> createCubePolytope () {
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
    return poly; //check move constructor!
}



int main (int argc, char *argv[])
{
    std::shared_ptr<OB::ConvexPolytope> poly{new OB::ConvexPolytope(OB::createBunnyPolytope())};
    std::shared_ptr<OB::ConvexPolytope> poly2{new OB::ConvexPolytope(OB::createBunnyPolytope())};

    OB::Transform t;

    OB::WorldConvexPolytope wcp0{"Obj0",
                                       poly};
    // t = OB::Translation(10, 0 ,0);
    // wcp0.set_pose(t);

    OB::WorldConvexPolytope wcp1{"Obj1",
                                       poly2};

    t = OB::Translation(5, 0 ,0);
    t = OB::Translation(0, 5 ,0) * OB::AngleAxis(0.5*OB::PI, OB::Vector::UnitZ());
    wcp1.set_pose(t);



    OB::OgreVClipApp app;
    app.initApp();
    app.addWorldConvexPolytope(wcp0);
    app.addWorldConvexPolytope(wcp1);
    app.getRoot()->startRendering();
    app.closeApp();


}
