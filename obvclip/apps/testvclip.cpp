#include <ostream>

#include "polytope.hpp"
#include "ogrevclip_app.hpp"
#include "polytope_examples.hpp"
#include <random>
#include "vclip.hpp"
#include "oblog.hpp"

using namespace OB;


int main (int argc, char *argv[])
{
    FILELog::ReportingLevel() = ldebug4;

    std::vector<Point> points1 = get_prism_points(4, 1, 1);
    std::vector<Point> points2 = get_pyramid_points(3, 1, 1);

    // std::vector<Point> points = OB::get_bunny_points();
    // for (Point& p: points)
    //     p *= 5;

    // std::vector<Point> points = OB::get_rbox_points("1000 s D3"); /// 1000 points in a 3d sphere
    //OB::Real mult = 1;
    // for (Point& p: points)
    //     p *= (2*mult);

    // std::vector<Point> points =  get_bunny_points();
    // for (Point& p : points) { p = p*2; }
    // std::vector<Point> points2 = points;

    std::shared_ptr<OB::ConvexPolytope> cv1 = std::make_shared<OB::ConvexPolytope>();
    cv1->generate_from_vertices(points1);
    assert(cv1->euler_number_correct());

    std::shared_ptr<OB::ConvexPolytope> cv2 = std::make_shared<OB::ConvexPolytope>();
    cv2->generate_from_vertices(points2);
    assert(cv2->euler_number_correct());


    std::shared_ptr<OB::ConvexPolytope> poly =
        std::shared_ptr<OB::ConvexPolytope>(cv1);


    std::shared_ptr<OB::ConvexPolytope> poly2 =
        std::shared_ptr<OB::ConvexPolytope>(cv2);


    OB::WorldConvexPolytope wcp0{"Obj0", poly};
    OB::Transform t1;
    OB::Vector translation1{0,10,0};
    OB::Matrix3 rotmatrix1;
    rotmatrix1 << 0.68147, -0.216155, -0.699197,
        0.581945,  -0.41928,   0.69681,
        -0.443778, -0.881749, -0.159936;
    t1 = OB::Translation(translation1) * rotmatrix1;

    //t1 = OB::Translation(translation1) * OB::Quaternion(0.43530320304438430146, 0.72029361595389407302, + 0.4991926259171006186, -0.20614303399667252559);
    t1 = OB::Translation(translation1) * OB::AngleAxis(0.0*OB::PI, OB::Vector::UnitZ());

    wcp0.set_pose(t1);

    OB::WorldConvexPolytope wcp1{"Obj1", poly2};
    OB::Transform t2;
    OB::Vector translation2{0, 9.5,0};
    OB::Matrix3 rotmatrix2;
    rotmatrix2 << -0.440567, -0.608536,  0.659988,
        -0.212282, -0.643712, -0.735235,
        0.87226, -0.464024,  0.154417;
    t2 = OB::Translation(translation2) * rotmatrix2;

    t2 = OB::Translation(translation2) * OB::Quaternion(-0.063033757333528478428, 0.57278688550463519036, + 0.62438380575412422147, 0.52733935216548533109);
    t2 = OB::Translation(translation2) * OB::AngleAxis(0.0*OB::PI, OB::Vector::UnitZ());
    wcp1.set_pose(t2);

    //TODO COLOCAR COMO TESTBULLET
    OB::VClipWitness witness = OB::VClipWitness(OB::Feature{5, OB::FeatureType::face},
                                                OB::Feature{2, OB::FeatureType::vertex});


    OB::OgreVClipApp app;
    app.initApp();
    app.addWorldConvexPolytope(wcp0);
    app.addWorldConvexPolytope(wcp1);
    app.setVClipPair(wcp0, wcp1, witness);
    app.getRoot()->startRendering();
    app.closeApp();


}
