#include "system.hpp"
#include "engine.hpp"
#include "looper.hpp"
#include <iostream>
#include "oblog.hpp"
#include <random>

#include "polytope_examples.hpp"

using namespace OB;

int main( int argc, const char* argv[] )
{
    FILELog::ReportingLevel() = linfo;
    //FILELog::ReportingLevel() = ldebug4;

    System system{};
    system.set_default_gravity(Vector(0.0, -5.0, 0.0));

    system.create_engine("ode", EngineType::Ode);

    Engine& engineOde2 = system.create_engine("odevclip", EngineType::Ode);
    engineOde2.set_use_vclip(true);

    system.create_engine("bullet", EngineType::Bullet);
    Engine& engineBullet2 = system.create_engine("bulletvclip", EngineType::Bullet);
    engineBullet2.set_use_vclip(true);
    // std::cout << "todo creado " << std::endl;

    Real radius(3);
    Real mass(6);

    // std::vector<Point> pointsground = get_prism_points(4, 1, 20);
    // Body& ground = system.createConvex("ground", pointsground, 0);
    // ground.setStatic(true);

    Plane groundPlane{Vector(0, 1, 0), 0};
    Body& box1 = system.create_body("ground");
    box1.set_static(true);
    PlaneShape& box1_shape1 = box1.add_plane_shape(groundPlane);

    //random engine with seed.
    std::default_random_engine eng{ static_cast<long unsigned int>(379) };
    std::uniform_real_distribution<double> urd(0.5, 1);

    // for (int i = 0; i < 5; i++) {
    //     //std::vector<Point> points = get_prism_points(6, 1, 1);
    //     std::vector<Point> points =  get_bunny_points();

    //     Body& convex1 = system.createConvex("convex" + std::to_string(i), points, mass);
    //     Vector origin(1 * urd(eng), 20 * urd(eng), 1 * urd(eng));
    //     convex1.getInitialState().setPosition(origin);
    //     convex1.getInitialState().setLinearVelocity(Vector(urd(eng), urd(eng), urd(eng)));
    //     convex1.getInitialState().setAngularVelocity(Vector(urd(eng), urd(eng), urd(eng)));
    // }

    for (int i = 0; i < 10; i++)
    {
        std::vector<Point> points =  get_rbox_points("1000 s D3"); /// 1000 points in a 3d sphere
        for (auto& p : points) {
            L_(linfo) << " Point to add" << p.transpose();
            p *= 2; // a bit bigger
        }
        Body& convex = system.create_body("rbox" + std::to_string(i));
        ConvexShape convex_shape =convex.add_convex_shape(points);
        Vector origin(5, 5 * i, 1 * urd(eng));
        convex.get_initial_state().set_position(origin);
        convex.get_initial_state().set_linear_velocity(Vector(urd(eng), urd(eng), urd(eng)));
        convex.get_initial_state().set_angular_velocity(Vector(urd(eng), urd(eng), urd(eng)));

        Vector minAabb;
        Vector maxAabb;
        convex_shape.get_convex_polytope()->get_local_aabb(minAabb, maxAabb);
        convex.set_inertia_tensor(get_inertia_tensor_box(maxAabb - minAabb, mass));
        convex.set_mass(mass);

    }

    system.insert_all_bodies_in_all_engines();

    // std::vector<Point> points2 = get_prism_points(3, 1, 1);
    // Body& convex2 = system.createConvex("convex2", points2, mass);
    // OB::AngleAxis(0.5*OB::PI, OB::Vector::UnitZ().normalized());
    // Quaternion initialOrientation{AngleAxis(OB::PI, OB::Vector{1,1,1}.normalized())};
    // convex2.getInitialState().setPosition(Vector(5, 20, 0));
    // convex2.getInitialState().setOrientation(initialOrientation);
    // convex2.getInitialState().setLinearVelocity(Vector(-2, 0, 0));

    Looper looper{system};
    //looper.setPauseOnCollisions(true);

    looper.add_engine("ode", Vector{-40, 0, 0});
    looper.add_engine("odevclip", Vector{-15, 0, 0});

    looper.add_engine("bullet", Vector{15, 0, 0});
    looper.add_engine("bulletvclip", Vector{40, 0, 0});


    looper.loop();

}
