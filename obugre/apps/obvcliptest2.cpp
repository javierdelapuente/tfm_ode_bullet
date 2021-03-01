#include <iostream>
#include <random>

#include <oblog.hpp>
#include <polytope_examples.hpp>
#include "system.hpp"
#include "engine.hpp"
#include "looper.hpp"
#include "ode_engine.hpp"
#include "bullet_engine.hpp"

using namespace OB;



int main( int argc, const char* argv[] )
{
    FILELog::ReportingLevel() = linfo;

    System system{};
    system.set_default_gravity(Vector(0, -10, 0));

    EngineLogger engine_logger{"obvcliptest2.csv"};
    engine_logger.set_logger_only_engine(true);

    Real restitution{0.1};
    Real friction{6.3};

    Engine& bulletvclip = system.create_engine("bulletvclip", EngineType::Bullet);
    bulletvclip.set_use_vclip(true);
    BulletEngine& bulletvclipengine = dynamic_cast<BulletEngine&>(bulletvclip);
    bulletvclipengine.set_friction(friction);
    bulletvclipengine.set_restitution(restitution);

    Engine& bullet = system.create_engine("bullet", EngineType::Bullet);
    bullet.add_listener(engine_logger);
    std::cout << "todo creado " << std::endl;
    BulletEngine& bulletengine = dynamic_cast<BulletEngine&>(bullet);
    bulletengine.set_friction(friction);
    bulletengine.set_restitution(restitution);

    Vector groundextents{10,1,10};
    Body& ground =  system.create_body("ground");
    ground.get_initial_state().set_position(Vector(0,-10,10));
    ground.set_static(true);
    std::vector<Point> pointsground =  get_prism_points(4, 1, 10);
    ground.add_convex_shape(pointsground);

    Vector extents{2,5,6};
    Real mass{5};
    Vector position1{5,3,5};
    Body& body1 = system.create_body("body1");
    std::vector<Point> points1 =  get_prism_points(5, 1, 1);
    for (Point& p : points1) { p = p*2; }
    body1.add_convex_shape(points1);
    body1.set_mass(mass);
    body1.set_inertia_tensor(get_inertia_tensor_box(extents, mass));
    body1.get_initial_state().set_position(position1);
    body1.get_initial_state().set_linear_velocity(Vector(2.0, -10.0, 0.0));
    body1.get_initial_state().set_angular_velocity(Vector(2.0, 0.1, 0.0));

    Vector position2{5,10,5};
    Body& body2 = system.create_body("body2");
    std::vector<Point> points2 =  get_prism_points(5, 1, 1);
    for (Point& p : points2) { p = p*2; }
    body2.add_convex_shape(points2);
    body2.set_mass(mass);
    body2.set_inertia_tensor(get_inertia_tensor_box(extents, mass));
    body2.get_initial_state().set_position(position2);
    body2.get_initial_state().set_linear_velocity(Vector(2.0, -10.0, 0.0));
    body2.get_initial_state().set_angular_velocity(Vector(2.0, 0.1, 0.0));


    system.insert_all_bodies_in_all_engines();

    Looper looper{system, false};
    //looper.add_engine("odevclip", Vector{0, 0, 0});
    looper.add_engine("bulletvclip", Vector{0, 0, 20});
    looper.add_engine("bullet", Vector{0, 0, -20});
    //looper.add_engine("ode", Vector{-50, 0, -50});



    looper.set_simulation_speed(1.0);
    looper.loop();


}
