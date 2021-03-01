#include "system.hpp"
#include "engine.hpp"
#include "looper.hpp"
#include <iostream>
#include "oblog.hpp"
#include <random>
#include "ode_engine.hpp"
#include "bullet_engine.hpp"

#include "polytope_examples.hpp"

using namespace OB;

int main( int argc, const char* argv[] )
{
    FILELog::ReportingLevel() = linfo;

    System system{};
    system.set_default_gravity(Vector(0.0, -9.8, 0.0));

    Plane ground_plane{Vector(0, 1, 0).normalized(), 0};
    Body& ground =  system.create_body("ground");
    ground.get_initial_state().set_position(Vector(0,0,0));
    ground.set_static(true);
    PlaneShape& ground_shape = ground.add_plane_shape(ground_plane);

    Vector origin(-10);
    Vector extents(20, 4, 20);
    Real radius{3};
    Real mass{6};
    Body& box1 = system.create_body("boxground");
    BoxShape& box_shape1 = box1.add_box_shape(extents);
    box1.get_initial_state().set_position(origin);
    box1.set_static(true);


    Engine& ode = system.create_engine("ode", EngineType::Ode);
    OdeEngine& odeengine = dynamic_cast<OdeEngine&>(ode);
    odeengine.set_friction(0.0);
    odeengine.set_restitution(1.0);

    Engine& bullet = system.create_engine("bullet", EngineType::Bullet);
    BulletEngine& bulletengine = dynamic_cast<BulletEngine&>(bullet);
    bulletengine.set_friction(0.0);
    bulletengine.set_restitution(1.0);


    Real sphere_radius(3);
    Real sphere_mass(1);
    Body& sphere = system.create_body("sphere");
    SphereShape& sphere_shape = sphere.add_sphere_shape(sphere_radius);
    sphere.set_mass(sphere_mass);
    sphere.set_inertia_tensor(get_inertia_tensor_sphere(sphere_radius, sphere_mass));
    //L_(linfo) << "Inertia sphere "  <<  get_inertia_tensor_sphere(radius, mass);
    Vector position_sphere{0,20,0};
    sphere.get_initial_state().set_position(position_sphere);

    Body& sphere2 = system.create_body("sphere2");
    sphere2.add_sphere_shape(sphere_radius);
    sphere2.set_mass(sphere_mass);
    sphere2.set_inertia_tensor(get_inertia_tensor_sphere(sphere_radius, sphere_mass));
    //L_(linfo) << "Inertia sphere "  <<  get_inertia_tensor_sphere(radius, mass);
    Vector position_sphere2{20,20,0};
    sphere2.get_initial_state().set_position(position_sphere2);


    system.insert_all_bodies_in_all_engines();
    Looper looper{system};
    looper.add_engine("ode", Vector{0, 0, 0});
    looper.add_engine("bullet", Vector{100, 0, 0});

    looper.loop();

}
