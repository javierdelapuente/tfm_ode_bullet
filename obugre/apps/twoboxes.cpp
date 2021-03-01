#include <iostream>
#include <oblog.hpp>

#include "system.hpp"
#include "engine.hpp"
#include "looper.hpp"

using namespace OB;


int main( int argc, const char* argv[] )
{
    FILELog::ReportingLevel() = linfo;

    System system{};

    system.set_default_gravity(Vector(0, -9.8, 0));

    Engine &ode_engine = system.create_engine("ode", EngineType::Ode);
    std::cout << "ode creado " << std::endl;

    Engine &bullet_engine = system.create_engine("bullet", EngineType::Bullet);
    std::cout << "bullet creado " << std::endl;


    Vector origin(-5, 20, 0);
    Vector extents(2, 4, 6);
    Real radius{3};
    Real mass{6};
    Body& box1 = system.create_body("box1");
    BoxShape& box_shape1 = box1.add_box_shape(extents);
    box1.get_initial_state().set_position(origin);
    box1.get_initial_state().set_linear_velocity(Vector(5.0, 0.0, 0.0));
    box1.get_initial_state().set_angular_velocity(Vector(2, 0, 0));
    box1.set_mass(mass);
    box1.set_inertia_tensor(get_inertia_tensor_box(extents, mass));

    Body& box2 = system.create_body("box2");
    BoxShape& box_shape2 = box2.add_box_shape(extents);
    box2.set_mass(mass);
    box2.set_inertia_tensor(get_inertia_tensor_box(extents, mass));
    //OB::AngleAxis(0.5*OB::PI, OB::Vector::UnitZ().normalized());
    Quaternion initial_orientation{AngleAxis(OB::PI, OB::Vector{1,1,1}.normalized())};
    box2.get_initial_state().set_position(Vector(5, 20, 0));
    box2.get_initial_state().set_orientation(initial_orientation);
    box2.get_initial_state().set_linear_velocity(Vector(-2, 0, 0));

    system.insert_body_in_engine("box1", "ode");
    system.insert_body_in_engine("box2", "ode");

    system.insert_body_in_engine("box1", "bullet");
    system.insert_body_in_engine("box2", "bullet");

    Looper looper{system};
    looper.add_engine("ode", Vector(0, 10, 0));
    looper.add_engine("bullet", Vector(0, 0, 0));

    looper.loop();
}
