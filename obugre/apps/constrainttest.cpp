#include "system.hpp"
#include "engine.hpp"
#include "looper.hpp"
#include <iostream>

using namespace OB;

//OB::AngleAxis(0.5*OB::PI, OB::Vector::UnitZ().normalized());

int main( int argc, const char* argv[] )
{
    System system{};

    system.set_default_gravity(Vector(0, 0, 0));

    system.create_engine("ode", EngineType::Ode);
    system.create_engine("bullet", EngineType::Bullet);

    Vector extents(2, 4, 2);
    Real radius{3};
    Real mass{6};

    Body& box1 = system.create_body("box1");
    BoxShape& box_shape1 = box1.add_box_shape(extents);
    box1.set_mass(mass);
    box1.set_inertia_tensor(get_inertia_tensor_box(extents, mass));
    Quaternion initialOrientation1{AngleAxis(OB::PI*0.25, OB::Vector{0,1,0}.normalized())};
    box1.get_initial_state().set_position(Vector(0, 0, 0));
    box1.get_initial_state().set_orientation(initialOrientation1);
    box1.get_initial_state().set_linear_velocity(Vector(0.0, 0.0, 5.0));
    box1.get_initial_state().set_angular_velocity(Vector(0, 0, 0));

    Body& box2 = system.create_body("box2");
    BoxShape& box_shape2 = box2.add_box_shape(extents);
    box2.set_mass(mass);
    box2.set_inertia_tensor(get_inertia_tensor_box(extents, mass));
    Quaternion initialOrientation2{AngleAxis(OB::PI*0.25, OB::Vector{0,1,0}.normalized())};
    //Quaternion initialOrientation2{AngleAxis(OB::PI*0, OB::Vector{0,1,0}.normalized())};
    box2.get_initial_state().set_position(Vector(4, 0, 0));
    box2.get_initial_state().set_orientation(initialOrientation2);
    box2.get_initial_state().set_linear_velocity(Vector(0, 0, 0));
    box2.get_initial_state().set_angular_velocity(Vector(0, -10, 0));

    HingeConstraint& constraint =
        system.create_hinge_constraint("c1", "box1", "box2",
                                       Vector(0, 1, 0), Point(2, 0, 0));


    system.insert_all_bodies_in_all_engines();
    system.insert_all_constraints_in_all_engines();

    Looper looper{system};
    looper.set_pause_on_collisions(true);
    looper.add_engine("ode", Vector(10, 0, 0));
    looper.add_engine("bullet", Vector(-10, 0, 0));
    looper.set_dynamic_simulation(false);
    looper.loop_once();
    looper.set_dynamic_simulation(false);
    looper.loop_once();
    looper.set_dynamic_simulation(false);
    looper.loop();

}
