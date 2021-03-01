#include <iostream>

#include "system.hpp"
#include "engine.hpp"
#include "looper.hpp"
#include "oblog.hpp"

using namespace OB;

int main( int argc, const char* argv[] )
{
    System system{};
    system.set_default_gravity(Vector(0, 0, 0));

    EngineLogger engine_logger{"comparativa1.csv"};

    Engine& ode = system.create_engine("ode", EngineType::Ode);
    ode.add_listener(engine_logger);
    Engine& bullet = system.create_engine("bullet", EngineType::Bullet);
    bullet.add_listener(engine_logger);
    std::cout << "todo creado " << std::endl;

    //Vector origin(0, 20, 0);
    Vector extents(4, 2, 6);
    Real mass(1);

    Body& boxmayor = system.create_body("boxmayor");
    BoxShape& boxmayor_shape = boxmayor.add_box_shape(extents);
    boxmayor.set_mass(mass);
    boxmayor.set_inertia_tensor(get_inertia_tensor_box(extents, mass));
    boxmayor.get_initial_state().set_position(Vector(-20, 16, 0));
    Quaternion initialOrientation2{AngleAxis(OB::PI*0.5, OB::Vector{0,0,1}.normalized())};
    boxmayor.get_initial_state().set_orientation(initialOrientation2);
    //box2.get_initial_state().set_linear_velocity(Vector(2.0, 0.0, 0.0));
    boxmayor.get_initial_state().set_angular_velocity(Vector(2.0, 0.1, 0.0));

    Body& boxmenor = system.create_body("boxmenor");
    BoxShape& boxmenor_shape = boxmenor.add_box_shape(extents);
    boxmenor.set_mass(mass);
    boxmenor.set_inertia_tensor(get_inertia_tensor_box(extents, mass));
    boxmenor.get_initial_state().set_position(Vector(-20, 0, 0));
    Quaternion initialOrientation3{AngleAxis(OB::PI*0.5, OB::Vector{0,1,0}.normalized())};
    boxmenor.get_initial_state().set_orientation(initialOrientation3);
    //box3.get_initial_state().set_linear_velocity(Vector(2.0, 0.0, 0.0));
    boxmenor.get_initial_state().set_angular_velocity(Vector(2.0, 0.1, 0.0));


    Body& boxintermedio = system.create_body("boxintermedio");
    BoxShape& boxintermedio_shape1 = boxintermedio.add_box_shape(extents);
    boxintermedio.set_mass(mass);
    boxintermedio.set_inertia_tensor(get_inertia_tensor_box(extents, mass));
    boxintermedio.get_initial_state().set_position(Vector(-20, 8, 0));
    //box1.get_initial_state().set_linear_velocity(Vector(2.0, 0.0, 0.0));
    boxintermedio.get_initial_state().set_angular_velocity(Vector(2.0, 0.1, 0.0));

    system.insert_all_bodies_in_all_engines();

    Looper looper{system};
    //looper.set_simulation_speed(20);
    looper.set_simulation_speed(1);
    looper.add_engine("ode");
    looper.add_engine("bullet", Vector{0, -24, 0});

    looper.set_simulation_speed(10.0);
    looper.loop(real_seconds{28.0});
    looper.set_dynamic_simulation(false);
    looper.set_simulation_speed(100.0);
    // a la los 45 minutos
    looper.loop(real_seconds{45*60-28.0});
    looper.set_simulation_speed(1.0);
    looper.set_dynamic_simulation(false);
    looper.loop();

}
