#include <iostream>

#include "system.hpp"
#include "engine.hpp"
#include "looper.hpp"
#include "oblog.hpp"
#include "ode_engine.hpp"
#include "bullet_engine.hpp"

using namespace OB;

int main( int argc, const char* argv[] )
{
    System system{};
    system.set_default_gravity(Vector(0, -10, 0));
    EngineLogger engine_logger{"comparativa3.csv"};

    Engine& ode = system.create_engine("ode", EngineType::Ode);
    ode.add_listener(engine_logger);

    OdeEngine& odeengine = dynamic_cast<OdeEngine&>(ode);
    odeengine.set_friction(0.0);
    odeengine.set_restitution(1.0);

    Engine& ode2 = system.create_engine("ode2", EngineType::Ode);
    ode2.add_listener(engine_logger);
    ode2.set_time_step(real_seconds{1.0/10});

    OdeEngine& odeengine2 = dynamic_cast<OdeEngine&>(ode2);
    odeengine2.set_friction(0.0);
    odeengine2.set_restitution(1.0);


    Engine& bullet = system.create_engine("bullet", EngineType::Bullet);
    bullet.add_listener(engine_logger);

    BulletEngine& bulletengine = dynamic_cast<BulletEngine&>(bullet);
    bulletengine.set_friction(0.0);
    bulletengine.set_restitution(1.0);

    Engine& bullet2 = system.create_engine("bullet2", EngineType::Bullet);
    bullet2.set_time_step(real_seconds{1.0/240});
    bullet2.add_listener(engine_logger);

    BulletEngine& bulletengine2 = dynamic_cast<BulletEngine&>(bullet2);
    bulletengine2.set_friction(0.0);
    bulletengine2.set_restitution(1.0);


    std::cout << "todo creado " << std::endl;

    Plane ground_plane{Vector(0, 1, 0).normalized(), 0};
    Body& ground =  system.create_body("ground");
    ground.get_initial_state().set_position(Vector(0,-20,0));
    ground.set_static(true);
    PlaneShape& ground_shape = ground.add_plane_shape(ground_plane);


    Vector extents(6, 2, 0.5);
    Real mass(5);

    Body& pole = system.create_body("pole");
    pole.set_static(true);
    BoxShape& pole_shape1 = pole.add_box_shape(Vector(2, 20, 2));
    //Quaternion initialOrientation1{AngleAxis(OB::PI*0.25, OB::Vector{0,1,0}.normalized())};
    pole.get_initial_state().set_position(Vector(0, -10, 0));


    Body& box1 = system.create_body("box" +  std::to_string(0));
    box1.set_static(true);
    BoxShape& box_shape1 = box1.add_box_shape(extents);
    box1.set_mass(mass);
    box1.set_inertia_tensor(get_inertia_tensor_box(extents, mass));
    //Quaternion initialOrientation1{AngleAxis(OB::PI*0.25, OB::Vector{0,1,0}.normalized())};
    box1.get_initial_state().set_position(Vector(0, 0, 0));
    //box1.get_initial_state().set_orientation(initialOrientation1);
    //box1.get_initial_state().set_linear_velocity(Vector(0.0, 100.0, 0.0));
    //box1.get_initial_state().set_angular_velocity(Vector(0, 100.0, 0));

    int num_links = 10;
    for (int i = 0; i < num_links; i++)
    {
        Body& box = system.create_body(std::string("box") + std::to_string(i+1));
        BoxShape& box_shape = box.add_box_shape(extents);
        box.set_mass(mass);
        box.set_inertia_tensor(get_inertia_tensor_box(extents, mass));
        //Quaternion initialOrientation1{AngleAxis(OB::PI*0.25, OB::Vector{0,1,0}.normalized())};
        box.get_initial_state().set_position(Vector((i+1)*7, 0, 0));
        //box1.get_initial_state().set_orientation(initialOrientation1);
        box.get_initial_state().set_linear_velocity(Vector(0, 0, 0));
        box.get_initial_state().set_angular_velocity(Vector(0, 0, 0));

        HingeConstraint& constraint =
            system.create_hinge_constraint(std::string("c1") + std::to_string(i),
                                           "box" + std::to_string(i),
                                           "box"  + std::to_string(i+1),
                                           Vector(0, 1, 0),
                                           Point(Vector( (i+1)*7 -3.5, 0, 0)));

        // if ( i == num_links -1 )
        // {
        //     box.get_initial_state().set_linear_velocity(Vector(0, 0, 100));;
        // }
    }
    Real sphere_radius(3);
    Real sphere_mass(10);
    Body& sphere = system.create_body("sphere");
    SphereShape& sphere_shape = sphere.add_sphere_shape(sphere_radius);
    sphere.set_mass(sphere_mass);
    sphere.set_inertia_tensor(get_inertia_tensor_sphere(sphere_radius, sphere_mass));
    //L_(linfo) << "Inertia sphere "  <<  get_inertia_tensor_sphere(radius, mass);
    Vector position_sphere{ static_cast<Real>((num_links + 1) * 7),0,0 };
    sphere.get_initial_state().set_position(position_sphere);
    sphere.get_initial_state().set_linear_velocity(Vector(0, 0, 100));
    //sphere.get_initial_state().set_angular_velocity(Vector(0, 10, 0));

    HingeConstraint& constraint =
        system.create_hinge_constraint(std::string("final"),
                                       "box" + std::to_string(num_links),
                                       "sphere",
                                       Vector(0, 1, 0),
                                       Point(Vector( (num_links+1)*7 -3.5, 0, 0)));



    system.insert_all_bodies_in_all_engines();
    system.insert_all_constraints_in_all_engines();

    Looper looper{system};
    looper.add_engine("ode", Vector{0, 0, 0});
    //looper.add_engine("ode2", Vector{0, 0, 100});
    looper.add_engine("bullet", Vector{100, 0, 0});
    looper.add_engine("bullet2", Vector{200, 0, 0});

    looper.loop();

}
