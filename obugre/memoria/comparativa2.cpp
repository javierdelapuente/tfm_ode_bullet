#include <iostream>

#include "system.hpp"
#include "engine.hpp"
#include "looper.hpp"
#include "oblog.hpp"
#include "ode_engine.hpp"
#include "bullet_engine.hpp"


using namespace OB;

// NO CONSIGO EXPLICAR LA DIFERENCIA DE ENERGIA ENTRE ODE Y BULLET POR EL MARGEN EN LA COLISION
// AL CAER AL SUELO

//bulletBody->setContactStiffnessAndDamping(bulletBody->getContactStiffness(), 0.0);


int main( int argc, const char* argv[] )
{
    System system{};
    system.set_default_gravity(Vector(0, -10, 0));

    EngineLogger engine_logger{"comparativa2.csv"};

    Engine& ode = system.create_engine("ode", EngineType::Ode);
    ode.add_listener(engine_logger);
    OdeEngine& odeengine = dynamic_cast<OdeEngine&>(ode);
    odeengine.set_friction(1000.0);
    odeengine.set_restitution(0.0);


    Engine& bullet = system.create_engine("bullet", EngineType::Bullet);
    bullet.add_listener(engine_logger);
    BulletEngine& bulletengine = dynamic_cast<BulletEngine&>(bullet);
    bulletengine.set_friction(1000.0);
    bulletengine.set_restitution(0.0);

    std::cout << "todo creado " << std::endl;

    Real radius(4);
    Real mass(1);

    Plane ground_plane{Vector(1, 1, 0).normalized(), 0};
    Body& ground =  system.create_body("ground");
    ground.set_static(true);
    PlaneShape& ground_shape = ground.add_plane_shape(ground_plane);

    Plane ground_plane2{Vector(0, 1, 0).normalized(), 0};
    Body& ground2 =  system.create_body("ground2");
    ground2.get_initial_state().set_position(Vector(0,-30,0));
    ground2.set_static(true);
    PlaneShape& ground2_shape = ground2.add_plane_shape(ground_plane2);


    Body& sphere1 = system.create_body("sphere1");
    SphereShape& sphere1_shape = sphere1.add_sphere_shape(radius);
    sphere1.set_mass(mass);
    sphere1.set_inertia_tensor(get_inertia_tensor_sphere(radius, mass));
    //L_(linfo) << "Inertia sphere "  <<  get_inertia_tensor_sphere(radius, mass);
    Vector position1{ground_plane.normal()*radius};
    sphere1.get_initial_state().set_position(position1);


    // momento de inercia de https://en.wikipedia.org/wiki/List_of_moments_of_inertia
    // esto es cascarón esférico
    Body& sphere2 = system.create_body("sphere2");
    SphereShape& sphere2_shape = sphere2.add_sphere_shape(radius);
    sphere2.set_mass(mass);
    sphere2.set_inertia_tensor(get_inertia_tensor_spheric_shell(radius, radius-0.5, mass));
    //L_(linfo) << "Inertia tensor spheric shell "  <<  get_inertia_tensor_spheric_shell(radius, radius-0.5, mass);
    Vector position2{ground_plane.normal()*radius};
    sphere2.get_initial_state().set_position(position2 + Vector(0,0,10));

    system.insert_all_bodies_in_all_engines();

    Looper looper{system};

    looper.add_engine("ode", Vector{-40, 0, 0});
    Quaternion bullet_orientation{AngleAxis(OB::PI, OB::Vector{0,1,0}.normalized())};
    looper.add_engine("bullet", Vector{+40, 0, 0}, bullet_orientation);

    looper.loop();


}
