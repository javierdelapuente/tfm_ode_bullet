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

std::string generate_string(int max_length){
    std::string possible_characters
        = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz";
    std::random_device rd;
    std::mt19937 engine(rd());
    std::uniform_int_distribution<> dist(0, possible_characters.size()-1);
    std::string ret = "";
    for(int i = 0; i < max_length; i++){
        int random_index = dist(engine); //get index between 0 and possible_characters.size()-1
        ret += possible_characters[random_index];
    }
    return ret;
}

Body& create_random_body(System &system, Vector position)
{

    //Vector origin(0, 20, 0);
    Vector extents(2, 1, 3);
    Real radius(1.5);
    Real mass(6);

    std::string body_name{generate_string(10)};

    // create body
    Body& body = system.create_body(body_name);
    std::random_device rd;
    std::mt19937 engine(rd());
    std::uniform_int_distribution<> dist(0, 2);
    switch(dist(engine))
    {
    case 0:
        body.add_box_shape(extents);
        break;
    case 1:
        body.add_sphere_shape(radius);
        break;
    case 2:
        std::vector<Point> points =  get_bunny_points();
        for (Point& p : points) { p = p*2; }
        body.add_convex_shape(points);
        break;
    }


    body.set_mass(mass);
    body.set_inertia_tensor(get_inertia_tensor_box(extents, mass));
    body.get_initial_state().set_position(position);
    body.get_initial_state().set_linear_velocity(Vector(2.0, -10.0, 0.0));
    body.get_initial_state().set_angular_velocity(Vector(2.0, 0.1, 0.0));
    return body;
}


int main( int argc, const char* argv[] )
{
    System system{};
    system.set_default_gravity(Vector(0, -10, 0));

    EngineLogger engine_logger{"comparativa4.csv"};
    engine_logger.set_logger_only_engine(true);

    Engine& ode = system.create_engine("ode", EngineType::Ode);
    ode.add_listener(engine_logger);

    OdeEngine& odeengine = dynamic_cast<OdeEngine&>(ode);
    odeengine.set_friction(0.5);
    odeengine.set_restitution(0.5);

    Engine& bullet = system.create_engine("bullet", EngineType::Bullet);
    bullet.add_listener(engine_logger);
    std::cout << "todo creado " << std::endl;


    BulletEngine& bulletengine = dynamic_cast<BulletEngine&>(bullet);
    bulletengine.set_friction(0.5);
    bulletengine.set_restitution(0.5);


    Plane ground_plane{Vector(0, 1, -1).normalized(), 0};
    Body& ground =  system.create_body("ground");
    ground.get_initial_state().set_position(Vector(0,-50,10));
    ground.set_static(true);
    PlaneShape& ground_shape = ground.add_plane_shape(ground_plane);

    Plane ground_plane_left{Vector(1, 1, 0).normalized(), 0};
    Body& ground_left =  system.create_body("ground_left");
    ground_left.get_initial_state().set_position(Vector(-10,-50,0));
    ground_left.set_static(true);
    PlaneShape& ground_shape_left = ground_left.add_plane_shape(ground_plane_left);

    Plane ground_plane_right{Vector(-1, 1, 0).normalized(), 0};
    Body& ground_right =  system.create_body("ground_right");
    ground_right.get_initial_state().set_position(Vector(10,-50,0));
    ground_right.set_static(true);
    PlaneShape& ground_shape_right = ground_right.add_plane_shape(ground_plane_right);

    Plane ground_plane_back{Vector(0, 1, 1).normalized(), 0};
    Body& ground_back =  system.create_body("ground_back");
    ground_back.get_initial_state().set_position(Vector(0,-50,-10));
    ground_back.set_static(true);
    PlaneShape& ground_shape_back = ground_back.add_plane_shape(ground_plane_back);


    system.insert_all_bodies_in_all_engines();

    Looper looper{system, false};
    looper.add_engine("ode", Vector{-50, 0, 0});
    looper.add_engine("bullet", Vector{50, 0, 0});

    int number_objects = 1000;
    int objects_per_second = 10;
    looper.set_simulation_speed(0.2);

    for (int i = 0; i < number_objects/objects_per_second; i++)
    {
        L_(linfo) << " LOOP " << i;
        looper.loop(real_seconds{1.0});
        for (int i = 0; i < objects_per_second; i++)
        {
            Body& body = create_random_body(system, Vector((i-objects_per_second/2)*10 , 30, 0));
            system.insert_body_in_engine(body.get_name(), "ode");
            system.insert_body_in_engine(body.get_name(), "bullet");
        }
    }
    looper.loop(real_seconds{50.0});

    looper.set_dynamic_simulation(false);
    if (!looper.is_end_loop())
    {
        looper.loop();
    }


}
