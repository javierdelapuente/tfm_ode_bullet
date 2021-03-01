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
        {
            std::vector<Point> points =  get_prism_points(5, 1, 1);
            for (Point& p : points) { p = p*2; }
            body.add_convex_shape(points);
            break;
        }
    case 1:
        {
            std::vector<Point> points =  get_rbox_points("10000 s D3"); /// 1000 points in a 3d sphere
            body.add_sphere_shape(radius);
            break;
        }
    case 2:
        {
            std::vector<Point> points =  get_bunny_points();
            for (Point& p : points) { p = p*2; }
            body.add_convex_shape(points);
            break;
        }
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
    FILELog::ReportingLevel() = linfo;

    System system{};
    system.set_default_gravity(Vector(0, -10, 0));

    EngineLogger engine_logger{"performancevclip.csv"};
    engine_logger.set_logger_only_engine(true);

    Real restitution{0.1};
    Real friction{6.3};


    Engine& odevclip = system.create_engine("odevclip", EngineType::Ode);
    odevclip.set_use_vclip(true);
    OdeEngine& odevclipengine = dynamic_cast<OdeEngine&>(odevclip);
    odevclipengine.set_friction(friction);
    odevclipengine.set_restitution(restitution);

    Engine& bulletvclip = system.create_engine("bulletvclip", EngineType::Bullet);
    bulletvclip.set_use_vclip(true);
    BulletEngine& bulletvclipengine = dynamic_cast<BulletEngine&>(bulletvclip);
    bulletvclipengine.set_friction(friction);
    bulletvclipengine.set_restitution(restitution);

    Engine& ode = system.create_engine("ode", EngineType::Ode);
    ode.add_listener(engine_logger);
    OdeEngine& odeengine = dynamic_cast<OdeEngine&>(ode);
    odeengine.set_friction(friction);
    odeengine.set_restitution(restitution);

    Engine& bullet = system.create_engine("bullet", EngineType::Bullet);
    bullet.add_listener(engine_logger);
    std::cout << "todo creado " << std::endl;
    BulletEngine& bulletengine = dynamic_cast<BulletEngine&>(bullet);
    bulletengine.set_friction(friction);
    bulletengine.set_restitution(restitution);

    Plane ground_plane{Vector(0, 1, -1).normalized(), 0};
    Body& ground =  system.create_body("ground");
    ground.get_initial_state().set_position(Vector(0,-10,10));
    ground.set_static(true);
    PlaneShape& ground_shape = ground.add_plane_shape(ground_plane);

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
    looper.add_engine("odevclip", Vector{0, 0, 0});
    //looper.add_engine("bulletvclip", Vector{50, 0, 50});
    //looper.add_engine("bullet", Vector{50, 0, -50});
    //looper.add_engine("ode", Vector{-50, 0, -50});



    looper.set_simulation_speed(1.0);
    looper.loop();


}
