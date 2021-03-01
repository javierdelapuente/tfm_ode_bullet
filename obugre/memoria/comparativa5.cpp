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

struct BrickDimensions
{
    int length = 3;
    int width = 1;
    int height = 1;
    int mass = 1;
};

struct LayerDimensions
{
    int length = 7;
    int width = 7;
    int spacing = 1;
};



class House
{
 public:
    House (System &system) : system(system) {}
    System &system;

    BrickDimensions brick_dimensions{};
    LayerDimensions layer_dimensions{};
    Body * base = nullptr;
    Body * roof = nullptr;

    int num_layers = 50;

    Vector base_position{0,0,0};

    int base_height = 1;
    int roof_height = 1;
    int base_mass = 10;
    int roof_mass = 10;

    void build()
    {

        Vector curr_base_position{base_position};

        Vector extents_base{ static_cast<Real>(layer_dimensions.length), 
            static_cast<Real>(base_height), 
            static_cast<Real>(layer_dimensions.width)};
        base = &system.create_body("base");
        base->add_box_shape(extents_base);
        base->set_mass(base_mass);
        base->set_inertia_tensor(get_inertia_tensor_box
                                 (Vector(extents_base), base_mass));

        base->get_initial_state().set_position(curr_base_position + extents_base/2);
        curr_base_position += base_height*Vector(0,1,0);

        for (int i = 0; i < num_layers; i++)
        {
            // create layer in curr_base_position
            int offset = (layer_dimensions.spacing + 1) * (i%2);
            create_layer(curr_base_position, offset);
            curr_base_position += Vector(0,1,0)*brick_dimensions.height;
        }


        Vector extents_roof{ static_cast<Real>(layer_dimensions.length), 
            static_cast<Real>(roof_height), 
            static_cast<Real>(layer_dimensions.width)};
        roof = &system.create_body("roof");
        roof->add_box_shape(extents_roof);
        roof->set_mass(roof_mass);
        roof->set_inertia_tensor(get_inertia_tensor_box
                                 (Vector(extents_roof), roof_mass));

        roof->get_initial_state().set_position(curr_base_position + extents_roof/2);

    }

    // clockwise.
    enum BrickLine {leftright, backfront, rightleft, frontback, endlines };

    void create_layer(Vector curr_base_position, int offset)
    {
        L_(linfo) << "creating layer";
        bool brick_in_end_line = false;
        int line_position = offset;

        for (int line = leftright; line != endlines; line++ )
        {
            BrickLine brick_line = static_cast<BrickLine>(line);
            int line_length = 0;
            if (brick_line == leftright ||  brick_line == rightleft)
            {
                line_length = layer_dimensions.length;
            }
            else
            {
                line_length = layer_dimensions.width;
            }

            L_(linfo) << "creating line in layer";
            while (line_position + layer_dimensions.spacing < line_length)
            {
                // do not put last brick in layer if it collides with the first one
                if (brick_line == frontback && offset < layer_dimensions.spacing + brick_dimensions.width) {
                    if (line_position + brick_dimensions.length >= line_length)
                    {
                        break;
                    }
                }
                L_(linfo) << "line_position " << line_position;

                create_brick(curr_base_position, brick_line, line_position);
                line_position += brick_dimensions.length + layer_dimensions.spacing;
            }

            if (line_position >= line_length)
            {
                line_position = layer_dimensions.spacing + brick_dimensions.width;
            }
            else
            {
                line_position = 0;
            }


        }
    }

    void create_brick(Vector curr_base_position, BrickLine line_orientation, int line_position)
    {
        L_(linfo) << "creating brick. orientation " << line_orientation
                  << " position_in_line " << line_position;
        Vector extents;
        Vector brick_position;

        switch (line_orientation) {
        case leftright:
            extents = Vector(brick_dimensions.length, brick_dimensions.height, brick_dimensions.width);
            brick_position = Vector(1,0,0)*line_position + extents/2;
            break;
        case backfront:
            extents = Vector(brick_dimensions.width, brick_dimensions.height, brick_dimensions.length);
            brick_position = Vector(0,0,1)*line_position + extents/2 + Vector(1,0,0)*(layer_dimensions.length-1);
            break;
        case rightleft:
            extents = Vector(brick_dimensions.length, brick_dimensions.height, brick_dimensions.width);
            //brick_position = Vector(-1,0,0)*line_position + Vector(0,0,1)*layer_dimensions.width + extents/2;
            brick_position = Vector(-1,0,0)*line_position + Vector(0,0,1)*(layer_dimensions.width-1) +  Vector(1,0,0)*(layer_dimensions.length) + extents/2 - Vector(extents[0],0,0);
            break;
        case frontback:
            extents = Vector(brick_dimensions.width, brick_dimensions.height, brick_dimensions.length);
            brick_position = Vector(0,0,-1)*line_position + Vector(0,0,1)*(layer_dimensions.width) + extents/2 - Vector(0,0,extents[2]);
            break;
        }

        Body& brick  = system.create_body("brick-" + generate_string(5) );
        brick.add_box_shape(extents);
        brick.set_mass(brick_dimensions.mass);
        brick.set_inertia_tensor(get_inertia_tensor_box
                                 (Vector(extents), brick_dimensions.mass));

        brick.get_initial_state().set_position(curr_base_position + brick_position);

    }
};


class BallDestroyer: public EngineListener
{
public:

    bool ball_thrown = false;

    void engine_pre_step(Engine &engine) override
    {
        if (!ball_thrown)
        {
            std::chrono::seconds sec{10};
            if (engine.get_physics_time() > sec)
            {
                L_(linfo) << "launching ball " << engine.get_physics_time().count();
                Real mass{20};
                Real radius{2};
                Body& body = engine.get_system().create_body("sphere");
                body.add_sphere_shape(radius);
                body.set_mass(mass);
                body.set_inertia_tensor(get_inertia_tensor_sphere(radius, mass));
                body.get_initial_state().set_position(Vector(5, 20, 40));
                body.get_initial_state().set_linear_velocity(Vector(0, 0.0, -50.0));
                engine.get_system().insert_body_in_all_engines(body.get_name());
                ball_thrown = true;
            }
        }
    }
};


int main( int argc, const char* argv[] )
{
    System system{};
    system.set_default_gravity(Vector(0, -10, 0));

    //EngineLogger engine_logger{"comparativa5.csv"};
    BallDestroyer destroyer{};

    // Real friction{1};
    // Real restitution{0.9};

    Real friction{1};
    Real restitution{0.3};

    Engine& ode = system.create_engine("ode", EngineType::Ode);
    //ode.add_listener(engine_logger);
    ode.add_listener(destroyer);

    OdeEngine& odeengine = dynamic_cast<OdeEngine&>(ode);
    odeengine.set_friction(friction*0.5);
    odeengine.set_restitution(restitution);
    odeengine.set_quick_step(true);
    odeengine.set_time_step(real_seconds{1.0/60});

    Engine& bullet = system.create_engine("bullet", EngineType::Bullet);
    //bullet.add_listener(engine_logger);
    bullet.set_time_step(real_seconds{1.0/240});


    BulletEngine& bulletengine = dynamic_cast<BulletEngine&>(bullet);
    bulletengine.set_friction(friction);
    bulletengine.set_restitution(restitution);

    std::cout << "todo creado " << std::endl;

    Plane ground_plane{Vector(0, 1, 0).normalized(), 0};
    Body& ground =  system.create_body("ground");
    ground.get_initial_state().set_position(Vector(0,0,0));
    ground.set_static(true);
    PlaneShape& ground_shape = ground.add_plane_shape(ground_plane);

    House house{system};
    house.build();

    system.insert_all_bodies_in_all_engines();

    Looper looper{system};
    looper.set_simulation_speed(0.4);
    looper.add_engine("ode", Vector{-50, 0, 0});
    looper.add_engine("bullet", Vector{50, 0, 0});

    looper.loop();

}
