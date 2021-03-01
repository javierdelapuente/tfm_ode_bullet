#include "looper.hpp"
#include "oblog.hpp"


using namespace OB;


Looper::Looper(System& system, bool disable_render_bodies)
    :system(system),
     renderer(disable_render_bodies)
{
    //we should be more careful with constructors...
    renderer.initApp();
    renderer.addInputListener(this);
}


Looper::~Looper()
{
    renderer.closeApp();
}

void Looper::add_engine(const std::string& engine_name,
                        const Vector& display_offset,
                        const Quaternion &display_orientation)
{
    Engine& engine = system.get_engine(engine_name);
    renderer.add_engine(engine, display_offset, display_orientation);
    engines.push_back(engine);
}


bool Looper::keyPressed(const OgreBites::KeyboardEvent& evt)
{
    L_(linfo) << "Key pressed";
    if (evt.keysym.sym == OgreBites::SDLK_ESCAPE)
    {
        end_loop = true;
        return true;
    }
    if (evt.keysym.sym == OgreBites::SDLK_SPACE)
    {
        dynamic_simulation = !dynamic_simulation;
        return true;
    }
    if (evt.keysym.sym == 'c')
    {
        pause_on_collisions = !pause_on_collisions;
        return true;
    }
    return false;
}

bool Looper::mousePressed(const OgreBites::MouseButtonEvent& evt)
{
    L_(linfo) << "Mouse pressed";
    return false;
}


void Looper::loop(real_seconds time_to_advance)
{
    std::chrono::microseconds final_physics_time
        {current_physics_time
         + std::chrono::duration_cast<std::chrono::microseconds>(time_to_advance)};

    while (!end_loop && !renderer.getRoot()->endRenderingQueued()
           && final_physics_time > current_physics_time)
    {
        loop_once();
    }
}


void Looper::loop_once()
{
    std::chrono::high_resolution_clock::time_point pt = std::chrono::high_resolution_clock::now();
    std::chrono::microseconds previous_loop_duration =
        std::chrono::duration_cast<std::chrono::microseconds>(pt - previous_time_point);
    previous_time_point = pt;

    if (dynamic_simulation) {
        previous_loop_duration =
            std::chrono::duration_cast<std::chrono::microseconds>
            (previous_loop_duration * current_simulation_speed);

        if (previous_loop_duration.count() > 0) {
            current_physics_time += (previous_loop_duration);
        }
        for (Engine& engine : engines)
        {

            while (engine.get_physics_time() < current_physics_time)
            {
                L_(linfo) << "step engine " << engine.get_name()
                          << ". engine time: " << engine.get_physics_time().count()
                          << ". current time " << current_physics_time.count();
                engine.step();

                // auto lastContactData = engine.get_last_contact_points();
                // for (auto& cd : lastContactData)
                // {
                //     L_(linfo) << engine.get_name() << "Last contact data. point:" << cd.point.transpose()
                //               << " normal:" << cd.normal.transpose()
                //               << " distance:" << cd.distance;
                // }

                if (pause_on_collisions && engine.get_last_contact_points().size() > 0)
                {
                    dynamic_simulation = false;
                    break;
                }
            }
        }
    }
    else
    {
        // put real_time to lowest engine time
        for (Engine& engine: engines)
        {
            current_physics_time = std::min(current_physics_time, engine.get_physics_time());
            // grr to update things
            renderer.engine_post_step(engine);
        }
    }

    // This could block the current thread.
    // If VSync is on, rate goes to 60fps.
    renderer.renderOneFrame(current_physics_time, current_simulation_speed);
    //L_(linfo) << "time for loop " << previous_loop_duration.count()
    //          << " physics time " << current_physics_time.count();
}
