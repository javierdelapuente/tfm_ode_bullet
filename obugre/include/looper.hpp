#ifndef OBUGRE_LOOPER_HPP
#define OBUGRE_LOOPER_HPP

#include <OgreInput.h>

#include "system.hpp"
#include "engine.hpp"
#include "renderer.hpp"

namespace OB
{

    enum class EngineLogType {engine = 0, body};

    class Looper : public OgreBites::InputListener
    {
    public:
        Looper(System& system, bool disable_render_bodies = false);
        ~Looper();

        void loop(real_seconds time_to_advance = real_seconds{3600*24});
        void loop_once();

        bool keyPressed(const OgreBites::KeyboardEvent& evt) override;
        bool mousePressed(const OgreBites::MouseButtonEvent& evt) override;

        void add_engine(const std::string& engine_name,
                        const Vector &display_offset = Vector::Zero(),
                        const Quaternion &display_orientation = Quaternion::Identity());

        void set_pause_on_collisions (bool p) { pause_on_collisions = p; }
        bool is_pause_on_collisions() { return pause_on_collisions; }
        bool is_end_loop() { return end_loop; }

        void set_dynamic_simulation (bool p) { dynamic_simulation = p; }

        void set_simulation_speed (Real speed) {
            simulation_speed = speed;
            current_simulation_speed = speed;
        }


        Looper() = delete;
        Looper(const Looper&) = delete;
        Looper& operator=(Looper const&) = delete;

    private:

        std::chrono::microseconds current_physics_time{0};
        std::chrono::high_resolution_clock::time_point previous_time_point
            {std::chrono::high_resolution_clock::time_point::min()};

        // faster or slower than real time if different from 1
        Real simulation_speed = 1.0;
        Real current_simulation_speed = simulation_speed;

        System &system;
        bool pause_on_collisions = false;
        bool end_loop = false;
        bool dynamic_simulation = false;

        Renderer renderer;

        using EnginesVector = std::vector<std::reference_wrapper<Engine>>;
        EnginesVector engines;
    };

}

#endif
