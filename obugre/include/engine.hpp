#ifndef OBUGRE_ENGINE_HPP
#define OBUGRE_ENGINE_HPP

#include <string>
#include <vector>
#include <chrono>
#include <fstream>

#include "body.hpp"

namespace OB
{

    // we could use this for body movement...
    class System;
    class Engine;
    class EngineListener
    {
    public:
        virtual void engine_pre_step(Engine &engine) {}
        virtual void engine_post_step(Engine &engine) {}
        virtual void body_added(Engine& engine, BodyState &state) {}
        virtual void body_deleted(Engine& engine, BodyState &state) {}
        virtual void constraint_added(Engine& engine, Constraint &constraint) {}
    };

    class EngineLogger : public EngineListener
    {
    public:
        enum class LogType{engine = 0, body, convex_test};
        EngineLogger(const std::string& filename) : log_stream(filename) {};
        void engine_pre_step(Engine &engine) override;
        void set_logger_only_engine(bool l) { logger_only_engine = l; }
        void set_logger_convex_time(bool l) { logger_convex_time = l; }

    private:
        std::ofstream log_stream;
        bool logger_only_engine = false;
        bool logger_convex_time = false;
    };



    struct ContactPoint {
        Point point;
        Vector normal;
        Real distance;
    };

    using real_seconds = std::chrono::duration<Real, std::ratio<1>>;

    enum class EngineType {Ode, Bullet};

    class Engine : protected ShapeVisitor,
                   protected ConstraintVisitor
    {
    public:

        Engine(const std::string& name, System& system): name(name), system(system) {}

        Engine(const Engine&) = delete;
        Engine& operator=(const Engine&) = delete;
        virtual ~Engine() = default;

        virtual void set_gravity(Vector gravity) = 0;
        void add_body(const Body& body);
        void add_constraint(Constraint& body);

        void step();
        void set_time_step(real_seconds ts) { time_step = ts;}
        real_seconds get_time_step() { return time_step;}
        int get_total_steps() { return total_steps; }

        virtual void set_use_vclip(bool vclip) { use_vclip = vclip; }
        bool is_use_vclip() const { return use_vclip; }

        void set_margin(Real new_margin) { margin = new_margin; }
        Real get_margin() { return margin; }

        const BodyState& get_bodystate(const std::string& name) const {
            return *(get_bodystate_map().at(name));
        }

        bool exists_bodystate(const std::string& name) const {
            return get_bodystate_map().count(name) > 0;
        }

        BodyState& get_bodystate(const std::string& name) {
            return *(get_bodystate_map().at(name));
        }

        bool exists_constraint(const std::string& name) const {
            return constraint_map.count(name) > 0;
        }

        const std::string& get_name() const { return name; }
        System& get_system() {return system; }

        Vector get_linear_momentum() const;
        Vector get_angular_momentum() const;
        Real get_kinetic_energy() const;

        void add_listener(EngineListener& listener) { listeners.push_back(listener); }

        class iterator_bodystate;
        iterator_bodystate begin() { return iterator_bodystate(get_bodystate_map().begin()); }
        iterator_bodystate end() { return iterator_bodystate(get_bodystate_map().end()); }

        std::chrono::microseconds get_physics_time() const { return physical_from_start; }
        std::chrono::microseconds get_used_time() const { return used_from_start; }

        virtual std::vector<ContactPoint> get_last_contact_points() = 0;

        using BodyStateMapOwning = std::map<std::string, std::unique_ptr<BodyState>>;
        using ConstraintMap = std::map<std::string, std::reference_wrapper<Constraint>>;

        void set_disable_vclip_cache(bool disable) { disable_vclip_cache = disable; }
        bool is_disable_vclip_cache() { return disable_vclip_cache; }
    protected:
        virtual void step_engine(real_seconds timestep) = 0;
        virtual void add_body_internal(BodyState& state) = 0;
        virtual void add_constraint_internal(Constraint& body) = 0;

        virtual void update_bodystates() = 0;
        BodyStateMapOwning& get_bodystate_map() { return bodystate_map; }
        const BodyStateMapOwning& get_bodystate_map() const { return bodystate_map; }


    private:
        std::string name;
        System& system;
        BodyStateMapOwning bodystate_map;
        ConstraintMap constraint_map;

        std::chrono::microseconds physical_from_start{0};
        std::chrono::microseconds used_from_start{0};
        real_seconds time_step{1.0/60.0};
        int total_steps = 0;


        bool use_vclip = false;
        Real margin = 0.08; // this is twice the margin for one object

        using EngineListeners = std::vector<std::reference_wrapper<EngineListener>>;
        EngineListeners listeners;

        bool disable_vclip_cache = false;

    public:
        class iterator_bodystate : public std::iterator<std::output_iterator_tag, BodyState>
        {
        public:
            explicit iterator_bodystate(BodyStateMapOwning::iterator inner_id):inner_id(inner_id) {}
            reference operator*() const { return *(inner_id->second); }
            iterator_bodystate operator++() { iterator_bodystate i = *this; inner_id++; return i; }
            iterator_bodystate operator++(int junk) { inner_id++; return *this; }
            bool operator!=(const iterator_bodystate &o) const { return inner_id != o.inner_id; }
        private:
            BodyStateMapOwning::iterator inner_id;
        };
    };


}

#endif
