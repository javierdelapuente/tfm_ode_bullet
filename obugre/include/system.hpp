#ifndef OBUGRE_SYSTEM_HPP
#define OBUGRE_SYSTEM_HPP

#include <vector>
#include <obconfig.hpp>
#include "engine.hpp"
#include "body.hpp"

namespace OB
{

    class System
    {
    public:
        System() = default;
        System(const System&) = delete;
        System& operator=(const System&) = delete;

        Engine& create_engine(const std::string& name, EngineType type);

        Body& create_body(const std::string& name);

        void insert_body_in_engine(const std::string& body,
                                   const std::string& engine);

        void insert_constraint_in_engine(const std::string& constraint,
                                         const std::string& engine);

        void insert_body_in_all_engines(const std::string& body);

        void insert_all_bodies_in_all_engines();
        void insert_all_constraints_in_all_engines();

        // the way we create constraints (with the initial body position)
        // limits its utility, as it is based on initial body positions.
        HingeConstraint& create_hinge_constraint(const std::string& name,
                                                 const std::string& body1,
                                                 const std::string& body2,
                                                 const Vector& axis_global,
                                                 const Point& point_global);

        void set_default_gravity(Vector gravity);

        Engine& get_engine(const std::string& name) { return *engines_map.at(name); }


    private:
        using EngineMapOwning = std::map<std::string, std::unique_ptr<Engine>>;
        EngineMapOwning engines_map;

        using ConstraintVectorOwning = std::map<std::string, std::unique_ptr<Constraint>>;
        ConstraintVectorOwning constraints_map;

        using BodyMapOwning = std::map<std::string, std::unique_ptr<Body>>;
        BodyMapOwning bodies_map;

        Vector gravity{0, -9.8, 0};
    };


}


#endif
