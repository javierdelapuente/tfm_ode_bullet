#include "system.hpp"
#include "ode_engine.hpp"
#include "bullet_engine.hpp"

using namespace OB;

Engine& System::create_engine(const std::string& name, EngineType engine_type)
{
    std::unique_ptr<Engine> engineptr;
    switch(engine_type) {
    case EngineType::Ode:
        engineptr.reset(new OdeEngine(name, *this));
        break;
    case EngineType::Bullet:
        engineptr.reset(new BulletEngine(name, *this));
        break;
    default:
        throw std::logic_error("System::createEngine. unknown engine type." );
    }

    auto resp = engines_map.insert({name ,std::move(engineptr)});
    Engine& engine = *(resp.first->second);
    engine.set_gravity(gravity);
    return engine;
}

void System::set_default_gravity(Vector new_gravity)
{
    gravity = new_gravity;
}


Body& System::create_body(const std::string& name)
{
    if (bodies_map.count(name) > 0) {
        L_(lerror) << "error inserting: " << name;
        throw std::logic_error("body name already in system");
    }

    std::unique_ptr<Body> body = std::unique_ptr<Body>(new Body(name));
    auto resp = bodies_map.insert({name ,std::move(body)});
    Body& bodyresp = *(resp.first->second);
    return bodyresp;
}

void System::insert_body_in_engine(const std::string& body_name,
                                   const std::string& engine_name)
{
    Body& body = *bodies_map.at(body_name);
    engines_map.at(engine_name)->add_body(body);
}


void System::insert_body_in_all_engines(const std::string& body_name)
{
    Body& body = *bodies_map.at(body_name);
    for (auto& engine : engines_map)
    {
        engine.second->add_body(body);
    }
}


void System::insert_all_bodies_in_all_engines()
{
    for (auto& engine : engines_map)
    {
        for (auto& body : bodies_map)
        {
            engine.second->add_body(*body.second);
        }
    }
}

void System::insert_all_constraints_in_all_engines()
{
    for (auto& engine: engines_map)
    {
        for (auto& constraint: constraints_map)
        {
            engine.second->add_constraint(*constraint.second);
        }
    }
}

void System::insert_constraint_in_engine(const std::string& constraint_name,
                                   const std::string& engine_name)
{
    Constraint& constraint = *constraints_map.at(constraint_name);
    engines_map.at(engine_name)->add_constraint(constraint);
}


HingeConstraint& System::create_hinge_constraint(const std::string& name,
                                                 const std::string& body_name1,
                                                 const std::string& body_name2,
                                                 const Vector& axis_global,
                                                 const Point& point_global)
{
    if (constraints_map.count(name) > 0) {
        throw std::logic_error("constraint name already in system");
    }

    Body& body1 = *bodies_map.at(body_name1);
    Body& body2 = *bodies_map.at(body_name2);

    std::unique_ptr<Constraint> constraint =
        std::unique_ptr<Constraint>(new HingeConstraint(name,
                                                        body1.get_initial_state(),
                                                        body2.get_initial_state(),
                                                        axis_global,
                                                        point_global));
    auto resp = constraints_map.insert({name ,std::move(constraint)});

    return dynamic_cast<HingeConstraint&>(*(resp.first->second));
}
