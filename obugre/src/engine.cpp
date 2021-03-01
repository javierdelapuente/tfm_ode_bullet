#include <chrono>

#include "engine.hpp"
#include "oblog.hpp"

using namespace OB;

void Engine::step()
{
    for (EngineListener& listener : listeners)
    {
        listener.engine_pre_step(*this);
    }
    std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
    step_engine(time_step);
    total_steps++;
    std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
    used_from_start += std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1);
    physical_from_start += std::chrono::duration_cast<std::chrono::microseconds>(time_step);

    update_bodystates();
    for (EngineListener& listener : listeners)
    {
        listener.engine_post_step(*this);
    }
}

void Engine::add_body(const Body& body)
{
    if (exists_bodystate(body.get_name())) {
        throw std::logic_error("body already in engine");
    }

    get_bodystate_map()[body.get_name()] =
        std::unique_ptr<BodyState>(new BodyState(body.get_initial_state()));

    BodyState& body_state = get_bodystate(body.get_name());

    add_body_internal(body_state);

    for (EngineListener& listener : listeners)
    {
        listener.body_added(*this, body_state);
    }
}

void Engine::add_constraint(Constraint& constraint)
{
    if (exists_constraint(constraint.get_name())) {
        throw std::logic_error("constraint already in engine");
    }

    constraint_map.emplace(constraint.get_name(), std::ref(constraint));

    add_constraint_internal(constraint);

    for (EngineListener& listener : listeners)
    {
        listener.constraint_added(*this, constraint);
    }
}



Vector Engine::get_linear_momentum() const
{
    Vector total = Vector::Zero();

    for (const auto& any : get_bodystate_map() ) {
        BodyState &state = *(any.second);
        total += state.get_linear_momentum();
    }

    return total;
}

Vector Engine::get_angular_momentum() const
{
    Vector total = Vector::Zero();
    for (const auto& any : get_bodystate_map()) {
        BodyState &state = *(any.second);
        total += state.get_angular_momentum_world_origin();
    }
    return total;
}

Real Engine::get_kinetic_energy() const
{
    Real total = 0;
    for (const auto& any : get_bodystate_map()) {
        BodyState &state = *(any.second);
        total += state.get_kinetic_energy();
    }
    return total;
}
