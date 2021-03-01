#include <chrono>

#include "engine.hpp"
#include "oblog.hpp"
#include "ode_engine.hpp"

using namespace OB;

void EngineLogger::engine_pre_step(Engine &engine)
{
    Vector linMomentum = engine.get_linear_momentum();
    Vector angMomentum = engine.get_angular_momentum();
    Real kinEnergy = engine.get_kinetic_energy();
    Vector position{0,0,0};
    log_stream << static_cast<int>(LogType::engine) << ";"
               << engine.get_physics_time().count() << ";"
               << engine.get_name() << ";"
               << engine.get_used_time().count() << ";"
               << linMomentum[0] << ";"<< linMomentum[1] << ";"<< linMomentum[2] << ";"
               << angMomentum[0] << ";"<< angMomentum[1] << ";"<< angMomentum[2] << ";"
               << kinEnergy << ";"
               << position[0] << ";"<< position[1] << ";"<< position[2]
               << "\n";

    if (!logger_only_engine)
    {
        for (const auto& body : engine) {
            Vector linMomentum = body.get_linear_momentum();
            Vector angMomentum = body.get_angular_momentum();
            Vector position = body.get_position();
            Real kinEnergy = body.get_kinetic_energy();
            log_stream << static_cast<int>(LogType::body) << ";"
                       << engine.get_physics_time().count() << ";"
                       << engine.get_name() << ";"
                       << body.get_body().get_name() << ";"
                       << linMomentum[0] << ";"<< linMomentum[1] << ";"<< linMomentum[2] << ";"
                       << angMomentum[0] << ";"<< angMomentum[1] << ";"<< angMomentum[2] << ";"
                       << kinEnergy << ";"
                       << position[0] << ";"<< position[1] << ";"<< position[2]
                       << "\n";

        }
    }

    // for ode log convex time.
    if (logger_convex_time) {
        OdeEngine* ode_engine = dynamic_cast<OdeEngine*>(&engine);
        if (ode_engine)
        {
            log_stream << static_cast<int>(LogType::convex_test) << ";"
                       << engine.get_physics_time().count() << ";"
                       << engine.get_name() << ";"
                       << engine.get_used_time().count() << ";"
                       << ode_engine->get_convex_tests_duration().count() << ";"
                       << ode_engine->get_total_convex_tests()
                       << "\n";
        }
    }
}
