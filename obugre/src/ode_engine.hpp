#ifndef OBUGRE_ODEENGINEIMPL_HPP
#define OBUGRE_ODEENGINEIMPL_HPP

#include <string>
#include <vector>
#include <unordered_map>
#include <ode/ode.h>

#include <vclip.hpp>

#include "engine.hpp"
#include "oblog.hpp"

namespace OB
{

    // to have many instances of the ode engine
    class OdeInitializer
    {
    public:
        OdeInitializer() {
            dInitODE();
            L_(linfo) << " called dInitODE";
        }

        ~OdeInitializer()
        {
            dCloseODE();
            L_(linfo) << " called dCloseODE";
        }
    };

    class OdeEngine : public Engine
    {
    public:
        OdeEngine(const std::string &name, System &system);
        ~OdeEngine();

        OdeEngine(const OdeEngine&) = delete;
        OdeEngine& operator=(const OdeEngine&) = delete;

        void set_gravity(Vector gravity) override;

        void dNearCallback(void *data, dGeomID o1, dGeomID o2);
        int collideConvexConvex(dxGeom *o1, dxGeom *o2, int flags,
                                dContactGeom *contact, int skip);

        int dCollideConvexConvexVClip(dxGeom *o1, dxGeom *o2, int flags,
                                      dContactGeom *contact, int skip);

        std::vector<ContactPoint> get_last_contact_points() override;


        void set_friction(Real new_friction) { friction = new_friction;} ;
        void set_restitution(Real new_restitution) { restitution = new_restitution; };
        void set_quick_step(Real new_quick_step) { quick_step = new_quick_step; };

        std::chrono::microseconds get_convex_tests_duration() { return convex_collision_duration; }
        int get_total_convex_tests() { return total_convex_tests; }

    protected:
        void add_body_internal(BodyState& state) override;
        void step_engine(real_seconds timestep) override;
        void add_constraint_internal(Constraint& body) override;
        void update_bodystates() override;

        void add_shapes(dBodyID dBody, const Body& body, BodyState& situation);
        void add_static_shapes(const Body& body, BodyState& situation);

        void visit_shape(ShapeVisitorOperation operation, BoxShape& shape,
                         BodyState& state) override;
        void visit_shape(ShapeVisitorOperation operation, ConvexShape& shape,
                         BodyState& state) override;
        void visit_shape(ShapeVisitorOperation operation, PlaneShape& shape,
                         BodyState& state) override;
        void visit_shape(ShapeVisitorOperation operation, SphereShape& shape,
                         BodyState& state) override;

        void visit_constraint(ConstraintVisitorOperation operation,
                              HingeConstraint& hingeConstraint) override;


    private:
        std::shared_ptr<OdeInitializer> ode_initializer;
        dWorldID world;
        dSpaceID space;
        dJointGroupID contactgroup;

        std::map<std::string, dBodyID> dRigidBodiesMap;
        std::map<std::string, dJointID> dJointMap;

        bool collision_with_joint = true;
        bool quick_step = true;

        // this is because there no static body in ODE. We model a body
        // as a list of shapes.
        std::multimap<std::string, dGeomID> dBodyShapeMultiMap;

        std::unordered_map<dGeomID, std::unique_ptr<WorldConvexPolytope>> geom_convex_hashmap;
        VClipCache vclip_cache;

        dGeomID currentGeom;

        Real friction = 0;
        Real restitution = 1;

        std::chrono::microseconds convex_collision_duration{0};
        int total_convex_tests{0};
    };


    Transform get_pose_from_ODE(const dReal* position /*dVector3*/,
                                const dReal* rotation /*dVector3*/);

}

#endif
