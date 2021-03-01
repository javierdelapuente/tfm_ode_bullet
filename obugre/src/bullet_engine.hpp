#ifndef OBUGRE_BULLETENGINEIMPL_HPP
#define OBUGRE_BULLETENGINEIMPL_HPP

#include <string>
#include <vector>
#include <unordered_map>
#include <btBulletDynamicsCommon.h>

#include "engine.hpp"
#include "oblog.hpp"
#include "vclip.hpp"
#include "btConvexConvexVclipAlgorithm.hpp"

class btDefaultCollisionConfiguration;
class btCollisionDispatcher;
class btBroadphaseInterface;
class btSequentialImpulseConstraintSolver;
class btDiscreteDynamicsWorld;
class btCollisionShape;
class btRigidBody;
struct btDefaultMotionState;

namespace OB
{

    class BulletEngine: public Engine
    {
    public:
        BulletEngine(const std::string &name, System &system);
        ~BulletEngine() override;

        BulletEngine(const BulletEngine&) = delete;
        BulletEngine& operator=(const BulletEngine&) = delete;


        void set_gravity(Vector gravity) override;
        std::vector<ContactPoint> get_last_contact_points() override;

        // for the newly created bodies.
        void set_friction(Real new_friction) {friction = new_friction;};
        void set_restitution(Real new_restitution) {restitution = new_restitution;};
        void set_use_vclip(bool vclip) override;
    protected:
        void add_body_internal(BodyState& state) override;
        void add_constraint_internal(Constraint& constraint) override;
        void step_engine(real_seconds timestep) override;
        void update_bodystates() override;

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

        void add_shapes(btCollisionShape*& collisionShape, const Body& body, BodyState& state);
    private:
        std::unique_ptr<btDefaultCollisionConfiguration> collisionConfiguration;
        std::unique_ptr<btCollisionDispatcher> dispatcher;
        std::unique_ptr<btBroadphaseInterface> overlappingPairCache;
        std::unique_ptr<btSequentialImpulseConstraintSolver> solver;
        std::unique_ptr<btDiscreteDynamicsWorld> dynamicsWorld;

        std::map<std::string, std::unique_ptr<btRigidBody>> btRigidBodiesMap;
        std::map<std::string, std::unique_ptr<btTypedConstraint>> btConstraintMap;

        std::vector<std::unique_ptr<btDefaultMotionState>> motionStates;
        std::vector<std::unique_ptr<btCollisionShape>> collisionShapes;

        std::unordered_map<const btCollisionShape*,
                           std::unique_ptr<WorldConvexPolytope>> shape_convex_hashmap;

        Real friction = 0;
        Real restitution = 1;

        std::unique_ptr<btConvexConvexVclipAlgorithm::CreateFunc> create_func_vclip;
        VClipCache vclip_cache;
    };


}

#endif
