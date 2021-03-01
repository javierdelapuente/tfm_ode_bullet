#include <btBulletDynamicsCommon.h>
#include <BulletDynamics/ConstraintSolver/btNNCGConstraintSolver.h>
#include <BulletDynamics/ConstraintSolver/btNNCGConstraintSolver.h>
#include <BulletDynamics/MLCPSolvers/btMLCPSolver.h>
#include <BulletDynamics/MLCPSolvers/btDantzigSolver.h>
#include <BulletDynamics/MLCPSolvers/btSolveProjectedGaussSeidel.h>
#include <BulletDynamics/MLCPSolvers/btLemkeSolver.h>



#include "bullet_engine.hpp"
#include "oblog.hpp"

using namespace OB;


BulletEngine::BulletEngine(const std::string &name, System &system) : Engine(name, system)
{
    collisionConfiguration.reset(new btDefaultCollisionConfiguration());
    dispatcher.reset(new btCollisionDispatcher(collisionConfiguration.get()));
    overlappingPairCache.reset(new btDbvtBroadphase());
    solver.reset(new btSequentialImpulseConstraintSolver);
    //solver.reset(new btNNCGConstraintSolver);
    //solver.reset(new btMLCPSolver(new btSolveProjectedGaussSeidel));
    //solver.reset(new btMLCPSolver(new btLemkeSolver));
    //solver.reset(new btMLCPSolver(new btDantzigSolver));
    dynamicsWorld.reset(new btDiscreteDynamicsWorld(dispatcher.get(), overlappingPairCache.get(),
                                                    solver.get(), collisionConfiguration.get()));

    // btContactSolverInfo solverInfo = dynamicsWorld->getSolverInfo();
    // solverInfo.m_erp = 0.2;
    // solverInfo.m_globalCfm = 1e-5;

}

BulletEngine::~BulletEngine()
{
    //copied from "HelloWorld.cpp" example
    //remove the rigidbodies from the dynamics world and delete them
    L_(linfo) << "destroy BulletEngine";

    // we can think of more std::unique_ptr, but for now it is enough.
    for (int i = dynamicsWorld->getNumCollisionObjects() - 1; i >= 0; i--)
    {
        btCollisionObject* obj = dynamicsWorld->getCollisionObjectArray()[i];
        btRigidBody* body = btRigidBody::upcast(obj);
        //     if (body && body->getMotionState())
        //     {
        //         delete body->getMotionState();
        //     }
        dynamicsWorld->removeCollisionObject(obj);
        //     delete obj;
    }

}

void BulletEngine::step_engine(real_seconds timestep)
{
    //TODO CUIDADO, esto no es el timestep real.
    // La función es:
    //virtual int stepSimulation(btScalar timeStep,
    //                          int maxSubSteps = 1,
    //                          btScalar fixedTimeStep = btScalar(1.) / btScalar(60.)) = 0;

    dynamicsWorld->stepSimulation(timestep.count(), 1, timestep.count());
}



void BulletEngine::update_bodystates()
{
    for (auto const& pair : btRigidBodiesMap)
    {
        // first is string, second dBodyID
        btRigidBody* body = pair.second.get();
        if (body) {
            BodyState& bs = get_bodystate(pair.first);
            if (bs.get_body().is_static())
                continue;

            btTransform trans;
            trans = body->getCenterOfMassTransform();
            btVector3 btPos = trans.getOrigin();
            btQuaternion btRot = trans.getRotation();
            bs.set_position(Vector(btPos.getX(), btPos.getY(), btPos.getZ()));
            bs.set_orientation(Quaternion(btRot.getW(), btRot.getX(), btRot.getY(), btRot.getZ()));

            btVector3 btLinVel = body->getLinearVelocity();
            btVector3 btAngVel = body->getAngularVelocity();
            bs.set_linear_velocity(Vector(btLinVel.getX(), btLinVel.getY(), btLinVel.getZ()));
            bs.set_angular_velocity(Vector(btAngVel.getX(), btAngVel.getY(), btAngVel.getZ()));
        }
    }
}

void BulletEngine::set_gravity(Vector new_gravity)
{
    dynamicsWorld->setGravity(btVector3(new_gravity[0], new_gravity[1], new_gravity[2]));
}



void BulletEngine::set_use_vclip(bool vclip)
{
    Engine::set_use_vclip(vclip);
    // what about registerClosestPointsCreateFunc
    // void registerCollisionCreateFunc(int proxyType0, int proxyType1,
    //                                  btCollisionAlgorithmCreateFunc* createFunc);
    if (vclip)
    {
        create_func_vclip.reset
            (new btConvexConvexVclipAlgorithm::CreateFunc{vclip_cache,
                                                              shape_convex_hashmap,
                                                              get_margin()});
        btCollisionAlgorithmCreateFunc *cf = create_func_vclip.get();
		dispatcher->registerCollisionCreateFunc(CONVEX_HULL_SHAPE_PROXYTYPE,
                                                CONVEX_HULL_SHAPE_PROXYTYPE, cf);
    }
    else
    {
        btCollisionAlgorithmCreateFunc *cf =
            collisionConfiguration->getCollisionAlgorithmCreateFunc
            (CONVEX_HULL_SHAPE_PROXYTYPE, CONVEX_HULL_SHAPE_PROXYTYPE);

        dispatcher->registerCollisionCreateFunc(CONVEX_HULL_SHAPE_PROXYTYPE,
                                                CONVEX_HULL_SHAPE_PROXYTYPE, cf);
    }
}



void BulletEngine::add_body_internal(BodyState& state)
{
    const Body& body = state.get_body();
    btCollisionShape* collisionShape;
    add_shapes(collisionShape, body, state);

    btScalar mass = body.get_mass();
    Vector inertiaTensor = body.get_inertia_tensor();
    btVector3 inertia = btVector3(inertiaTensor[0],
                                  inertiaTensor[1],
                                  inertiaTensor[2]);
    btTransform bodyTransform;
    bodyTransform.setIdentity();
    const Point& pos = state.get_position();
    bodyTransform.setOrigin(btVector3(pos[0], pos[1], pos[2]));
    const Quaternion& ori = state.get_orientation();
    bodyTransform.setRotation(btQuaternion(ori.x(), ori.y(), ori.z(), ori.w()));

    motionStates.emplace_back(new btDefaultMotionState(bodyTransform));
    btDefaultMotionState* bodyState = motionStates.back().get(); //new btDefaultMotionState(bodyTransform);

    std::unique_ptr<btRigidBody> bulletBody =
        std::unique_ptr<btRigidBody> (new btRigidBody(mass, bodyState, collisionShape, inertia));

    const Vector& lvel = state.get_linear_velocity();
    bulletBody->setLinearVelocity(btVector3(lvel[0], lvel[1], lvel[2]));
    const Vector& avel = state.get_angular_velocity();
    bulletBody->setAngularVelocity(btVector3(avel[0], avel[1], avel[2]));


    bulletBody->setDamping(0,0);
    bulletBody->setRestitution(restitution);
    bulletBody->setFriction(friction);
    bulletBody->setContactStiffnessAndDamping(bulletBody->getContactStiffness(), 0.0);
    //bulletBody->setContactStiffnessAndDamping(10000000000000, 0.0);


    if (body.is_static())
    {
        bulletBody->setMassProps(0.0, inertia);
    }
    else
    {
        bulletBody->setFlags(BT_ENABLE_GYROPSCOPIC_FORCE);
        //bulletBody->setFlags(BT_ENABLE_GYROSCOPIC_FORCE_IMPLICIT_WORLD);
        //bulletBody->setActivationState(WANTS_DEACTIVATION);
        bulletBody->setActivationState(DISABLE_DEACTIVATION);
    }

    dynamicsWorld->addRigidBody(bulletBody.get());
    btRigidBodiesMap[body.get_name()] = std::move(bulletBody);
}

void BulletEngine::add_shapes(btCollisionShape*& collisionShape, const Body& body, BodyState& state)
{
    for (const auto &shape : body.get_shapes())
    {
        shape->accept_visitor(ShapeVisitorOperation::add, *this, state);
        //TODO, AGRUPAR TODAS EN UNA, ESTO SI HAY DOS NO FUNCIONA!!
        collisionShape = collisionShapes.back().get();
    }
}


void BulletEngine::visit_shape(ShapeVisitorOperation operation, BoxShape& shape,
                            BodyState& state)
{
    collisionShapes.push_back(std::unique_ptr<btCollisionShape>
                              (new btBoxShape(btVector3(shape.getExtentX() * 0.5,
                                                        shape.getExtentY()  * 0.5,
                                                        shape.getExtentZ()  * 0.5)
                                              )));
}

void BulletEngine::visit_shape(ShapeVisitorOperation operation, ConvexShape& convex,
                               BodyState& state)
{
    std::unique_ptr<btConvexHullShape> btchs = std::unique_ptr<btConvexHullShape>
        (new btConvexHullShape{});
    //btConvexHullShape;
    const std::shared_ptr<ConvexPolytope>& cp = convex.get_convex_polytope();

    for (const auto& vertex : cp->get_vertices()) {
        const Point &p = vertex.get_position();
        btchs->addPoint(btVector3{p[0], p[1], p[2]}, false);
    }
    //btchs->setMargin(get_margin()*0.5); // margin per body, not global
    btchs->setMargin(get_margin());
    btchs->recalcLocalAabb();

    std::ostringstream ossname;
    ossname << state.get_body().get_name();
    ossname << (void const *)btchs.get();
    shape_convex_hashmap.emplace(btchs.get(), new WorldConvexPolytope{ossname.str(), cp});

    collisionShapes.push_back(std::move(btchs));
}


void BulletEngine::visit_shape(ShapeVisitorOperation operation, PlaneShape& shape,
                            BodyState& state)
{
    const Plane& plane = shape.get_plane();
    collisionShapes.push_back(std::unique_ptr<btCollisionShape>
                              (new btStaticPlaneShape(btVector3(plane.normal()[0],
                                                                plane.normal()[1],
                                                                plane.normal()[2]),
                                                      plane.offset())));
}

void BulletEngine::visit_shape(ShapeVisitorOperation operation, SphereShape& shape,
                            BodyState& state)
{
    collisionShapes.push_back
        (std::unique_ptr<btCollisionShape>(new btSphereShape(shape.get_radius())));
}


void BulletEngine::add_constraint_internal(Constraint& constraint)
{
    constraint.accept_visitor(ConstraintVisitorOperation::add, *this);
}


void BulletEngine::visit_constraint(ConstraintVisitorOperation operation,
                                 HingeConstraint& hingeConstraint)
{

    btRigidBody* rba = btRigidBodiesMap[hingeConstraint.get_body1().get_name()].get();
    btRigidBody* rbb = btRigidBodiesMap[hingeConstraint.get_body2().get_name()].get();

    btVector3 axisA = btVector3(hingeConstraint.get_axis_local_body1()[0],
                                hingeConstraint.get_axis_local_body1()[1],
                                hingeConstraint.get_axis_local_body1()[2]);

    btVector3 pivotA = btVector3(hingeConstraint.get_point_local_body1()[0],
                                 hingeConstraint.get_point_local_body1()[1],
                                 hingeConstraint.get_point_local_body1()[2]);

    btVector3 axisB = btVector3(hingeConstraint.get_axis_local_body2()[0],
                                hingeConstraint.get_axis_local_body2()[1],
                                hingeConstraint.get_axis_local_body2()[2]);

    btVector3 pivotB = btVector3(hingeConstraint.get_point_local_body2()[0],
                                 hingeConstraint.get_point_local_body2()[1],
                                 hingeConstraint.get_point_local_body2()[2]);

    std::unique_ptr<btHingeConstraint> hinge;

    if (hingeConstraint.get_body1().is_static())
    {
        hinge = std::unique_ptr<btHingeConstraint>
            (new btHingeConstraint(*rbb,
                                   pivotB,
                                   axisB));
    }
    else if (hingeConstraint.get_body2().is_static())
    {
        hinge = std::unique_ptr<btHingeConstraint>
            (new btHingeConstraint(*rba,
                                   pivotA,
                                   axisA));
    }
    else
    {
        hinge = std::unique_ptr<btHingeConstraint>
            (new btHingeConstraint(*rba, *rbb,
                                   pivotA, pivotB,
                                   axisA, axisB));
    }

    // hinge->setParam(BT_CONSTRAINT_STOP_CFM, 0.0, -1);
    // hinge->setParam(BT_CONSTRAINT_STOP_ERP, 1.0, -1);

    dynamicsWorld->addConstraint(hinge.get());

    btConstraintMap[hingeConstraint.get_name()] = std::move(hinge);
}


std::vector<ContactPoint> BulletEngine::get_last_contact_points()
{
    std::vector<ContactPoint> ct;

    int numManifolds = dispatcher->getNumManifolds();
    for (int i = 0; i < numManifolds; i++)
    {
        btPersistentManifold* contactManifold = dispatcher->getManifoldByIndexInternal(i);
        int numContacts = contactManifold->getNumContacts();
        for (int j = 0; j < numContacts; j++)
        {
            btManifoldPoint& cp = contactManifold->getContactPoint(j);
            ContactPoint cd;
            cd.point = Vector{cp.m_positionWorldOnB.getX(),
                              cp.m_positionWorldOnB.getY(),
                              cp.m_positionWorldOnB.getZ()};
            cd.normal = Vector{cp.m_normalWorldOnB.getX(),
                               cp.m_normalWorldOnB.getY(),
                               cp.m_normalWorldOnB.getZ()};
            cd.distance = cp.getDistance();
            ct.push_back(cd);

        }
    }

    return ct;
}
