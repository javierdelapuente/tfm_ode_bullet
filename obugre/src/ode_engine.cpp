#include <sstream>
#include <chrono>

//ODE internal
#include <joints/contact.h>
#include <joints/joint.h>
#include <ode/odemath.h>
#include <collision_std.h>

#include <contacts.hpp>

#include "ode_engine.hpp"
#include "oblog.hpp"

//#include "ode/collision_std.h"
int dCollideConvexConvex (dxGeom *o1, dxGeom *o2, int flags,
                          dContactGeom *contact, int skip);


//# include "collision_libccd.h"
int dCollideConvexConvexCCD(dxGeom* o1, dxGeom* o2, int flags, dContactGeom* contact, int skip);


using namespace OB;

#undef min
#undef max

// to access ode matrixes
#define _I(i,j) I[(i)*4+(j)]

std::shared_ptr<OdeInitializer> global_ode;

OdeEngine::OdeEngine(const std::string &name, System& system)
    : Engine(name, system)
{
    // this is not thread safe.
    if (!global_ode)
        global_ode = std::make_shared<OdeInitializer>();
    ode_initializer = global_ode;
    L_(linfo) << "constructing OdeEngine";

    world = dWorldCreate();
    space = dHashSpaceCreate(0);
    //space = dSimpleSpaceCreate(0);
    contactgroup =
        dJointGroupCreate(0);

    dWorldSetCFM(world,1e-5);
    dWorldSetAutoDisableFlag(world,1);

    //dWorldSetLinearDamping(world, 0.00001);
    dWorldSetLinearDamping(world, 0.0);
    //dWorldSetAngularDamping(world, 0.005);
    dWorldSetAngularDamping(world, 0.0);
    //dWorldSetMaxAngularSpeed(world, 200);
    //dWorldSetMaxAngularSpeed(world, 0);

    //dWorldSetContactMaxCorrectingVel(world,0.1);
    //dWorldSetContactSurfaceLayer(world,0.001);
}


OdeEngine::~OdeEngine()
{
    // we should delete the joint groups first.
    // dSpaceDestroy destroys all geoms associated
    // dWorldDestroy destroys all bodies associated
    L_(linfo) << "destroy OdeEngine";

    dJointGroupEmpty (contactgroup);
    dJointGroupDestroy (contactgroup);
    dSpaceDestroy (space);
    dWorldDestroy (world);
}

OdeEngine *ODECallbackWrapperCurrentEngine;
void ODECallbackWrapper(void *data, dGeomID o1, dGeomID o2)
{
    ODECallbackWrapperCurrentEngine->dNearCallback(data, o1, o2);
}

int ODEdCollideConvexConvex (dxGeom *o1, dxGeom *o2, int flags,
                             dContactGeom *contact, int skip)
{

    return ODECallbackWrapperCurrentEngine->collideConvexConvex(o1, o2, flags,
                                                                contact, skip);
}

void OdeEngine::set_gravity(Vector new_gravity)
{
    dWorldSetGravity(world, new_gravity[0], new_gravity[1], new_gravity[2]);
}

void OdeEngine::step_engine(real_seconds timestep)
{
    ODECallbackWrapperCurrentEngine = this;
    dSetColliderOverride(dConvexClass, dConvexClass, &ODEdCollideConvexConvex);

    dJointGroupEmpty(contactgroup);

    dSpaceCollide (space,0, &ODECallbackWrapper);

    if (quick_step)
    {
        dWorldQuickStep(world, timestep.count());
    }
    else
    {
        dWorldStep(world, timestep.count());
    }
}


void OdeEngine::update_bodystates()
{
    //TODO STEP AND UPDATE BODY POSITIONS AND ORIENTATIONS IN BODYSITUATIONS.
    for (auto const& pair : dRigidBodiesMap)
    {
        // first is string, second dBodyID
        dBodyID oBody = pair.second;
        if (oBody) {
            const dReal *pos = dBodyGetPosition(oBody);
            const dReal *ori = dBodyGetQuaternion (oBody);

            BodyState& bs = get_bodystate(pair.first);
            bs.set_position(Point(pos[0], pos[1], pos[2]));
            bs.set_orientation(Quaternion(ori[0], ori[1], ori[2], ori[3]));

            const dReal *lvel = dBodyGetLinearVel(oBody);
            const dReal *lang = dBodyGetAngularVel(oBody);
            bs.set_linear_velocity(Vector(lvel[0], lvel[1], lvel[2]));
            bs.set_angular_velocity(Vector(lang[0], lang[1], lang[2]));
        }
    }
}


void OdeEngine::add_body_internal(BodyState& body_state)
{
    const Body& body = body_state.get_body();

    if (body.is_static()) {
        add_static_shapes(body, body_state);
    } else {
        const Point& pos = body_state.get_position();
        const Quaternion& ori = body_state.get_orientation();
        dQuaternion dq = {ori.w(), ori.x(), ori.y(), ori.z()};
        dBodyID dBody = dBodyCreate(world);
        dBodySetPosition (dBody, pos[0],pos[1], pos[2]);
        dBodySetQuaternion (dBody, dq);

        const Vector& lVel = body_state.get_linear_velocity();
        dBodySetLinearVel(dBody, lVel[0], lVel[1], lVel[2]);
        const Vector& aVel = body_state.get_angular_velocity();
        dBodySetAngularVel(dBody, aVel[0], aVel[1], aVel[2]);

        dMass mass;
        dMassSetZero (&mass);
        mass.mass = body.get_mass();
        Vector inertiaTensor = body.get_inertia_tensor();
        mass._I(0,0) = inertiaTensor[0];
        mass._I(1,1) = inertiaTensor[1];
        mass._I(2,2) = inertiaTensor[2];
        dBodySetMass(dBody, &mass);

        dRigidBodiesMap[body.get_name()] = dBody;
        add_shapes(dBody, body, body_state);
    }
}


void OdeEngine::add_shapes(dBodyID dBody, const Body& body, BodyState& state)
{
    for (const auto &shape : body.get_shapes())
    {
        shape->accept_visitor(ShapeVisitorOperation::add, *this, state);
        dGeomSetBody(currentGeom, dBody);
    }
}

void OdeEngine::add_static_shapes(const Body& body, BodyState& state )
{
    for (const auto &shape : body.get_shapes())
    {
        shape->accept_visitor(ShapeVisitorOperation::add, *this, state);
        // grrr
        // plane is non-placeable, breaks everything...
        const PlaneShape* isPlane = dynamic_cast<const PlaneShape*>(shape.get());
        if (!isPlane) {
            //TODO THIS IS WRONG WE SHOULD COMBINE THIS TRANSFORMATION WITH
            //THE ORIGINAL ONE IN THE SHAPE.
            Point pos = state.get_position();
            const Quaternion& ori = state.get_orientation();
            dQuaternion dq = {ori.w(), ori.x(), ori.y(), ori.z()};

            dGeomSetPosition(currentGeom, pos[0],pos[1], pos[2]);
            dGeomSetQuaternion(currentGeom, dq);
        }
        //TODO add shape to list of shapes so we can delete them.
    }
}


void OdeEngine::visit_shape(ShapeVisitorOperation operation, BoxShape& shape,
                            BodyState& state)
{
    currentGeom = dCreateBox (space,
                              shape.getExtentX(),
                              shape.getExtentY(),
                              shape.getExtentZ());
}

void OdeEngine::visit_shape(ShapeVisitorOperation operation, ConvexShape& convex,
                            BodyState& state)
{
    const std::shared_ptr<ConvexPolytope>& cp = convex.get_convex_polytope();
    convex.generate_Ode_data();
    currentGeom = dCreateConvex (space,
                                 convex.get_Ode_planes(),
                                 convex.get_Ode_plane_count(),
                                 convex.get_Ode_points(),
                                 convex.get_Ode_point_count(),
                                 convex.get_Ode_polygons());
    // // I don't know if this could leak on failure, but I don't care. This is for vclip
    std::ostringstream ossname;
    ossname << state.get_body().get_name();
    ossname << (void const *)currentGeom;
    geom_convex_hashmap.emplace(currentGeom, new WorldConvexPolytope{ossname.str(), cp});
}


void OdeEngine::visit_shape(ShapeVisitorOperation operation, PlaneShape& shape,
                            BodyState& state)
{
    //TODO, COMBINE BODY POSITION WITH SHAPE POSITION

    // non placeable geom.
    Plane plane = shape.get_plane();
    OB::Transform t1;
    Vector position = state.get_position();
    position *= -1; // GRRR
    t1 = OB::Translation(position) * state.get_orientation();
    // we need to update transformations here
    plane.transform(t1);
    L_(linfo) << "OdeEngine::visitShape Plane normal: " << plane.normal().transpose()
              << " offset: " << plane.offset();

    currentGeom = dCreatePlane (space, plane.normal()[0], plane.normal()[1], plane.normal()[2],
                                plane.offset());
}

void OdeEngine::visit_shape(ShapeVisitorOperation operation, SphereShape& shape,
                            BodyState& state)
{
    currentGeom = dCreateSphere (space, shape.get_radius());
}


void OdeEngine::dNearCallback(void *data, dGeomID o1, dGeomID o2)
{
    //L_(linfo) << "ODEEngine::dNearCallback";

    const int MAX_CONTACTS = 100;
    dContact contacts[MAX_CONTACTS];
    // Fill this if necessary..... better after dCollide

    //trying it to be an elastic collision
    for (int i=0; i<MAX_CONTACTS; i++) {
        // contacts[i].surface.mode = dContactBounce;// | dContactSoftCFM;
        // //contacts[i].surface.mu = dInfinity;
        //   contacts[i].surface.mu = 0;
        //   contacts[i].surface.mu2 = 0;
        //   contacts[i].surface.bounce = 1;
        //   contacts[i].surface.bounce_vel = 0.0;
        //   //contacts[i].surface.soft_cfm = 0.01;
        //   //...

        contacts[i].surface.mode = dContactBounce | dContactApprox1;
        //contacts[i].surface.mu = dInfinity;
        contacts[i].surface.mu = friction;
        //contacts[i].surface.mu = 0;
        //contacts[i].surface.mu2 = dInfinity;
        contacts[i].surface.mu2 = friction;
        //contacts[i].surface.bounce = 1;
        contacts[i].surface.bounce = restitution;
        contacts[i].surface.bounce_vel = 0.0;
        //contacts[i].surface.soft_cfm = 0.01;
        //...

        // contacts[i].surface.mode += dContactRolling;
        // contacts[i].surface.rho = friction;
        // contacts[i].surface.rho2 = friction;
        //contacts[i].surface.rhoN = friction;

        contacts[i].surface.mode += dContactSoftERP | dContactSoftCFM;
        contacts[i].surface.soft_erp = 0.2; //0.2;
        contacts[i].surface.soft_cfm = 1e-5; //1e-5;

    }

    dBodyID b1 = dGeomGetBody(o1);
    dBodyID b2 = dGeomGetBody(o2);

    if (!collision_with_joint && b1 && b2 && dAreConnectedExcluding(b1,b2,dJointTypeContact))
        return;

    //example from whe ODE wiki

    if (dGeomIsSpace (o1) || dGeomIsSpace (o2)) {
        // colliding a space with something :
        dSpaceCollide2 (o1, o2, data, &ODECallbackWrapper);
        // collide all geoms internal to the space(s)
        if (dGeomIsSpace (o1))
            dSpaceCollide ((dSpaceID)o1, data, &ODECallbackWrapper);
        if (dGeomIsSpace (o2))
            dSpaceCollide ((dSpaceID)o2, data, &ODECallbackWrapper);
    } else {
        // colliding two non-space geoms, so generate contact
        // points between o1 and o2
        int num_contact = dCollide (o1, o2, MAX_CONTACTS, &contacts[0].geom,  sizeof(dContact));
        // add these contact points to the simulation ...
        for (int i = 0; i < num_contact; i++) {
            // L_(linfo) << "ode contact " << i
            //           << " normal: " << contacts[i].geom.normal[0] << " "
            //           << contacts[i].geom.normal[1] << " "  << contacts[i].geom.normal[2];
            // L_(linfo) << "ode contact " << i
            //           << " pos "
            //           << contacts[i].geom.pos[0] << " " << contacts[i].geom.pos[1]
            //           << " " << contacts[i].geom.pos[2]
            //           << " depth "  << contacts[i].geom.depth;
            // L_(linfo) << "ode contact " << i
            //           << " geom1 class " << dGeomGetClass(contacts[i].geom.g1)
            //           << " geom2 class " << dGeomGetClass(contacts[i].geom.g2);
            dJointID c = dJointCreateContact (world, contactgroup, contacts+i);
            dJointAttach (c, b1, b2);
        }
    }
}


const int numc_mask = (0xffff);
static inline
dContactGeom* SAFECONTACT(int Flags, dContactGeom* Contacts, int Index, int Stride){
    //dIASSERT(Index >= 0 && Index < (Flags & numc_mask));
    return ((dContactGeom*)(((char*)Contacts) + (Index * Stride)));
}

int OdeEngine::collideConvexConvex(dxGeom *o1, dxGeom *o2, int flags,
                                   dContactGeom *contact, int skip)
{
    std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
    int resp;
    if (is_use_vclip()) {
        dxConvex::margin = get_margin() * 0.5; // half for each object
        resp =  dCollideConvexConvexVClip (o1, o2, flags, contact, skip);
    }
    else
    {
        dxConvex::margin = 0.0;
        resp = dCollideConvexConvexCCD (o1, o2, flags, contact, skip);
        //TODO THIS IS SAT ALGORITHM. CHECK IT
        //resp = dCollideConvexConvex (o1, o2, flags, contact, skip);
    }
    std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
    auto int_us = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1);
    convex_collision_duration += int_us;
    auto integral_duration = static_cast<uint64_t>(int_us.count());
    ++total_convex_tests;
    L_(ldebug4) << "dCollideConvexConvex " << get_name()
                <<  ",time: " << integral_duration
                << " ,total " << convex_collision_duration.count()
                << " ,numtests " << total_convex_tests
                << " ,isvclip " << is_use_vclip()
                << " ,contacts " << resp;
    return resp;
}

int OdeEngine::dCollideConvexConvexVClip (dxGeom *o1, dxGeom *o2, int flags,
                                          dContactGeom *contact, int skip)
{
    int maxc = flags & numc_mask; // max collisions

    int contacts = 0;

    WorldConvexPolytope& wcpa = *geom_convex_hashmap[o1];
    WorldConvexPolytope& wcpb = *geom_convex_hashmap[o2];

    wcpa.set_pose(get_pose_from_ODE(dGeomGetPosition(o1), dGeomGetRotation(o1)));
    wcpb.set_pose(get_pose_from_ODE(dGeomGetPosition(o2), dGeomGetRotation(o2)));

    //std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
    VClip vclip(wcpa, wcpb);

    VClipWitness witness{VClipWitness::FirstVertices()};
    VClipResult result = is_disable_vclip_cache() ? vclip.solve(witness) : vclip(vclip_cache);
    //std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
    //auto int_us = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1);
    //auto integral_duration = static_cast<uint64_t>(int_us.count());
    //L_(linfo) << "VClip time us: " << integral_duration;

    //L_(ldebug3) << " VClip result " << result;
    //vclipCache

    // L_(ldebug3) << " Distance between features "
    //           << absdist_between_features(*geom_convex_hashmap[o1], result.witness.feature1,
    //                                      *geom_convex_hashmap[o2], result.witness.feature2);

    //L_(linfo) << "penetrating " << result.penetrating << " distance " << result.distance;

    if (result.distance > get_margin()) {
        return 0;
    }

    //if (result.penetrating()) { L_(linfo) << "ode penetration"; }

    // if (!result.penetrating())
    // {
        //L_(OB::linfo) << " VClip Result " << result;
        ContactDataVClip cd = get_simple_contact(wcpa, wcpb, result);
        dContactGeom *target = SAFECONTACT(flags, contact, contacts, skip);
        target->pos[0] = cd.point[0];
        target->pos[1] = cd.point[1];
        target->pos[2] = cd.point[2];
        // always positive or does not work well...
        target->depth = cd.depth + get_margin();
        //L_(OB::linfo) << " depth to ODE " << target->depth;
        target->normal[0] = cd.normal[0];
        target->normal[1] = cd.normal[1];
        target->normal[2] = cd.normal[2];
        target->g1 = o1;
        target->g2 = o2;
        contacts++;
        return contacts;
    // }
    // else
    // {
    //     L_(linfo) << "penetration";
    //     std::vector<ContactDataVClip> cds = get_contacts_envelope(wcpa,
    //                                                               wcpb,
    //                                                               result,
    //                                                               margin);
    //     L_(linfo) << "penetration num contacts: " << cds.size();
    //     for (ContactDataVClip cd : cds)
    //     {
    //         dContactGeom *target = SAFECONTACT(flags, contact, contacts, skip);
    //         target->pos[0] = cd.point[0];
    //         target->pos[1] = cd.point[1];
    //         target->pos[2] = cd.point[2];
    //         target->normal[0] = cd.normal[0];
    //         target->normal[1] = cd.normal[1];
    //         target->normal[2] = cd.normal[2];
    //         target->g1 = o1;
    //         target->g2 = o2;
    //         // always positive or does not work well...
    //         target->depth = cd.depth + margin + 0.05;
    //         contacts++;
    //         if (contacts == maxc)
    //         {
    //             break;
    //         }
    //     }
    //     return contacts;
    // }

}

std::vector<ContactPoint> OdeEngine::get_last_contact_points()
{
    // we have to access internal ode data :(
    std::vector<ContactPoint> ct;

    sizeint joint_bytes;
    for (dxJoint *j = (dxJoint *)contactgroup->beginEnum();
         j != NULL;
         j = (dxJoint *)contactgroup->continueEnum(joint_bytes)) {
        joint_bytes = j->size();
        dContact& contact = ((dxJointContact*)j)->contact;
        ContactPoint cd;
        cd.point = Vector
            {static_cast<Real>(contact.geom.pos[0]),
             static_cast<Real>(contact.geom.pos[1]),
             static_cast<Real>(contact.geom.pos[2])};
        cd.normal = Vector
            {static_cast<Real>(contact.geom.normal[0]),
             static_cast<Real>(contact.geom.normal[1]),
             static_cast<Real>(contact.geom.normal[2])};
        cd.distance = contact.geom.depth;
        ct.push_back(cd);
    }
    return ct;
}

void OdeEngine::add_constraint_internal(Constraint& constraint)
{
    constraint.accept_visitor(ConstraintVisitorOperation::add, *this);
}

void OdeEngine::visit_constraint(ConstraintVisitorOperation operation,
                                 HingeConstraint& hingeConstraint)
{
    // this is not the totally correct way of doing it. body1 and body2
    // could be incorrectly positioned. They should be placed correctly
    // for the constraint and placed back to their original position.
    dJointID odeHinge = dJointCreateHinge (world, 0);
    dJointMap[hingeConstraint.get_name()] = odeHinge;

    Transform toWorldFromBody = get_bodystate(hingeConstraint.get_body1().get_name()).get_pose();
    Point worldAnchor = toWorldFromBody * hingeConstraint.get_point_local_body1();
    Vector worldAxis = toWorldFromBody.linear() * hingeConstraint.get_axis_local_body1();

    dBodyID dbody1 = 0;
    dBodyID dbody2 = 0;

    if (!hingeConstraint.get_body1().is_static()) {
        dbody1 = dRigidBodiesMap[hingeConstraint.get_body1().get_name()];;
    }

    if (!hingeConstraint.get_body2().is_static()) {
        dbody2 = dRigidBodiesMap[hingeConstraint.get_body2().get_name()];
    }

    dJointAttach (odeHinge,
                  dbody1,
                  dbody2);
    dJointSetHingeAnchor (odeHinge, worldAnchor[0], worldAnchor[1], worldAnchor[2]);
    dJointSetHingeAxis (odeHinge, worldAxis[0], worldAxis[1], worldAxis[2]);

}

Transform OB::get_pose_from_ODE(const dReal * position /*dVector3*/,
                                const dReal * rotation /*dVector3*/)
{
    OB::Matrix3 rotmatrix;
    rotmatrix << rotation[0], rotation[1], rotation[2],
        rotation[4], rotation[5], rotation[6],
        rotation[8], rotation[9], rotation[10];

    // this is a bit verbose and inefficient. Don't know
    // how to do it better.
    Transform transform;
    transform = OB::Translation(position[0], position[1], position[2]);
    transform.rotate(rotmatrix);
    return transform;
}
