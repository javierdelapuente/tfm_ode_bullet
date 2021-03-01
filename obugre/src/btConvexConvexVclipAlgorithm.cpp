#include "btConvexConvexVclipAlgorithm.hpp"

#include "BulletCollision/NarrowPhaseCollision/btDiscreteCollisionDetectorInterface.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseInterface.h"
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"
#include "BulletCollision/CollisionShapes/btConvexShape.h"

#include "BulletCollision/CollisionShapes/btTriangleShape.h"

#include "BulletCollision/NarrowPhaseCollision/btGjkPairDetector.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h"
#include "BulletCollision/CollisionDispatch/btCollisionDispatcher.h"
#include "BulletCollision/CollisionShapes/btBoxShape.h"
#include "BulletCollision/CollisionDispatch/btManifoldResult.h"

#include "BulletCollision/NarrowPhaseCollision/btConvexPenetrationDepthSolver.h"
#include "BulletCollision/NarrowPhaseCollision/btContinuousConvexCollision.h"
#include "BulletCollision/NarrowPhaseCollision/btSubSimplexConvexCast.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkConvexCast.h"

#include "BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.h"
#include "BulletCollision/CollisionShapes/btSphereShape.h"

#include "BulletCollision/NarrowPhaseCollision/btMinkowskiPenetrationDepthSolver.h"

#include "BulletCollision/NarrowPhaseCollision/btGjkEpa2.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.h"
#include "BulletCollision/NarrowPhaseCollision/btPolyhedralContactClipping.h"
#include "BulletCollision/CollisionDispatch/btCollisionObjectWrapper.h"

#include "BulletCollision/NarrowPhaseCollision/btComputeGjkEpaPenetration.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkEpa3.h"
#include "BulletCollision/NarrowPhaseCollision/btMprPenetration.h"

#include <contacts.hpp>
#include "oblog.hpp"


btConvexConvexVclipAlgorithm::CreateFunc::CreateFunc(OB::VClipCache& cache,
                                                     const std::unordered_map<const btCollisionShape*, std::unique_ptr<OB::WorldConvexPolytope>>& shape_hashmap,
                                                     btScalar margin)
:cache(cache), shape_hashmap(shape_hashmap), margin(margin)
{
}

btConvexConvexVclipAlgorithm::CreateFunc::~CreateFunc()
{
}

btConvexConvexVclipAlgorithm::btConvexConvexVclipAlgorithm
(btPersistentManifold* mf,
 const btCollisionAlgorithmConstructionInfo& ci,
 const btCollisionObjectWrapper* body0Wrap,
 const btCollisionObjectWrapper* body1Wrap,
 OB::VClipCache &cache,
 const std::unordered_map<const btCollisionShape*, std::unique_ptr<OB::WorldConvexPolytope>>& shape_hashmap,
 btScalar margin)
	: btActivatingCollisionAlgorithm(ci, body0Wrap, body1Wrap),
	  m_ownManifold(false),
	  m_manifoldPtr(mf),
      cache(cache),
      shape_hashmap(shape_hashmap),
      margin(margin)
{
	(void)body0Wrap;
	(void)body1Wrap;
}


btConvexConvexVclipAlgorithm::~btConvexConvexVclipAlgorithm()
{
	if (m_ownManifold)
	{
		if (m_manifoldPtr)
			m_dispatcher->releaseManifold(m_manifoldPtr);
	}
}


void btConvexConvexVclipAlgorithm::processCollision(const btCollisionObjectWrapper* body0Wrap,
                                                    const btCollisionObjectWrapper* body1Wrap,
                                                    const btDispatcherInfo& dispatchInfo,
                                                    btManifoldResult* resultOut)
{
    //L_(OB::linfo) << "btConvexConvexVclipAlgorithm::processCollision";
	if (!m_manifoldPtr)
	{
		//swapped?
		m_manifoldPtr = m_dispatcher->getNewManifold(body0Wrap->getCollisionObject(), body1Wrap->getCollisionObject());
		m_ownManifold = true;
	}
	resultOut->setPersistentManifold(m_manifoldPtr);

	//const btConvexShape* min0 = static_cast<const btConvexShape*>(body0Wrap->getCollisionShape());
	//const btConvexShape* min1 = static_cast<const btConvexShape*>(body1Wrap->getCollisionShape());

    const btCollisionShape* min0 = body0Wrap->getCollisionShape();
    const btCollisionShape* min1 = body1Wrap->getCollisionShape();

    const btTransform& t0 = body0Wrap->getWorldTransform();
    const btTransform& t1 = body1Wrap->getWorldTransform();

    const auto& bulletwcp0 = shape_hashmap.at(min0);
    const auto& bulletwcp1 = shape_hashmap.at(min1);

    // I have to put this manually...
    bulletwcp0->set_pose(get_pose_from_Bullet(t0));
    bulletwcp1->set_pose(get_pose_from_Bullet(t1));

    OB::VClip vclip(*bulletwcp0, *bulletwcp1);
    // TODO disable cache not implemented
    OB::VClipResult result = vclip(cache);

    // L_(OB::linfo) << " Distance between features "
    //           << absdist_between_features(*bulletwcp0, result.witness.feature1,
    //                                      *bulletwcp1, result.witness.feature2);

    //L_(OB::linfo) << " object A " << bulletwcp0->get_name();
    //L_(OB::linfo) << " object B " << bulletwcp1->get_name();

    //if (result.penetrating()) { std::cout  << "bullet penetration" << std::endl; }

    if (result.distance < (margin + margin))
    {
        //L_(OB::linfo) << " VClip Result " << result;

        //if (result.penetrating) { L_(OB::linfo) << "bullet penetration"; }
        OB::ContactDataVClip cd = OB::get_simple_contact(*bulletwcp0,
                                                         *bulletwcp1,
                                                         result);
        // this is a trick. Actually, the depth in Bullet can be positive or negative.
        //cd.depth += margin;
        //L_(OB::linfo) << " vclip we have a contact. depth: " << cd.depth;

        // should we take the margin into account?
        cd.point_in_body_2 = cd.point_in_body_2 + (cd.normal * margin/2);

        btVector3 pointInWorld{cd.point_in_body_2[0],
                               cd.point_in_body_2[1],
                               cd.point_in_body_2[2]};
        btVector3 normalOnBInWorld{cd.normal[0], cd.normal[1], cd.normal[2]};
        // bullet uses depth negative for penetration, what a misnomer!!
        // https://pybullet.org/Bullet/phpBB3/viewtopic.php?t=7517
        btScalar depth{- (cd.depth + margin) };
        resultOut->addContactPoint(normalOnBInWorld, pointInWorld, depth);
    }


    if (m_ownManifold)
	{
		resultOut->refreshContactPoints();
	}
}


btScalar btConvexConvexVclipAlgorithm::calculateTimeOfImpact(btCollisionObject* col0,
                                                             btCollisionObject* col1,
                                                             const btDispatcherInfo& dispatchInfo,
                                                             btManifoldResult* resultOut)
{
	(void)resultOut;
	(void)dispatchInfo;
	btAssert(0);
	return 0;
}



OB::Transform get_pose_from_Bullet(const btTransform& bulletTransform)
{
    const btVector3& origin = bulletTransform.getOrigin();
    const btMatrix3x3& basis = bulletTransform.getBasis();

    const btVector3& row0 = basis.getRow(0);
    const btVector3& row1 = basis.getRow(1);
    const btVector3& row2 = basis.getRow(2);

    OB::Matrix3 rotmatrix;
    rotmatrix << row0[0], row0[1], row0[2],
        row1[0], row1[1], row1[2],
        row2[0], row2[1], row2[2];

    //initialization
    // OB::Transform transform;
    // transform = OB::Translation(origin.getX(), origin.getY(), origin.getZ());
    // return transform.rotate(rotmatrix);

    return OB::Translation(origin.getX(), origin.getY(), origin.getZ()) * rotmatrix;
}
