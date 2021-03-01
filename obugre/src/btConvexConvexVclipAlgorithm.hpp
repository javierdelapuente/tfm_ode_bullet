#ifndef BT_CONVEX_CONVEX_VCLIP_ALGORITHM_H
#define BT_CONVEX_CONVEX_VCLIP_ALGORITHM_H


#include "BulletCollision/CollisionDispatch/btActivatingCollisionAlgorithm.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkPairDetector.h"
#include "BulletCollision/NarrowPhaseCollision/btPersistentManifold.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h"
#include "BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.h"
#include "BulletCollision/CollisionDispatch/btCollisionCreateFunc.h"
#include "BulletCollision/CollisionDispatch/btCollisionDispatcher.h"
#include "LinearMath/btTransformUtil.h"  //for btConvexSeparatingDistanceUtil
#include "vclip.hpp"

// based on btConvexConvexMprAlgorithm
class btConvexConvexVclipAlgorithm : public btActivatingCollisionAlgorithm
{
    bool m_ownManifold;
    btPersistentManifold* m_manifoldPtr;
    ///cache separating vector to speedup collision detection

    OB::VClipCache &cache;
    const std::unordered_map<const btCollisionShape*, std::unique_ptr<OB::WorldConvexPolytope>>& shape_hashmap;
    btScalar margin;
public:
    btConvexConvexVclipAlgorithm(btPersistentManifold* mf,
                                 const btCollisionAlgorithmConstructionInfo& ci,
                                 const btCollisionObjectWrapper* body0Wrap,
                                 const btCollisionObjectWrapper* body1Wrap,
                                 OB::VClipCache &cache,
                                 const std::unordered_map<const btCollisionShape*, std::unique_ptr<OB::WorldConvexPolytope>>& shape_hashmap,
                                 btScalar margin);

    virtual ~btConvexConvexVclipAlgorithm();

    virtual void processCollision(const btCollisionObjectWrapper* body0Wrap,
                                  const btCollisionObjectWrapper* body1Wrap,
                                  const btDispatcherInfo& dispatchInfo,
                                  btManifoldResult* resultOut);

    virtual btScalar calculateTimeOfImpact(btCollisionObject* body0, btCollisionObject* body1,
                                           const btDispatcherInfo& dispatchInfo,
                                           btManifoldResult* resultOut);

    virtual void getAllContactManifolds(btManifoldArray& manifoldArray)
    {
        ///should we use m_ownManifold to avoid adding duplicates?
        if (m_manifoldPtr && m_ownManifold)
            manifoldArray.push_back(m_manifoldPtr);
    }

    const btPersistentManifold* getManifold()
    {
        return m_manifoldPtr;
    }

    struct CreateFunc : public btCollisionAlgorithmCreateFunc
    {
        CreateFunc(OB::VClipCache& cache,
                   const std::unordered_map<const btCollisionShape*,
                   std::unique_ptr<OB::WorldConvexPolytope>>& shape_hashmap,
                   btScalar margin);

        virtual ~CreateFunc();

        virtual btCollisionAlgorithm* CreateCollisionAlgorithm
        (btCollisionAlgorithmConstructionInfo& ci,
         const btCollisionObjectWrapper* body0Wrap,
         const btCollisionObjectWrapper* body1Wrap)
        {
            void* mem =
                ci.m_dispatcher1->allocateCollisionAlgorithm(sizeof(btConvexConvexVclipAlgorithm));
            return new (mem) btConvexConvexVclipAlgorithm(ci.m_manifold, ci,
                                                          body0Wrap, body1Wrap,
                                                          cache,
                                                          shape_hashmap,
                                                          margin);
        }
    private:
        OB::VClipCache& cache;
        const std::unordered_map<const btCollisionShape*,
                                 std::unique_ptr<OB::WorldConvexPolytope>>& shape_hashmap;
        btScalar margin;
    };
};

OB::Transform get_pose_from_Bullet(const btTransform& bulletTransform);

#endif
