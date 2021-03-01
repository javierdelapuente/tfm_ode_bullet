
// ideas from from http://wiki.ogre3d.org/Skeleton+Debugger

#include "body_debug.hpp"
#include "oblog.hpp"
#include "renderer.hpp"

using namespace OB;
using namespace Ogre;

BodyDebug::BodyDebug(Renderer& renderer,
                     Ogre::SceneNode *bodySceneNode,
                     Ogre::SceneManager *sceneManager,
                     Ogre::Camera *camera,
                     Engine &engine,
                     BodyState* bodyState) : renderer(renderer), engine(engine)
{
    mBaseNode = nullptr;
    mDebugNode = nullptr;
    mBodyState = bodyState;
    mBodySceneNode = bodySceneNode;
    mSceneManager = sceneManager;
    mCamera = camera;
    createAxesMaterial();
    createAxesMesh();
    createArrowMesh();

    // base axes
    mBaseNode = mBodySceneNode->createChildSceneNode();
    mAxisNode = mBaseNode->createChildSceneNode();
    Ogre::Entity *entityAxes = mSceneManager->createEntity(mAxesMeshPtr);
    mAxisNode->attachObject(entityAxes);

    mDebugNode = mBaseNode->createChildSceneNode();
    //mDebugNode->setScale(2, 2, 2);


    mLinearMomentumNode = mDebugNode->createChildSceneNode();
    Ogre::Entity *entityLm = mSceneManager->createEntity(mArrowMeshPtr);
    mLinearMomentumNode->attachObject(entityLm);
    entityLm->setMaterialName("Obugre/arrow_yellow");
    //mLinearMomentumNode->setVisible(false);

    mAngularMomentumNode = mDebugNode->createChildSceneNode();
    Ogre::Entity *entityAngMom = mSceneManager->createEntity(mArrowMeshPtr);
    mAngularMomentumNode->attachObject(entityAngMom);
    entityAngMom->setMaterialName("Obugre/arrow_magenta");
    //mAngularMomentumNode->setVisible(false);

    if (bodyState)
    {
        mAngularVelocityNode = mDebugNode->createChildSceneNode();
        Ogre::Entity *entityAngVel = mSceneManager->createEntity(mArrowMeshPtr);
        mAngularVelocityNode->attachObject(entityAngVel);
        entityAngVel->setMaterialName("Obugre/arrow_cyan");
    }

    mTextDisplay = std::unique_ptr<ObjectTextDisplay>(new ObjectTextDisplay(entityAxes, mCamera));
    mTextDisplay->enable(true);

    mCamera->addListener(this);
}

BodyDebug::~BodyDebug()
{
    mCamera->removeListener(this);
}


void BodyDebug::update()
{
    bool show_axis = renderer.get_show_body_debug_axis();
    bool show_vectors = renderer.get_show_body_debug_vectors();
    bool show_text = renderer.get_show_body_debug_text();
    if (!mBodyState)
    {
        show_axis = renderer.get_show_engine_debug_axis();
        show_vectors = renderer.get_show_engine_debug_vectors();
        show_text = renderer.get_show_engine_debug_text();
    }

    if (!(show_axis || show_vectors || show_text))
    {
        mBaseNode->setVisible(false);
        mTextDisplay->enable(false);
        return;
    }
    mBaseNode->setVisible(true);

    mTextDisplay->enable(show_text);
    mAxisNode->setVisible(show_axis);
    mDebugNode->setVisible(show_vectors);


    Real kineticEnergy = mBodyState ? mBodyState->get_kinetic_energy() : engine.get_kinetic_energy();
    std::string text{};
    if (mBodyState)
    {
        text += "Body: " + mBodyState->get_body().get_name() + "\n";
    }
    else
    {
        text += "Engine: " + engine.get_name() + "\n";
    }
    text += "Kinetic Energy: " + std::to_string(kineticEnergy);

    {
        // update linear momentum
        Vector linMom = mBodyState ? mBodyState->get_linear_momentum() : engine.get_linear_momentum();
        Quaternion orientation = mBodyState ? mBodyState->get_orientation() : Quaternion::Identity();

        // get local linear momentum of the body.
        Quaternion linMomq{0,0,0,0};
        linMomq.vec() = linMom;

        Quaternion linMomLocalq = orientation.inverse() * linMomq * orientation;
        Vector linMomLocal = linMomLocalq.vec();
        Ogre::Vector3 lmlocal = Ogre::Vector3(linMomLocal[0], linMomLocal[1], linMomLocal[2]);
        Ogre::Vector3 lm = Ogre::Vector3(linMom[0], linMom[1], linMom[2]);

        Ogre::Quaternion q  = mArrowOrientation.getRotationTo(lmlocal);
        mLinearMomentumNode->setOrientation(q);
        mLinearMomentumNode->setScale(lm.length(), 1, 1);
        //spdlog::info("BodyDebug::update linear momentum {} local {}", lm, lmlocal);
        text += "\nLinear Momentum norm: " + std::to_string(linMom.norm());
    }

    // update angular velocity
    if (mBodyState) {
        Vector angVel = mBodyState->get_angular_velocity();
        Quaternion orientation = mBodyState ? mBodyState->get_orientation() : Quaternion::Identity();
        Quaternion angVelq{0,0,0,0};
        angVelq.vec() = angVel;

        Quaternion angVelLocalq = orientation.inverse() * angVelq * orientation;
        Vector angVelLocal = angVelLocalq.vec();

        Ogre::Vector3 av = Ogre::Vector3(angVel[0], angVel[1], angVel[2]);
        Ogre::Vector3 avlocal = Ogre::Vector3(angVelLocal[0], angVelLocal[1], angVelLocal[2]);
        //Ogre::Vector3 avlocal = av;
        Ogre::Quaternion q = mArrowOrientation.getRotationTo(avlocal);
        mAngularVelocityNode->setOrientation(q);
        mAngularVelocityNode->setScale(av.length(), 1, 1);
    }

    {
        // update angular momentum
        Quaternion orientation = mBodyState ? mBodyState->get_orientation() : Quaternion::Identity();
        Vector angMom = mBodyState ? mBodyState->get_angular_momentum() : engine.get_angular_momentum();
        Quaternion angMomq{0,0,0,0};
        angMomq.vec() = angMom;

        Quaternion angMomLocalq = orientation.inverse() * angMomq * orientation;
        Vector angMomLocal = angMomLocalq.vec();

        //L_(linfo) << " Angular momentum " << angMom.transpose();
        Ogre::Vector3 am = Ogre::Vector3(angMom[0], angMom[1], angMom[2]);
        Ogre::Vector3 amlocal = Ogre::Vector3(angMomLocal[0], angMomLocal[1], angMomLocal[2]);
        Ogre::Quaternion q = mArrowOrientation.getRotationTo(amlocal);
        mAngularMomentumNode->setOrientation(q);
        mAngularMomentumNode->setScale(am.length(), 1, 1);
        text += "\nAngular Momentum norm: " + std::to_string(angMom.norm());
    }


    if (show_text)
    {
        mTextDisplay->setText(text);
        mTextDisplay->update();
    }
}


void BodyDebug::createAxesMaterial()
{
    //spdlog::info("BodyDebug::createAxesMaterial");
    Ogre::String matName = "SkeletonDebug/AxesMat";

    mAxisMatPtr = MaterialManager::getSingleton().getByName(matName);
    if (!mAxisMatPtr)
    {
        mAxisMatPtr = MaterialManager::getSingleton().create(matName, ResourceGroupManager::INTERNAL_RESOURCE_GROUP_NAME);

        // First pass for axes that are partially within the model (shows transparency)
        Pass* p = mAxisMatPtr->getTechnique(0)->getPass(0);
        p->setLightingEnabled(false);
        p->setPolygonModeOverrideable(false);
        p->setVertexColourTracking(TVC_AMBIENT);
        p->setSceneBlending(SBT_TRANSPARENT_ALPHA);
        p->setCullingMode(CULL_NONE);
        p->setDepthWriteEnabled(false);
        p->setDepthCheckEnabled(false);

        // Second pass for the portion of the axis that is outside the model (solid colour)
        Pass* p2 = mAxisMatPtr->getTechnique(0)->createPass();
        p2->setLightingEnabled(false);
        p2->setPolygonModeOverrideable(false);
        p2->setVertexColourTracking(TVC_AMBIENT);
        p2->setCullingMode(CULL_NONE);
        p2->setDepthWriteEnabled(false);
        //spdlog::info("material created.");
    }

}



void BodyDebug::createAxesMesh()
{
    //spdlog::info("BodyDebug::createAxesMesh");

    String meshName = "SkeletonDebug/AxesMesh";
    mAxesMeshPtr = MeshManager::getSingleton().getByName(meshName);
    if (!mAxesMeshPtr)
    {
        ManualObject mo("");
        //mo.begin("BaseWhite");
        mo.begin("SkeletonDebug/AxesMat");
        /* 3 axes, each made up of 2 of these (base plane = XY)
         *   .------------|\
         *   '------------|/
         */
        mo.estimateVertexCount(7 * 2 * 3);
        mo.estimateIndexCount(3 * 2 * 3);
        Ogre::Quaternion quat[6];
        ColourValue col[3];

        // x-axis
        quat[0] = Ogre::Quaternion::IDENTITY;
        quat[1].FromAxes(Vector3::UNIT_X, Vector3::NEGATIVE_UNIT_Z, Vector3::UNIT_Y);
        col[0] = ColourValue::Red;
        col[0].a = 0.8;
        // y-axis
        quat[2].FromAxes(Vector3::UNIT_Y, Vector3::NEGATIVE_UNIT_X, Vector3::UNIT_Z);
        quat[3].FromAxes(Vector3::UNIT_Y, Vector3::UNIT_Z, Vector3::UNIT_X);
        col[1] = ColourValue::Green;
        col[1].a = 0.8;
        // z-axis
        quat[4].FromAxes(Vector3::UNIT_Z, Vector3::UNIT_Y, Vector3::NEGATIVE_UNIT_X);
        quat[5].FromAxes(Vector3::UNIT_Z, Vector3::UNIT_X, Vector3::UNIT_Y);
        col[2] = ColourValue::Blue;
        col[2].a = 0.8;

        Vector3 basepos[7] =
            {
             // stalk
             Vector3(0, 0.05, 0),
             Vector3(0, -0.05, 0),
             Vector3(0.7, -0.05, 0),
             Vector3(0.7, 0.05, 0),
             // head
             Vector3(0.7, -0.15, 0),
             Vector3(1, 0, 0),
             Vector3(0.7, 0.15, 0)
            };


        // vertices
        // 6 arrows
        for (size_t i = 0; i < 6; ++i)
        {
            // 7 points
            for (size_t p = 0; p < 7; ++p)
            {
                Vector3 pos = quat[i] * basepos[p];
                mo.position(pos);
                mo.colour(col[i / 2]);
            }
        }

        // indices
        // 6 arrows
        for (uint32 i = 0; i < 6; ++i)
        {
            uint32 base = i * 7;
            mo.triangle(base + 0, base + 1, base + 2);
            mo.triangle(base + 0, base + 2, base + 3);
            mo.triangle(base + 4, base + 5, base + 6);
        }
        mo.end();
        mAxesMeshPtr = mo.convertToMesh(meshName, ResourceGroupManager::INTERNAL_RESOURCE_GROUP_NAME);
        mAxesMeshPtr->_setBounds(mo.getBoundingBox());
        mAxesMeshPtr->_setBoundingSphereRadius(mo.getBoundingRadius());
        //mAxesMeshPtr->load();
        //mAxesMeshPtr->setMaterial(mAxisMatPtr);
        //mAxesMeshPtr->getSubMeshes()[0]->setMaterial(mAxisMatPtr);
    }
}




void BodyDebug::createArrowMesh()
{
    //spdlog::info("BodyDebug::mArrowMeshPtr");

    mArrowOrientation = Ogre::Vector3(1,0,0);
    String meshName = "SkeletonDebug/ArrowMesh";
    mArrowMeshPtr = MeshManager::getSingleton().getByName(meshName);
    if (!mArrowMeshPtr)
    {
        ManualObject mo("");
        mo.begin("SkeletonDebug/AxesMat");
        /* 1 axes, each made up of 2 of these (base plane = XY)
         *   .------------|\
         *   '------------|/
         */
        mo.estimateVertexCount(7 * 2);
        mo.estimateIndexCount(3 * 2);
        Ogre::Quaternion quat[2];
        ColourValue col[1];

        // x-axis
        quat[0] = Ogre::Quaternion::IDENTITY;
        quat[1].FromAxes(Vector3::UNIT_X, Vector3::NEGATIVE_UNIT_Z, Vector3::UNIT_Y);
        col[0] = ColourValue::Red;
        col[0].a = 0.8;

        Vector3 basepos[7] =
            {
             // stalk
             Vector3(0, 0.05, 0),
             Vector3(0, -0.05, 0),
             Vector3(0.7, -0.05, 0),
             Vector3(0.7, 0.05, 0),
             // head
             Vector3(0.7, -0.15, 0),
             Vector3(1, 0, 0),
             Vector3(0.7, 0.15, 0)
            };


        // vertices
        // 2 arrows
        for (size_t i = 0; i < 2; ++i)
        {
            // 7 points
            for (size_t p = 0; p < 7; ++p)
            {
                Vector3 pos = quat[i] * basepos[p];
                mo.position(pos);
                mo.colour(col[i / 2]);
            }
        }

        // indices
        // 2 arrows
        for (uint32 i = 0; i < 2; ++i)
        {
            uint32 base = i * 7;
            mo.triangle(base + 0, base + 1, base + 2);
            mo.triangle(base + 0, base + 2, base + 3);
            mo.triangle(base + 4, base + 5, base + 6);
        }
        mo.end();
        mArrowMeshPtr = mo.convertToMesh(meshName, ResourceGroupManager::INTERNAL_RESOURCE_GROUP_NAME);
        mArrowMeshPtr->_setBounds(mo.getBoundingBox());
        mArrowMeshPtr->_setBoundingSphereRadius(mo.getBoundingRadius());
    }


}
