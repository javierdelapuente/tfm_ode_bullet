#ifndef OBUGRE_BODYDEBUG_HPP
#define OBUGRE_BODYDEBUG_HPP

#include "objectTextDisplay.hpp"
#include "engine.hpp"

namespace OB
{

    class Renderer;
    class BodyDebug: public Ogre::Camera::Listener
    {
    public:
        BodyDebug(Renderer& renderer,
                  Ogre::SceneNode *bodySceneNode,
                  Ogre::SceneManager *sceneManager,
                  Ogre::Camera *camera,
                  Engine& engine,
                  BodyState* bodyState = nullptr);

        ~BodyDebug();

        void showAxes(bool show);
        void showNames(bool show);

        void update();

    private:
        void createAxesMaterial();
        void createAxesMesh();
        void createArrowMesh();

        void cameraPreRenderScene(Ogre::Camera* cam) override
        {
            Ogre::Camera::Listener::cameraPreRenderScene(cam);
            update();
        }

        BodyState * mBodyState;
        Renderer & renderer;
        Engine & engine;

        Ogre::SceneManager *mSceneManager;
        Ogre::SceneNode *mBodySceneNode;
        Ogre::Camera *mCamera;
        Ogre::SceneNode *mBaseNode;
        Ogre::SceneNode *mAxisNode;
        Ogre::SceneNode *mDebugNode;
        Ogre::SceneNode *mLinearMomentumNode;
        Ogre::SceneNode *mAngularMomentumNode;
        Ogre::SceneNode *mAngularVelocityNode;

        Ogre::MaterialPtr mAxisMatPtr;
        Ogre::MeshPtr mAxesMeshPtr;
        Ogre::MeshPtr mArrowMeshPtr;
        Ogre::Vector3 mArrowOrientation;
        std::unique_ptr<ObjectTextDisplay> mTextDisplay;
    };

}

#endif
