#ifndef OBVCLIP_OGREBITESAPP_HPP
#define OBVCLIP_OGREBITESAPP_HPP

#include <OgreApplicationContext.h>
#include <OgreCameraMan.h>
#include <OgreTrays.h>
#include <string>

#include "polytope.hpp"
#include "MovableText.hpp"
#include "vclip.hpp"

namespace OB
{

    class OgreVClipApp: public OgreBites::ApplicationContext,
                        public OgreBites::InputListener
    {
    public:

        ~OgreVClipApp() { }

        void addWorldConvexPolytope(WorldConvexPolytope& cv);

        // this is to force initialization of one pair
        void setVClipPair(WorldConvexPolytope& cv1,
                          WorldConvexPolytope& cv2,
                          VClipWitness& witness);
    private:
        void setup() override;
        void shutdown() override;
        bool renderOneFrame();
        bool keyPressed(const OgreBites::KeyboardEvent& evt);

        void drawWireFrame(Ogre::SceneNode* node, const std::string& name,
                           const ConvexPolytope& cv);
        void drawSolid(Ogre::SceneNode* node, const std::string& name, const ConvexPolytope& cv);

        // global coordinatas
        void drawDebugFeature(Ogre::SceneNode* debugNode, const WorldConvexPolytope& wcp, Feature f);
        void drawShowContacts(Ogre::SceneNode* debugNode);


        std::unique_ptr<OgreBites::CameraMan> cameraMan;
        Ogre::SceneManager* sceneManager;
        Ogre::Camera* camera;

        bool frameStarted(const Ogre::FrameEvent& evt) override
        {
            updatePoses();
            return OgreBites::ApplicationContext::frameStarted(evt);
        };
        //bool frameRenderingQueued(const FrameEvent&) override;
        //bool frameEnded(const FrameEvent&) override;

        void updatePoses();
        void hideShowParts();
        bool showWireframe = true;
        bool showSolid = false;
        bool showDebugBody = false;
        bool showVertexEdgeVoronoiPlanes = false;
        bool showEdgeFaceVoronoiPlanes = false;
        bool showContacts = false;
        bool contactsSimple = false;

        std::vector<std::reference_wrapper<WorldConvexPolytope> > polytopes ;

        double characterSize = 0.3;
        double arrowScaling = 0.5;

        std::unique_ptr<OgreBites::TrayManager> trayManager;
        std::vector<std::unique_ptr<Ogre::MovableText>> movableTexts;

        void createAxesMaterial();
        void createAxesMesh();
        void createArrowMesh();
        Ogre::Vector3 mArrowOrientation;

        // for step by step vclip

        bool vClipReady = false;
        std::unique_ptr<OB::VClip> vclip;
        using WorldConvexPolytopePairVector = std::vector<std::pair<std::reference_wrapper<WorldConvexPolytope>, std::reference_wrapper<WorldConvexPolytope>>>;

        WorldConvexPolytopePairVector allpairs;
        WorldConvexPolytopePairVector::iterator currentPairIterator;
        void stepVClip();
        void nextVClipPair();
        void drawVClipPair();


    };

}
#endif
