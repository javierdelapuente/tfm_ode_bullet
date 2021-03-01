#include <iostream>
#include "Ogre.h"
#include "OgreApplicationContext.h"
#include "OgreInput.h"
#include "OgreRTShaderSystem.h"
#include <OgreWindowEventUtilities.h>
#include <iostream>
#include "btBulletDynamicsCommon.h"
#include <ode/ode.h>

using namespace Ogre;
using namespace OgreBites;

class BasicTutorial1
  : public ApplicationContext
  , public InputListener
{
public:
  BasicTutorial1();
  virtual ~BasicTutorial1() {}

  void setup();
  void setupbullet();

  void mainloop();

  bool keyPressed(const KeyboardEvent& evt);
protected:
  btDefaultCollisionConfiguration* collisionConfiguration;
  btCollisionDispatcher* dispatcher;
  btBroadphaseInterface* overlappingPairCache;
  btSequentialImpulseConstraintSolver* solver;
  btDiscreteDynamicsWorld* dynamicsWorld;
  btAlignedObjectArray<btCollisionShape*> collisionShapes;

  SceneNode* ogreNode4;
  SceneNode* ogreNode5;

  dWorldID world;
  dSpaceID space;
  dJointGroupID contactgroup;
  dBodyID body;
  dGeomID geom;
};

BasicTutorial1::BasicTutorial1()
  : ApplicationContext("OgreTutorialApp")
{
  // inicializo bullet y ode
  collisionConfiguration = new btDefaultCollisionConfiguration();
  dispatcher = new btCollisionDispatcher(collisionConfiguration);
  overlappingPairCache = new btDbvtBroadphase();
  solver = new btSequentialImpulseConstraintSolver;
  dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, overlappingPairCache, solver, collisionConfiguration);
  dynamicsWorld->setGravity(btVector3(0, -10, 0));

  // ahora ode
  dInitODE();
  world = dWorldCreate();
  space = dHashSpaceCreate(0);
  contactgroup = dJointGroupCreate(0);
  dWorldSetGravity(world,0,-10,0);
  dWorldSetCFM(world,1e-5);
  dWorldSetAutoDisableFlag(world,1);

  dWorldSetLinearDamping(world, 0.00001);
  dWorldSetAngularDamping(world, 0.005);
  dWorldSetMaxAngularSpeed(world, 200);

  dWorldSetContactMaxCorrectingVel(world,0.1);
  dWorldSetContactSurfaceLayer(world,0.001);

}

void BasicTutorial1::setup()
{
    // do not forget to call the base first
    ApplicationContext::setup();
    addInputListener(this);

    // get a pointer to the already created root
    Root* root = getRoot();
    SceneManager* scnMgr = root->createSceneManager();

    // register our scene with the RTSS
    RTShader::ShaderGenerator* shadergen = RTShader::ShaderGenerator::getSingletonPtr();
    shadergen->addSceneManager(scnMgr);

    // -- tutorial section start --
    //! [turnlights]
    scnMgr->setAmbientLight(ColourValue(0.5, 0.5, 0.5));
    //! [turnlights]

    //! [newlight]
    Light* light = scnMgr->createLight("MainLight");
    SceneNode* lightNode = scnMgr->getRootSceneNode()->createChildSceneNode();
    lightNode->attachObject(light);
    //! [newlight]

    //! [lightpos]
    lightNode->setPosition(20, 80, 50);
    //! [lightpos]

    //! [camera]
    SceneNode* camNode = scnMgr->getRootSceneNode()->createChildSceneNode();

    // create the camera
    Camera* cam = scnMgr->createCamera("myCam");
    cam->setNearClipDistance(5); // specific to this sample
    cam->setAutoAspectRatio(true);
    camNode->attachObject(cam);
    camNode->setPosition(0, 0, 440);

    // and tell it to render into the main window
    getRenderWindow()->addViewport(cam);
    //! [camera]


    //! [cameramove]
    camNode->setPosition(0, 47, 422);
    //! [cameramove]


    //! [entity4]
    Entity* ogreEntity4 = scnMgr->createEntity("mySphere", Ogre::SceneManager::PT_CUBE);
    ogreNode4 = scnMgr->getRootSceneNode()->createChildSceneNode();
    // ogreNode4->setScale(Ogre::Vector3(0.5,0.5,0.5));// por defecto es 100x100x100
    ogreNode4->setPosition(-20, 48, 0);
    // ogreNode4->roll(Radian(Math::PI/4));
    ogreNode4->attachObject(ogreEntity4);
    // ! [entity4]

    btCollisionShape* box4 = new btBoxShape(btVector3(btScalar(50), btScalar(50), btScalar(50)));
    btScalar mass = 5;
    btVector3 inertia;
    box4->calculateLocalInertia(mass, inertia);
    btTransform box4transform;
    box4transform.setIdentity();
    box4transform.setOrigin(btVector3(-20, 48, 0));

    btDefaultMotionState* box4State = new btDefaultMotionState(box4transform);
    btRigidBody *box4Body = new btRigidBody(mass, box4State, box4, inertia);
    dynamicsWorld->addRigidBody(box4Body);


    // ahora en ode
    Entity* ogreEntity5 = scnMgr->createEntity("mySphere2", Ogre::SceneManager::PT_CUBE);
    ogreEntity5->setMaterialName("Obugre/RadioactiveGreenAlpha");

    ogreNode5 = scnMgr->getRootSceneNode()->createChildSceneNode();
    ogreNode5->setPosition(+100, 48, 0);
    ogreNode5->attachObject(ogreEntity5);

    body = dBodyCreate(world);
    dBodySetPosition(body, 100, 48, 0);
    dMass m;
    dMassSetBox(&m,1, 50, 50, 50);
    geom = dCreateBox(space,50, 50, 50);
    dGeomSetBody(geom, body);
    dBodySetMass(body, &m);

    Ogre::Plane plane(Vector3::UNIT_Y, -20);
    Ogre::MeshManager::getSingleton().createPlane("plane",
                                                  ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, plane,
                                                  1500,1500,20,20,true,1,5,5,Vector3::UNIT_Z);
    Ogre::Entity* ent = scnMgr->createEntity("LightPlaneEntity",
                                                "plane");
    scnMgr->getRootSceneNode()->createChildSceneNode()->attachObject(ent);
    ent->setMaterialName("Obugre/RadioactiveGreenAlpha");
}


bool BasicTutorial1::keyPressed(const KeyboardEvent& evt)
{
    if (evt.keysym.sym == SDLK_ESCAPE)
    {
        getRoot()->queueEndRendering();
    }
    return true;
}

void BasicTutorial1::mainloop()
{
  getRoot()->queueEndRendering(false);
  getRoot()->clearEventTimes();
  while (!getRoot()->endRenderingQueued())
    {
      Ogre::WindowEventUtilities::messagePump();

      dynamicsWorld->stepSimulation(1.f / 60.f, 10);

      for (int j = dynamicsWorld->getNumCollisionObjects() - 1; j >= 0; j--)
        {

          btCollisionObject* obj = dynamicsWorld->getCollisionObjectArray()[j];
          btRigidBody* body = btRigidBody::upcast(obj);
          btTransform trans;
          if (body && body->getMotionState())
			{
              body->getMotionState()->getWorldTransform(trans);
			}
          else
			{
              trans = obj->getWorldTransform();
			}
          printf("world pos object %d = %f,%f,%f\n", j, float(trans.getOrigin().getX()), float(trans.getOrigin().getY()), float(trans.getOrigin().getZ()));
          // ahora hay que mover!!
          btQuaternion rot = trans.getRotation();
          btVector3 pos = trans.getOrigin();
          ogreNode4->setOrientation(rot.w(), rot.x(), rot.y(), rot.z());
          ogreNode4->setPosition(pos.x(), pos.y(), pos.z());
        }

      dWorldQuickStep(world, 0.02);
      dJointGroupEmpty(contactgroup);

      const dReal *pos = dBodyGetPosition(body);
      ogreNode5->setPosition(pos[0], pos[1], pos[2]);

      if (!getRoot()->renderOneFrame())
        break;


    }

}

int main(int argc, char **argv)
{
    try
    {
    	BasicTutorial1 app;
        app.initApp();
        app.mainloop();
        app.closeApp();
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error occurred during execution: " << e.what() << '\n';
        return 1;
    }

    return 0;
}
