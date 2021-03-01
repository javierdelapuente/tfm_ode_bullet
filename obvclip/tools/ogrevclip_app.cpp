#include <sstream>
#include <memory>
#include <OgreWindowEventUtilities.h>
#include <Ogre.h>
#include "ogrevclip_app.hpp"
#include "MovableText.hpp"
#include "oblog.hpp"
#include "vclip.hpp"
#include "contacts.hpp"

using namespace OB;
using namespace Ogre;
using namespace OgreBites;

void OgreVClipApp::setup()
{
    ApplicationContext::setup();
    addInputListener(this);

    Ogre::Root* root = getRoot();
    sceneManager = root->createSceneManager();

    RTShader::ShaderGenerator* shadergen = RTShader::ShaderGenerator::getSingletonPtr();
    shadergen->addSceneManager(sceneManager);


    sceneManager->setAmbientLight(ColourValue(0.5, 0.5, 0.5));

    Light* light = sceneManager->createLight("MainLight");
    light->setDiffuseColour(0.5, 0.5, 0.5);
    light->setSpecularColour(0.5, 0.5, 0.5);

    SceneNode* lightNode = sceneManager->getRootSceneNode()->createChildSceneNode();
    lightNode->attachObject(light);
    lightNode->setPosition(20, 80, 50);

    SceneNode* camNode = sceneManager->getRootSceneNode()->createChildSceneNode();
    camera = sceneManager->createCamera("camera");
    camera->setNearClipDistance(1);
    camera->setAutoAspectRatio(true);
    // camNode->lookAt(Vector3(0, 0, 0), Node::TransformSpace::TS_WORLD);
    camNode->attachObject(camera);

    Viewport* viewPort = getRenderWindow()->addViewport(camera);
    viewPort->setBackgroundColour(ColourValue::White);
    camNode->setPosition(0, 7, 80);

    cameraMan.reset(new CameraMan(camNode));
    cameraMan->setStyle(OgreBites::CS_ORBIT);
    addInputListener(cameraMan.get());

    sceneManager->addRenderQueueListener(getOverlaySystem());
    //trayManager = std::make_unique<OgreBites::TrayManager>("traymanager", getRenderWindow()); //TODO c++14
    trayManager = std::unique_ptr<OgreBites::TrayManager>(new OgreBites::TrayManager("traymanager", getRenderWindow()));
    trayManager->showFrameStats(TL_BOTTOMLEFT);
    // trayManager->showLogo(TL_BOTTOMRIGHT);
    trayManager->hideCursor();
    OgreBites::Label *label =  trayManager->createLabel(TL_TOPLEFT, "nombre", "Prueba V-Clip", 220);
    addInputListener(trayManager.get());

    //create axes.
    createAxesMaterial();
    createAxesMesh();
    createArrowMesh();

    // add world axes
    SceneNode* axesnode = sceneManager->getRootSceneNode()->createChildSceneNode();
    //draw axis
    Ogre::Entity *entityAxes = sceneManager->createEntity("Obvclip/AxesMesh");
    axesnode->attachObject(entityAxes);


    // //TODO PRUEBA DE PLANOS.
    //SceneNode* planeNode = sceneManager->getRootSceneNode()->createChildSceneNode();
    // Ogre::MovablePlane* mPlane = new Ogre::MovablePlane("Plane");
    // mPlane->d = -1;
    // //mPlane->normal = Ogre::Vector3::UNIT_Y;
    // mPlane->normal = Ogre::Vector3(0,1,0);
    // Ogre::MeshManager::getSingleton().
    //   createPlane("PlaneMesh",
    //               Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
    //               *mPlane,
    //               10, 10, //width, height,
    //               1, 1,  // xsegments, ysegments
    //               true, //normals
    //               1, 1, 1, // numTextCoordsets, uTile, vTile
    //               //Ogre::Vector3::UNIT_Z // upVector
    //               Ogre::Vector3(1,0,1) // upVector
    //               );
    // Ogre::Entity* mPlaneEntity = sceneManager->createEntity("PlaneMesh");
    // mPlaneEntity->setMaterialName("Obvclip/RadioactiveGreenAlpha");
    // planeNode->attachObject(mPlaneEntity);
    // planeNode->attachObject(entity);

    //OTRA OPCION
    // Entity *entity = sceneManager->createEntity(Ogre::SceneManager::PT_PLANE);
    // entity->setMaterialName("Obvclip/RadioactiveGreenAlpha");

}

void OgreVClipApp::shutdown()
{
    // not very RAII
    trayManager.reset();
    cameraMan.reset();
    movableTexts.clear();
}

void OgreVClipApp::addWorldConvexPolytope(WorldConvexPolytope& cv)
{
    for (OB::WorldConvexPolytope &a: polytopes) {
        if (a.get_name() == cv.get_name())
            throw std::logic_error("WorldConvexPolytope duplicated name: "+ cv.get_name());
    }
    polytopes.push_back(std::ref(cv));

    SceneNode* cvNode = sceneManager->getRootSceneNode()->createChildSceneNode(cv.get_name());
    drawWireFrame(cvNode, cv.get_name(), cv.convex_polytope());
    drawSolid(cvNode, cv.get_name(), cv.convex_polytope());
    hideShowParts();
}

void OgreVClipApp::drawWireFrame(Ogre::SceneNode* cvnode,
                                 const std::string& name,
                                 const ConvexPolytope& cv)
{
    SceneNode* node = cvnode->createChildSceneNode(name + "-wireframe");
    SceneNode* debugnode = cvnode->createChildSceneNode(name + "-debug");

    //draw axis
    Ogre::Entity *entityAxes = sceneManager->createEntity("Obvclip/AxesMesh");
    debugnode->attachObject(entityAxes);


    Ogre::MovableText* msg = new Ogre::MovableText(name,
                                                   name,
                                                   characterSize,
                                                   ColourValue::Black);
    movableTexts.push_back(std::unique_ptr<Ogre::MovableText>(msg));
    msg->setTextAlignment(Ogre::MovableText::H_LEFT, Ogre::MovableText::V_CENTER);
    msg->setLocalTranslation(Vector3(0.5, -0.5, 0));
    msg->showOnTop(true);
    debugnode->attachObject(msg);


    Ogre::ManualObject* man = sceneManager->createManualObject();

    const FaceList& faces = cv.get_faces();

    for (auto it = faces.begin(); it != faces.end(); ++it)
    {
        FeatureId fid = std::distance(faces.begin(), it);
        const Face& face = *it;
        FeatureId heId = face.get_halfedge_id();

        //man->begin("Examples/OgreLogo", Ogre::RenderOperation::OT_LINE_STRIP);
        man->begin("Obvclip/WireFrame",
                   Ogre::RenderOperation::OT_LINE_STRIP,
                   Ogre::ResourceGroupManager::AUTODETECT_RESOURCE_GROUP_NAME);
        do {
            const HalfEdge& he = cv.get_halfedge(heId);

            FeatureId headId = he.get_headvertex_id();
            FeatureId tailId = cv.get_halfedge(he.get_pair_id()).get_headvertex_id();

            const Vertex& tail = cv.get_vertex(tailId);
            const Vertex& head = cv.get_vertex(headId);

            man->position(tail.get_position()[0], tail.get_position()[1], tail.get_position()[2]);
            man->position(head.get_position()[0], head.get_position()[1], head.get_position()[2]);

            heId = he.get_next_id();
        }while (heId != face.get_halfedge_id());
        man->end();

        //how draw something in the center of the face.
        Point center = face.get_center(cv);
        // man->begin("Examples/OgreLogo", Ogre::RenderOperation::OT_POINT_LIST);
        // man->position(center[0], center[1], center[2]);
        // man->end();

        SceneNode* facenode = debugnode->createChildSceneNode();
        facenode->setPosition(center[0], center[1], center[2]);

        SceneNode* facenormalnode = facenode->createChildSceneNode();
        Ogre::Entity *entityArrow = sceneManager->createEntity("Obvclip/ArrowMesh");
        facenormalnode->attachObject(entityArrow);
        // arrow for the normal
        Vector normal = face.get_plane().normal();
        Ogre::Vector3 planeNormal = Ogre::Vector3(normal[0], normal[1], normal[2]);
        Ogre::Quaternion q = mArrowOrientation.getRotationTo(planeNormal);
        facenormalnode->setOrientation(q);
        facenormalnode->setScale(arrowScaling, arrowScaling, arrowScaling);

        // Text for the face?
        std::ostringstream faceInfo;
        faceInfo << "F" << fid;

        std::ostringstream faceName;
        faceName << name << "-" << faceInfo.str();

        Ogre::MovableText* msg = new Ogre::MovableText(faceName.str(),
                                                       faceInfo.str(), characterSize,
                                                       ColourValue::Black);
        movableTexts.push_back(std::unique_ptr<Ogre::MovableText>(msg));
        msg->setTextAlignment(Ogre::MovableText::H_LEFT, Ogre::MovableText::V_CENTER);
        msg->setLocalTranslation(Vector3(0.5, -0.5, 0));
        msg->showOnTop(true);
        facenode->attachObject(msg);
    }
    node->attachObject(man);

    const VertexList& vertices = cv.get_vertices();
    for (auto it = vertices.begin(); it != vertices.end(); ++it)
    {
        FeatureId vid = std::distance(vertices.begin(), it);
        std::ostringstream vertexInfo;
        vertexInfo << "V";
        vertexInfo << vid; // << " " << it->get_position().transpose();

        std::ostringstream vertexName;
        vertexName << name << "-" << vertexInfo.str();

        Ogre::MovableText* msg = new Ogre::MovableText(vertexName.str(),
                                                       vertexInfo.str(),
                                                       characterSize,
                                                       ColourValue::Black);
        movableTexts.push_back(std::unique_ptr<Ogre::MovableText>(msg));
        msg->setTextAlignment(Ogre::MovableText::H_CENTER, Ogre::MovableText::V_CENTER);
        msg->showOnTop(true);
        msg->setLocalTranslation(Vector3(0.5, -0.5, 0));

        SceneNode* vnode = debugnode->createChildSceneNode();
        vnode->attachObject(msg);
        vnode->setPosition(it->get_position()[0],
                           it->get_position()[1],
                           it->get_position()[2]);
        //bounding box :
        std::cout  << msg->GetAABB();
    }

}

void OgreVClipApp::drawSolid(Ogre::SceneNode* cvnode, const std::string& name,
                             const ConvexPolytope& cv)
{
    SceneNode* node = cvnode->createChildSceneNode(name + "-solid");

    Ogre::ManualObject* man = sceneManager->createManualObject();

    const FaceList& faces = cv.get_faces();
    for (auto it = faces.begin(); it != faces.end(); ++it)
    {
        const Face& face = *it;
        FeatureId heId = face.get_halfedge_id();
        man->begin("Examples/OgreLogo", Ogre::RenderOperation::OT_TRIANGLE_FAN);
        do {
            const HalfEdge& he = cv.get_halfedge(heId);

            FeatureId headId = he.get_headvertex_id();
            FeatureId tailId = cv.get_halfedge(he.get_pair_id()).get_headvertex_id();

            const Vertex& tail = cv.get_vertex(tailId);
            const Vertex& head = cv.get_vertex(headId);

            man->position(tail.get_position()[0], tail.get_position()[1], tail.get_position()[2]);
            man->position(head.get_position()[0], head.get_position()[1], head.get_position()[2]);

            heId = he.get_next_id();
        }while (heId != face.get_halfedge_id());
        man->end();
    }

    for (const auto& sect: man->getSections())
    {
        sect->setMaterialName("Obvclip/solid");
    }

    node->attachObject(man);
}

void OgreVClipApp::drawDebugFeature(Ogre::SceneNode* debugNode,
                                    const WorldConvexPolytope& wcp, Feature f)
{
    SceneNode* objNode = debugNode->createChildSceneNode();

    Vector translation =  wcp.get_pose().translation();
    objNode->setPosition(translation[0], translation[1], translation[2]);
    Quaternion orientation{wcp.get_pose().rotation()};
    objNode->setOrientation(orientation.w(), orientation.x(),
                            orientation.y(), orientation.z());

    const ConvexPolytope& polytope = wcp.convex_polytope();
    switch (f.type)
    {
    case FeatureType::vertex: {
        Ogre::ManualObject* man = sceneManager->createManualObject();
        man->begin("Obvclip/DebugVClip",
                   Ogre::RenderOperation::OT_POINT_LIST,
                   Ogre::ResourceGroupManager::AUTODETECT_RESOURCE_GROUP_NAME);
        Point p = polytope.get_vertex_point(f);
        man->position(p[0],p[1],p[2]);
        man->end();
        objNode->attachObject(man);

        // draw voronoi planes. think again in this
        if (showVertexEdgeVoronoiPlanes) {
            auto vps = polytope.get_vpve_vertex(f);
            for (auto &vp : vps) {
                Vector normal = vp.plane.normal();
                Ogre::Vector3 planeNormal = Ogre::Vector3(normal[0], normal[1], normal[2]);
                Ogre::Vector3 planeOrientation(0,0,1); // default in Ogre
                Ogre::Quaternion qp = planeOrientation.getRotationTo(planeNormal);
                SceneNode* planeNode = objNode->createChildSceneNode();
                planeNode->setOrientation(qp);
                planeNode->setPosition(p[0], p[1], p[2]);
                planeNode->setScale(0.05, 0.05, 0.05);
                Entity *entity = sceneManager->createEntity(Ogre::SceneManager::PT_PLANE);
                entity->setMaterialName("Obvclip/DebugPlaneVClip");
                planeNode->attachObject(entity);
            }
        }

        break;
    }
    case FeatureType::edge: {
        Ogre::ManualObject* man = sceneManager->createManualObject();
        man->begin("Obvclip/DebugVClip",
                   Ogre::RenderOperation::OT_LINE_STRIP,
                   Ogre::ResourceGroupManager::AUTODETECT_RESOURCE_GROUP_NAME);
        EdgePoints e =  polytope.get_edgepoints(f);
        man->position(e.tail[0],e.tail[1],e.tail[2]);
        man->position(e.head[0],e.head[1],e.head[2]);
        man->end();
        objNode->attachObject(man);

        //draw vertex-edge voronoi planes. think again in this
        if (showVertexEdgeVoronoiPlanes)
        {
            auto vps = polytope.get_vpve_edge(f);
            for (auto &vp : vps) {
                Point vertexPoint =  polytope.get_vertex_point(vp.feature);
                Vector normal = vp.plane.normal();
                Ogre::Vector3 planeNormal = Ogre::Vector3(normal[0], normal[1], normal[2]);
                Ogre::Vector3 planeOrientation(0,0,1); // default in Ogre
                Ogre::Quaternion qp = planeOrientation.getRotationTo(planeNormal);
                SceneNode* planeNode = objNode->createChildSceneNode();
                planeNode->setOrientation(qp);
                planeNode->setPosition(vertexPoint[0], vertexPoint[1], vertexPoint[2]);
                planeNode->setScale(0.05, 0.05, 0.05);
                Entity *entity = sceneManager->createEntity(Ogre::SceneManager::PT_PLANE);
                entity->setMaterialName("Obvclip/DebugPlaneVClip");
                planeNode->attachObject(entity);
            }
        }

        if (showEdgeFaceVoronoiPlanes) {
            auto vps = polytope.get_vpfe_edge(f);
            for (auto &vp : vps) {
                Point mp =  e.midpoint();
                Vector normal = vp.plane.normal();
                Ogre::Vector3 planeNormal = Ogre::Vector3(normal[0], normal[1], normal[2]);
                Ogre::Vector3 planeOrientation(0,0,1); // default in Ogre
                Ogre::Quaternion qp = planeOrientation.getRotationTo(planeNormal);
                SceneNode* planeNode = objNode->createChildSceneNode();
                planeNode->setOrientation(qp);
                planeNode->setPosition(mp[0], mp[1], mp[2]);
                planeNode->setScale(0.05, 0.05, 0.05);
                Entity *entity = sceneManager->createEntity(Ogre::SceneManager::PT_PLANE);
                entity->setMaterialName("Obvclip/DebugPlaneVClip");
                planeNode->attachObject(entity);
            }
        }


        break;
    }
    case FeatureType::face: {
        Ogre::ManualObject* man = sceneManager->createManualObject();
        man->begin("Obvclip/DebugVClip",
                   Ogre::RenderOperation::OT_LINE_STRIP,
                   Ogre::ResourceGroupManager::AUTODETECT_RESOURCE_GROUP_NAME);

        std::vector<Point> points = polytope.get_face_points(f);
        for (auto& p : points) {
            man->position(p[0],p[1],p[2]);
        }
        man->position(points.front()[0],points.front()[1],points.front()[2]);
        man->end();
        objNode->attachObject(man);


        if (showEdgeFaceVoronoiPlanes) {
            auto vps = polytope.get_vpfe_face(f);
            for (auto &vp : vps) {
                EdgePoints e = polytope.get_edgepoints(vp.feature);
                Point mp =  e.midpoint();
                Vector normal = vp.plane.normal();
                Ogre::Vector3 planeNormal = Ogre::Vector3(normal[0], normal[1], normal[2]);
                Ogre::Vector3 planeOrientation(0,0,1); // default in Ogre
                Ogre::Quaternion qp = planeOrientation.getRotationTo(planeNormal);
                SceneNode* planeNode = objNode->createChildSceneNode();
                planeNode->setOrientation(qp);
                planeNode->setPosition(mp[0], mp[1], mp[2]);
                planeNode->setScale(0.05, 0.05, 0.05);
                Entity *entity = sceneManager->createEntity(Ogre::SceneManager::PT_PLANE);
                entity->setMaterialName("Obvclip/DebugPlaneVClip");
                planeNode->attachObject(entity);
            }
        }

        //THIS ARE ALL THE "PERIMITER PLANES"
        if (showVertexEdgeVoronoiPlanes)
        {
            auto scanningperimeter = polytope.get_scan_face(f);
            for (auto &sp : scanningperimeter) {
                for (auto &vp : sp.vertexedge_vps)
                {
                    if (sp.feature.isEdge()) {
                        Point vertexPoint =  polytope.get_vertex_point(vp.feature);
                        Vector normal = vp.plane.normal();
                        Ogre::Vector3 planeNormal = Ogre::Vector3(normal[0], normal[1], normal[2]);
                        Ogre::Vector3 planeOrientation(0,0,1); // default in Ogre
                        Ogre::Quaternion qp = planeOrientation.getRotationTo(planeNormal);
                        SceneNode* planeNode = objNode->createChildSceneNode();
                        planeNode->setOrientation(qp);
                        planeNode->setPosition(vertexPoint[0], vertexPoint[1], vertexPoint[2]);
                        planeNode->setScale(0.05, 0.05, 0.05);
                        Entity *entity = sceneManager->createEntity(Ogre::SceneManager::PT_PLANE);
                        entity->setMaterialName("Obvclip/DebugPlaneVClip");
                        planeNode->attachObject(entity);
                    }
                }

            }
        }


        break;
    }
    }
}

bool OgreVClipApp::renderOneFrame()
{
    Ogre::WindowEventUtilities::messagePump();
    return getRoot()->renderOneFrame();
}

void OgreVClipApp::updatePoses()
{
    for (OB::WorldConvexPolytope& wcv: polytopes)
    {
        SceneNode* node = sceneManager->getSceneNode(wcv.get_name());
        const auto& translation = wcv.get_pose().translation();
        node->setPosition(translation[0],
                          translation[1],
                          translation[2]);
        Quaternion q(wcv.get_pose().rotation());
        node->setOrientation(q.w(), q.x(), q.y(), q.z());
    }
}

void OgreVClipApp::hideShowParts()
{
    for (OB::WorldConvexPolytope& p : polytopes)
    {
        const String &name = p.get_name();
        SceneNode *wireframe =sceneManager->getSceneNode(name + "-wireframe");
        wireframe->setVisible(showWireframe);

        SceneNode *solid =sceneManager->getSceneNode(name + "-solid");
        solid->setVisible(showSolid);

        SceneNode *debug =sceneManager->getSceneNode(name + "-debug");
        debug->setVisible(showDebugBody);
    }
}

bool OgreVClipApp::keyPressed(const KeyboardEvent& evt)
{

    L_(lerror) << "Key Pressed: type: " << evt.type
               << " repeat: " << evt.repeat << " sym: "
               << evt.keysym.mod << "-" << evt.keysym.sym;
    if (evt.keysym.sym == SDLK_ESCAPE)
    {
        getRoot()->queueEndRendering();
    }
    else if (evt.keysym.sym == '1')
    {
        showWireframe = !showWireframe;
        hideShowParts();
    }
    else if (evt.keysym.sym == '2')
    {
        // hide/show solids
        showSolid = !showSolid;
        hideShowParts();
    }
    else if (evt.keysym.sym == '3')
    {
        // hide/show solids
        showDebugBody = !showDebugBody;
        hideShowParts();
    }
    else if (evt.keysym.sym == '4')
    {
        showVertexEdgeVoronoiPlanes = !showVertexEdgeVoronoiPlanes;
        drawVClipPair();
    }
    else if (evt.keysym.sym == '5')
    {
        showEdgeFaceVoronoiPlanes = !showEdgeFaceVoronoiPlanes;
        drawVClipPair();
    }
    else if (evt.keysym.sym == '6')
    {
        showContacts = !showContacts;
        drawVClipPair();
    }
    else if (evt.keysym.sym == '7')
    {
        contactsSimple = !contactsSimple;
        drawVClipPair();
    }



    else if (evt.keysym.sym == 'p')
    {
        // print polytope data
        for (OB::WorldConvexPolytope& cv : polytopes)
        {
            const String &name = cv.get_name();
            std::cout << name << " -> " << cv << std::endl;
        }
    }
    else if (evt.keysym.sym == 'a')
    {
        // vclip against all of objects
        OB::VClipCache cache;
        for (OB::WorldConvexPolytope& cv1 : polytopes)
        {
            for (OB::WorldConvexPolytope& cv2 : polytopes)
            {
                if (cv1.get_name() == cv2.get_name()) continue;
                std::cout << "VCLIP ALGORITHM>>> " << cv1.get_name() << " vs " << cv2.get_name() << std::endl;
                try {
                    VClipResult res = VClip(cv1, cv2)(cache);
                    std::cout << " Vclip resp " << res << std::endl;
                } catch (const std::logic_error& e) {
                    std::cout << " vclip exception "  << e.what()<< std::endl;
                }
            }
        }
    }

    else if (evt.keysym.sym == 'c')
    {
        stepVClip();
    }
    else if (evt.keysym.sym == 'n')
    {
        nextVClipPair();
    }

    //print screen to file
    else if (evt.keysym.sym == 'd')
    {
        getRenderWindow()->writeContentsToTimestampedFile ("vclipsnapshot", ".png");
        return true;
    }


    // false allow other handlers to process the keyPressed
    return false;
}

void OgreVClipApp::stepVClip()
{
    if (!vClipReady)
    {
        nextVClipPair();
    }
    else
    {
        const WorldConvexPolytope& wcp0 = (*currentPairIterator).first;
        const WorldConvexPolytope& wcp1 = (*currentPairIterator).second;
        VClipWitness beforeWitness = vclip->get_currentwitness();
        std::cout << "Before Witness" << beforeWitness << std::endl;
        Real distanceBefore = absdist_between_features(wcp0, beforeWitness.feature1,
                                                      wcp1, beforeWitness.feature2);
        std::cout << "DISTANCE BEFORE " << distanceBefore << std::endl;

        VClipStateResult finished = vclip->iterate().result;
        VClipWitness afterWitness  = vclip->get_currentwitness();
        Real distanceAfter = absdist_between_features(wcp0, afterWitness.feature1,
                                                     wcp1, afterWitness.feature2);
        std::cout << "DISTANCE AFTER " << distanceAfter << std::endl;
        std::cout << "iteration vclip. finished: " << finished << std::endl;
        std::cout << "After Witness" << afterWitness << std::endl;

    }
    drawVClipPair();
}

void OgreVClipApp::drawVClipPair()
{
    if (!vClipReady) {
        return;
    }

    // draw debug elements
    SceneNode *debugnode = nullptr;
    if (sceneManager->hasSceneNode("vclip-debug"))
    {
        debugnode = sceneManager->getSceneNode("vclip-debug");
        sceneManager->destroySceneNode(debugnode);
    }
    debugnode = sceneManager->getRootSceneNode()->createChildSceneNode("vclip-debug");
    drawDebugFeature(debugnode,
                     (*currentPairIterator).first,
                     vclip->get_currentwitness().feature1);
    drawDebugFeature(debugnode,
                     (*currentPairIterator).second,
                     vclip->get_currentwitness().feature2);

    if (showContacts) {
        drawShowContacts(debugnode);
    }
}

void OgreVClipApp::drawShowContacts(Ogre::SceneNode* debugNode)
{
    VClipResult lastResult = vclip->get_lastresult();
    std::vector<ContactDataVClip> cds;
    if (!contactsSimple) {
        cds = get_contacts_envelope((*currentPairIterator).first,
                                    (*currentPairIterator).second,
                                    lastResult,
                                    0.50); // envelope
    } else {
        cds.push_back(get_simple_contact((*currentPairIterator).first,
                                         (*currentPairIterator).second,
                                         lastResult));
    }

    for (auto &cd : cds)
    {
        SceneNode *node = debugNode->createChildSceneNode();
        Ogre::Entity *entityArrow = sceneManager->createEntity("Obvclip/ArrowMesh");
        node->attachObject(entityArrow);
        Ogre::Vector3 planeNormal = Ogre::Vector3(cd.normal[0],
                                                  cd.normal[1],
                                                  cd.normal[2]);
        Ogre::Quaternion q = mArrowOrientation.getRotationTo(planeNormal);
        node->setOrientation(q);
        node->setPosition(cd.point[0], cd.point[1], cd.point[2]);
    }
}


void OgreVClipApp::nextVClipPair()
{
    if (!vClipReady)
    {
        //generate all pairs
        for (OB::WorldConvexPolytope& cv1 : polytopes) {
            for (OB::WorldConvexPolytope& cv2 : polytopes) {
                if (cv1.get_name() == cv2.get_name()) continue;
                allpairs.push_back(std::make_pair<std::reference_wrapper<WorldConvexPolytope>, std::reference_wrapper<WorldConvexPolytope>>(cv1, cv2));
            }
        }
        currentPairIterator = allpairs.begin();
        vClipReady = true;
    }
    else
    {
        //iterate to the next one
        std::cout << "next vclip pair of polytopes" << std::endl;
        if (++currentPairIterator == allpairs.end())
        {
            std::cout << "starting from the beginning again" << std::endl;
            currentPairIterator = allpairs.begin();
        }
    }
    std::cout << " WorldConvexPolytopes1:" << (*currentPairIterator).first.get().get_name()
              << " WorldConvexPolytopes2:" << (*currentPairIterator).second.get().get_name()
              << std::endl;

    vclip.reset(new VClip{(*currentPairIterator).first.get(),
                              (*currentPairIterator).second.get()});
    vclip->set_currentwitness(VClipWitness::FirstVertices());

    std::cout << "Initial witness" << vclip->get_currentwitness() << std::endl;

    drawVClipPair();
}


void OgreVClipApp::setVClipPair(WorldConvexPolytope& cv1,
                                WorldConvexPolytope& cv2,
                                VClipWitness& witness)
{
    allpairs.clear();
    allpairs.push_back(std::make_pair<std::reference_wrapper<WorldConvexPolytope>,
                       std::reference_wrapper<WorldConvexPolytope>>(cv1, cv2));
    currentPairIterator = allpairs.begin();
    vClipReady = true;

    vclip.reset(new VClip{(*currentPairIterator).first.get(),
                              (*currentPairIterator).second.get()});
    vclip->set_currentwitness(witness);
    std::cout << "Initial witness" << vclip->get_currentwitness() << std::endl;

    drawVClipPair();
}




void OgreVClipApp::createAxesMaterial()
{
    //spdlog::info("BodyDebug::createAxesMaterial");
    Ogre::String matName = "Obvclip/AxesMat";

    Ogre::MaterialPtr mAxisMatPtr = MaterialManager::getSingleton().getByName(matName);
    if (!mAxisMatPtr)
    {
        mAxisMatPtr = MaterialManager::getSingleton()
            .create(matName, ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME); //, ResourceGroupManager::INTERNAL_RESOURCE_GROUP_NAME);

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



void OgreVClipApp::createAxesMesh()
{
    //spdlog::info("BodyDebug::createAxesMesh");

    String meshName = "Obvclip/AxesMesh";
    Ogre::MeshPtr mAxesMeshPtr = MeshManager::getSingleton().getByName(meshName);
    if (!mAxesMeshPtr)
    {
        ManualObject mo("");
        //mo.begin("BaseWhite");
        mo.begin("Obvclip/AxesMat");
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
        mAxesMeshPtr = mo.convertToMesh(meshName, ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
        mAxesMeshPtr->_setBounds(mo.getBoundingBox());
        mAxesMeshPtr->_setBoundingSphereRadius(mo.getBoundingRadius());
        //mAxesMeshPtr->load();
        //mAxesMeshPtr->setMaterial(mAxisMatPtr);
        //mAxesMeshPtr->getSubMeshes()[0]->setMaterial(mAxisMatPtr);
    }
}




void OgreVClipApp::createArrowMesh()
{
    //spdlog::info("BodyDebug::mArrowMeshPtr");

    mArrowOrientation = Ogre::Vector3(1,0,0);
    String meshName = "Obvclip/ArrowMesh";
    Ogre::MeshPtr mArrowMeshPtr = MeshManager::getSingleton().getByName(meshName);
    if (!mArrowMeshPtr)
    {
        ManualObject mo("");
        mo.begin("Obvclip/AxesMat");
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
        mArrowMeshPtr = mo.convertToMesh(meshName); //, ResourceGroupManager::INTERNAL_RESOURCE_GROUP_NAME);
        mArrowMeshPtr->_setBounds(mo.getBoundingBox());
        mArrowMeshPtr->_setBoundingSphereRadius(mo.getBoundingRadius());
    }

}
