#include <OgreWindowEventUtilities.h>
#include <Ogre.h>

#include "renderer.hpp"
#include "oblog.hpp"
#include "body_debug.hpp"

using namespace OB;
using namespace Ogre;
using namespace OgreBites;

void Renderer::shutdown()
{
    tray_manager.reset();
    camera_man.reset();
    OgreBites::ApplicationContext::shutdown();
}

void Renderer::setup()
{
    //change default log level.
    LogManager::getSingleton().setLogDetail(Ogre::LoggingLevel::LL_LOW);


    OgreBites::ApplicationContext::setup();
    addInputListener(this);

    Ogre::Root* root = getRoot();
    scene_manager = root->createSceneManager();

    // register our scene with the RTSS
    RTShader::ShaderGenerator* shadergen = RTShader::ShaderGenerator::getSingletonPtr();
    shadergen->addSceneManager(scene_manager);

    scene_manager->setAmbientLight(ColourValue(0.5, 0.5, 0.5));

    Light* light = scene_manager->createLight("MainLight");
    light->setDiffuseColour(0.5, 0.5, 0.5);
    light->setSpecularColour(0.5, 0.5, 0.5);

    SceneNode* lightNode = scene_manager->getRootSceneNode()->createChildSceneNode();
    lightNode->attachObject(light);
    lightNode->setPosition(20, 80, 50);

    SceneNode* camNode = scene_manager->getRootSceneNode()->createChildSceneNode();
    camera = scene_manager->createCamera("camera");
    camera->setNearClipDistance(1);
    camera->setAutoAspectRatio(true);
    camera->setProjectionType(PT_PERSPECTIVE);
    // camNode->lookAt(Vector3(0, 0, 0), Node::TransformSpace::TS_WORLD);
    camNode->attachObject(camera);

    Viewport* viewPort = getRenderWindow()->addViewport(camera);
    viewPort->setBackgroundColour(ColourValue::White);

    getRenderWindow()->setVSyncEnabled (vsync_enabled);

    camNode->setPosition(0, 7, 80);

    camera_man.reset(new OgreBites::CameraMan(camNode));
    camera_man->setStyle(OgreBites::CS_ORBIT);
    addInputListener(camera_man.get());

    scene_manager->addRenderQueueListener(getOverlaySystem());

    tray_manager = std::unique_ptr<OgreBites::TrayManager>
        (new OgreBites::TrayManager("traymanager", getRenderWindow()));
    tray_manager->showFrameStats(OgreBites::TL_BOTTOMLEFT);
    // trayManager->showLogo(OgreBites::TL_BOTTOMRIGHT);
    tray_manager->hideCursor();
    OgreBites::Label *label =  tray_manager->createLabel(OgreBites::TL_TOPLEFT,
                                                         "label1", "Global Data", 220);
    Ogre::StringVector paramNames;
    paramNames.push_back("Global Time");
    paramNames.push_back("Simulation Speed");
    OgreBites::ParamsPanel* paramsPanel = tray_manager->createParamsPanel(OgreBites::TL_TOPLEFT,
                                                                          "params_global_time",
                                                                          220,
                                                                          paramNames);
    addInputListener(tray_manager.get());
    create_arrow_mesh();
}

void Renderer::add_engine(Engine &engine, const Vector &offset, const Quaternion &orientation)
{
    engine.add_listener(*this);
    SceneNode* engineNode =
        scene_manager->getRootSceneNode()->createChildSceneNode(node_name_engine(engine));
    engineNode->setPosition(offset[0],
                            offset[1],
                            offset[2]);
    engineNode->setOrientation(orientation.w(), orientation.x(),
                               orientation.y(), orientation.z());

    SceneNode* engine_debug_node =
        engineNode->createChildSceneNode(node_name_engine_debug(engine));

    shared_ptr<BodyDebug> engineDebug = std::make_shared<BodyDebug>(*this,
                                                                  engine_debug_node,
                                                                  scene_manager,
                                                                  camera,
                                                                  engine);
    engine_debug_node->getUserObjectBindings().setUserAny("debug", Ogre::Any(engineDebug));

    std::string paramsName = "params/" + engine.get_name();
    if (!tray_manager->getWidget(paramsName))
    {
        tray_manager->createLabel(OgreBites::TL_TOPLEFT,
                                 "label/" + engine.get_name(),
                                 "Engine " + engine.get_name(), 220);
        Ogre::StringVector paramNames;
        paramNames.push_back("Physics time");
        paramNames.push_back("Used time");
        paramNames.push_back("Kinetic Energy");
        OgreBites::ParamsPanel* paramsPanel = tray_manager->createParamsPanel(OgreBites::TL_TOPLEFT,
                                                                              paramsName,
                                                                              220,
                                                                              paramNames);
    }

    for (auto it = engine.begin(); it != engine.end(); ++it)
    {
        body_added(engine, *it);
    }
}

void Renderer::engine_post_step(Engine &engine)
{

    std::string paramsName = "params/" + engine.get_name();
    OgreBites::ParamsPanel* paramsPanel =
        dynamic_cast<OgreBites::ParamsPanel*>(tray_manager->getWidget(paramsName));
    std::chrono::milliseconds physics_time =
        std::chrono::duration_cast<std::chrono::milliseconds>(engine.get_physics_time());
    std::chrono::milliseconds used_time =
        std::chrono::duration_cast<std::chrono::milliseconds>(engine.get_used_time());
    paramsPanel->setParamValue (0, std::to_string(physics_time.count()));
    paramsPanel->setParamValue (1, std::to_string(used_time.count()));
    std::ostringstream stream_kin_energy;
    stream_kin_energy << std::fixed << std::setprecision(3) << engine.get_kinetic_energy();
    paramsPanel->setParamValue (2, stream_kin_energy.str());


    if (disable_render_bodies)
    {
        // nothing more to do
        return;
    }

    for (auto &state: engine)
    {
        body_updated(engine, state);
    }

    // clear contacts.
    if (scene_manager->hasSceneNode(node_name_contacts(engine)))
    {
        SceneNode * contactNode = static_cast<SceneNode*>
            (scene_manager->getSceneNode(node_name_contacts(engine)));
        destroyAllAttachedMovableObjects(contactNode);
        contactNode->removeAndDestroyAllChildren();
        scene_manager->destroySceneNode(node_name_contacts(engine));
    }

    // recreate contacts.
    if (show_contacts)
    {
        SceneNode *contactNode =
            scene_manager->getSceneNode(node_name_engine(engine))
            ->createChildSceneNode(node_name_contacts(engine));
        auto points = engine.get_last_contact_points();
        for (auto &point : points)
        {
            SceneNode *node = contactNode->createChildSceneNode();
            Ogre::Entity *entityArrow = scene_manager->createEntity("Renderer/ArrowMesh");
            node->attachObject(entityArrow);
            Ogre::Vector3 planeNormal = Ogre::Vector3(point.normal[0],
                                                      point.normal[1],
                                                      point.normal[2]);
            Ogre::Quaternion q = arrow_orientation.getRotationTo(planeNormal);
            node->setOrientation(q);
            node->setPosition(point.point[0], point.point[1], point.point[2]);
        }
    }

}

void Renderer::body_added(Engine& engine, BodyState &state)
{

    if (disable_render_bodies) {
        return;
    }

    if (scene_manager->hasSceneNode (node_name_body_debug(engine, state))) {
        L_(lwarning) << " body already added in engine. nothing to do.";
        return;
    }

    SceneNode* engine_node =
        static_cast<SceneNode*>(scene_manager->getSceneNode(node_name_engine(engine)));

    SceneNode* body_node = engine_node->createChildSceneNode(node_name_body(engine, state));
    const Body& body = state.get_body();

    SceneNode* body_debug_node =
        body_node->createChildSceneNode(node_name_body_debug(engine, state));

    shared_ptr<BodyDebug> bodyDebug = std::make_shared<BodyDebug>(*this,
                                                                  body_debug_node,
                                                                  scene_manager,
                                                                  camera,
                                                                  engine,
                                                                  &state);
    body_debug_node->getUserObjectBindings().setUserAny("debug", Ogre::Any(bodyDebug));

    body_updated(engine, state);
    add_shapes(engine, state);
}

// de https://forums.ogre3d.org/viewtopic.php?t=53647
void Renderer::destroyAllAttachedMovableObjects( SceneNode* node )
{

    // Destroy all the attached objects
    for (auto it : node->getAttachedObjects())
    {
        MovableObject* pObject = static_cast<MovableObject*>(it);
        node->getCreator()->destroyMovableObject( pObject );
    }

    // Recurse to child SceneNodes
    for (auto it: node->getChildren())
    {
        SceneNode* pChildNode = static_cast<SceneNode*>(it);
        destroyAllAttachedMovableObjects( pChildNode );
    }

}

void Renderer::body_deleted(Engine& engine, BodyState &state)
{
    throw std::logic_error("Renderer::body_deleted NOT implemented");

    if (disable_render_bodies) {
        return;
    }

}

void Renderer::body_updated(Engine& engine, BodyState &state)
{
    if (disable_render_bodies) {
        return;
    }

    SceneNode* body_node = scene_manager->getSceneNode(node_name_body(engine, state));
    const Point& pos = state.get_position();
    const Quaternion& ori = state.get_orientation();
    body_node->setOrientation(ori.w(), ori.x(), ori.y(), ori.z());
    body_node->setPosition(pos[0], pos[1], pos[2]);
}


void Renderer::add_shapes(Engine& engine, BodyState &state)
{
    SceneNode* body_node = scene_manager->getSceneNode(node_name_body(engine, state));
    current_material_name = "Obugre/" +  engine.get_name();
    for (auto& shape : state.get_body().get_shapes())
    {
        current_node = body_node->createChildSceneNode();
        const Point& position = shape->get_position();
        current_node->setPosition(position[0], position[1], position[2]);
        const Quaternion& orientation = shape->get_orientation();
        current_node->setOrientation(orientation.w(),orientation.x(),orientation.y(),orientation.z());
        shape->accept_visitor(ShapeVisitorOperation::add, *this, state);
    }

}


bool Renderer::keyPressed(const OgreBites::KeyboardEvent& evt)
{
    L_(linfo) << " keyPressed " << evt.keysym.sym;

    if (evt.keysym.sym == '1') {
        show_body_debug_axis = !show_body_debug_axis;
        return true;
    }
    if (evt.keysym.sym == '2') {
        show_body_debug_text = !show_body_debug_text;
        return true;
    }
    if (evt.keysym.sym == '3') {
        show_body_debug_vectors = !show_body_debug_vectors;
        return true;
    }
    if (evt.keysym.sym == '4') {
        show_engine_debug_axis = !show_engine_debug_axis;
        return true;
    }
    if (evt.keysym.sym == '5') {
        show_engine_debug_text = !show_engine_debug_text;
        return true;
    }
    if (evt.keysym.sym == '6') {
        show_engine_debug_vectors = !show_engine_debug_vectors;
        return true;
    }
    if (evt.keysym.sym == '7') {
        show_contacts = !show_contacts;
        return true;
    }

    if (evt.keysym.sym == '8') {
        ortographic_projection = !ortographic_projection;
        L_(linfo) << "ortographic_projection " << ortographic_projection;
        if (ortographic_projection) {
            camera->setOrthoWindowHeight(ortographic_width);
            camera->setProjectionType(PT_ORTHOGRAPHIC);
        }
        else
        {

            camera->setProjectionType(PT_PERSPECTIVE);
        }
        return true;
    }

    if (evt.keysym.sym == '9') {
        ortographic_width *= 1.1;
        L_(linfo) << "ortographic_width " << ortographic_width;
        camera->setOrthoWindowHeight(ortographic_width);
        return true;
    }

    if (evt.keysym.sym == '0') {
        ortographic_width *= 0.9;
        L_(linfo) << "ortographic_width " << ortographic_width;
        camera->setOrthoWindowHeight(ortographic_width);
        return true;
    }


    // screen shot
    if (evt.keysym.sym == 's')
    {
        getRenderWindow()->writeContentsToTimestampedFile ("renderersnapshot", ".png");
        return true;
    }
    return false;
}

void Renderer::visit_shape(ShapeVisitorOperation operation, BoxShape& box,
                           BodyState& state)
{
    L_(linfo) << "Adding Box";
    SceneNode* shape_node = current_node->createChildSceneNode();
    shape_node->setScale(box.getExtentX() * 0.01,
                         box.getExtentY() * 0.01,
                         box.getExtentZ() * 0.01);
    Entity* entity = scene_manager->createEntity(Ogre::SceneManager::PT_CUBE);
    entity->setMaterialName(current_material_name);
    shape_node->attachObject(entity);
}

void Renderer::visit_shape(ShapeVisitorOperation operation, ConvexShape& convex,
                           BodyState& state)
{
    SceneNode* shape_node = current_node->createChildSceneNode();

    const ConvexPolytope& cv = *convex.get_convex_polytope();
    Ogre::ManualObject* man = scene_manager->createManualObject();

    const FaceList& faces = cv.get_faces();
    for (auto it = faces.begin(); it != faces.end(); ++it)
    {
        const Face& face = *it;
        const Vector& faceNormal = face.get_plane().normal();
        FeatureId heId = face.get_halfedge_id();
        man->begin(current_material_name, Ogre::RenderOperation::OT_TRIANGLE_FAN,
                   //ResourceGroupManager::INTERNAL_RESOURCE_GROUP_NAME
                   ResourceGroupManager::AUTODETECT_RESOURCE_GROUP_NAME
                   );
        do {
            const HalfEdge& he = cv.get_halfedge(heId);

            FeatureId headId = he.get_headvertex_id();
            FeatureId tailId = cv.get_halfedge(he.get_pair_id()).get_headvertex_id();

            const Vertex& tail = cv.get_vertex(tailId);
            const Vertex& head = cv.get_vertex(headId);

            man->position(tail.get_position()[0], tail.get_position()[1], tail.get_position()[2]);
            man->normal(faceNormal[0], faceNormal[1], faceNormal[2]);

            man->position(head.get_position()[0], head.get_position()[1], head.get_position()[2]);
            man->normal(faceNormal[0], faceNormal[1], faceNormal[2]);

            heId = he.get_next_id();
        }while (heId != face.get_halfedge_id());
        man->end();
    }

    shape_node->attachObject(man);
}

void Renderer::visit_shape(ShapeVisitorOperation operation, PlaneShape& shape,
                           BodyState& state)
{
    SceneNode* shape_node = current_node->createChildSceneNode();
    // TODO WHAT IF PLANE IS NOT PLACED AS DEFAULT?
    const OB::Plane &plane = shape.get_plane();

    Ogre::Vector3 defaultPtPlaneNormal{0, 0, 1};
    Ogre::Vector3 planeNormal = Ogre::Vector3(plane.normal()[0],
                                              plane.normal()[1],
                                              plane.normal()[2]);

    Ogre::Quaternion q = defaultPtPlaneNormal.getRotationTo(planeNormal);
    shape_node->setOrientation(q);

    OB::Vector offsetPosition = plane.normal() * plane.offset();
    // translate the plane by the offset.
    shape_node->setPosition(offsetPosition[0], offsetPosition[1], offsetPosition[2]);

    Entity* entity = scene_manager->createEntity(SceneManager::PT_PLANE);
    entity->setMaterialName(current_material_name);
    shape_node->attachObject(entity);
    Real scaleFactor = 0.5;
    shape_node->setScale(scaleFactor, scaleFactor, scaleFactor);
}

void Renderer::visit_shape(ShapeVisitorOperation operation, SphereShape& shape,
                           BodyState& state)
{
    SceneNode* shape_node = current_node->createChildSceneNode();
    Real scaleFactor = shape.get_radius() * 0.02;
    shape_node->setScale(scaleFactor, scaleFactor, scaleFactor);
    Entity* entity = scene_manager->createEntity(Ogre::SceneManager::PT_SPHERE);
    entity->setMaterialName(current_material_name);
    shape_node->attachObject(entity);
}


bool Renderer::renderOneFrame(std::chrono::microseconds global_time,
                              Real simulation_speed)
{
    Ogre::WindowEventUtilities::messagePump();
    std::chrono::milliseconds milliseconds =
        std::chrono::duration_cast<std::chrono::milliseconds>(global_time);

    OgreBites::ParamsPanel* paramsPanelGlobal =
        dynamic_cast<OgreBites::ParamsPanel*>(tray_manager->getWidget("params_global_time"));

    paramsPanelGlobal->setParamValue (0, std::to_string(milliseconds.count()));
    std::ostringstream stream_speed;
    stream_speed << std::fixed << std::setprecision(3) << simulation_speed;
    paramsPanelGlobal->setParamValue (1, stream_speed.str());
    return getRoot()->renderOneFrame();
}

void Renderer::create_arrow_mesh()
{
    arrow_orientation = Ogre::Vector3(1,0,0);
    String meshName = "Renderer/ArrowMesh";
    Ogre::MeshPtr mArrowMeshPtr = MeshManager::getSingleton().getByName(meshName);
    if (!mArrowMeshPtr)
    {
        ManualObject mo("");
        mo.begin("Obugre/arrow_normal", RenderOperation::OT_TRIANGLE_LIST,
                 ResourceGroupManager::AUTODETECT_RESOURCE_GROUP_NAME);
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
