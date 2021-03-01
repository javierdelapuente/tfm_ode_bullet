#ifndef OBUGRE_RENDERER_HPP
#define OBUGRE_RENDERER_HPP

#include <OgreApplicationContext.h>
#include <OgreCameraMan.h>
#include <OgreTrays.h>

#include "body.hpp"
#include "engine.hpp"

namespace OB
{


    class Renderer : public OgreBites::ApplicationContext,
                     public OgreBites::InputListener,
                     public EngineListener,
                     public ShapeVisitor
    {
    public:

        Renderer(bool disable_render_bodies = false)
            :disable_render_bodies(disable_render_bodies) {}


        void shutdown() override;
        bool keyPressed(const OgreBites::KeyboardEvent& evt) override;
        bool renderOneFrame(std::chrono::microseconds global_time,
                            Real simulation_speed);

        void add_engine(Engine &engine, const Vector &offset, const Quaternion &orientation);
        void engine_pre_step(Engine &engine) override {}
        void engine_post_step(Engine &engine) override;
        void body_added(Engine& engine, BodyState &state) override;
        void body_deleted(Engine& engine, BodyState &state) override;
        void body_updated(Engine& engine, BodyState &state);

        std::string node_name_engine(Engine& engine)
        { return "%engine%" + engine.get_name(); }

        std::string node_name_engine_debug(Engine& engine)
        { return "%engine%" + engine.get_name() + "%debug%"; }


        std::string node_name_body(Engine& engine, BodyState& state)
        { return "%engine%" + engine.get_name() + "%state%" + state.get_body().get_name(); }

        std::string node_name_body_debug(Engine& engine, BodyState& state)
        { return "%engine%" + engine.get_name() + "%debug%" + state.get_body().get_name(); }

        std::string node_name_contacts(Engine& engine)
        { return "%engine%" + engine.get_name() + "%contacts"; }

        std::string node_name_constraint(Engine& engine, Constraint &constraint)
        { return "%engine%" + engine.get_name() + "%constraints%" + constraint.get_name(); }

        void visit_shape(ShapeVisitorOperation operation, BoxShape& shape,
                         BodyState& state) override;
        void visit_shape(ShapeVisitorOperation operation, ConvexShape& shape,
                         BodyState& state) override;
        void visit_shape(ShapeVisitorOperation operation, PlaneShape& shape,
                         BodyState& state) override;
        void visit_shape(ShapeVisitorOperation operation, SphereShape& shape,
                         BodyState& state) override;

        bool get_show_body_debug_axis() { return show_body_debug_axis; }
        bool get_show_body_debug_vectors() { return show_body_debug_vectors; }
        bool get_show_body_debug_text() { return show_body_debug_text; }
        bool get_show_engine_debug_axis() { return show_engine_debug_axis; }
        bool get_show_engine_debug_vectors() { return show_engine_debug_vectors; }
        bool get_show_engine_debug_text() { return show_engine_debug_text; }

    private:
        void setup() override;

        void destroyAllAttachedMovableObjects( Ogre::SceneNode* node );

        std::unique_ptr<OgreBites::CameraMan> camera_man;
        std::unique_ptr<OgreBites::TrayManager> tray_manager;

        // managed by ogre
        Ogre::SceneManager* scene_manager;
        Ogre::Camera* camera;
        Ogre::SceneNode* current_node;
        std::string current_material_name;

        bool vsync_enabled = false;

        void add_shapes(Engine& engine, BodyState &state);

        void create_arrow_mesh();
        Ogre::Vector3 arrow_orientation;

        bool show_body_debug_axis = true;
        bool show_body_debug_vectors = true;
        bool show_body_debug_text = true;
        bool show_engine_debug_axis = true;
        bool show_engine_debug_vectors = true;
        bool show_engine_debug_text = true;
        bool show_contacts = false;

        bool disable_render_bodies = false;

        bool ortographic_projection = false;
        Real ortographic_width = 100;

    };


}

#endif
