#ifndef OBUGRE_BODY_HPP
#define OBUGRE_BODY_HPP


#include <vector>
#include <map>
#include <memory>
#include <string>
#include <obconfig.hpp>

#include "polytope.hpp"

// this is for polluting all with ode macros (to get the right dReal size)...
#include "ode/common.h"

namespace OB
{
    class Body;
    class BodyState;
    class Shape;
    class BoxShape;
    class ConvexShape;
    class PlaneShape;
    class SphereShape;
    class HingeConstraint;

    enum class ConstraintVisitorOperation {add };

    class ConstraintVisitor
    {
    public:
        virtual void visit_constraint(ConstraintVisitorOperation operation,
                                      HingeConstraint& constraint) = 0;
    };


    class Constraint
    {
    public:
        Constraint(const std::string& name,
                   const Body& body1,
                   const Body& body2)
            : name(name), body1(&body1), body2(&body2) {}

        Constraint(const Constraint&) = delete;
        Constraint& operator=(const Constraint&) = delete;

        virtual void accept_visitor(ConstraintVisitorOperation operation,
                                    ConstraintVisitor& visitor) = 0;

        const Body& get_body1() {return *body1;};
        const Body& get_body2() {return *body2;};

        const std::string& get_name() { return name; }
    private:
        const std::string name;
        const Body *body1 = nullptr;
        const Body *body2 = nullptr;

    };

    class HingeConstraint: public Constraint
    {
    public:
        //in global coordinates, so as in ODE, bodies must be correctly positioned.
        HingeConstraint(const std::string& name,
                        const BodyState &bodystate1,
                        const BodyState &bodystate2,
                        const Vector& axis_global_coord,
                        const Vector& point_global_coord);

        const Vector& get_axis_local_body1() { return axis_local_1; }
        const Point& get_point_local_body1() { return point_local_1; }
        const Vector& get_axis_local_body2() { return axis_local_2; }
        const Point& get_point_local_body2() { return point_local_2; }


        void accept_visitor(ConstraintVisitorOperation operation,
                            ConstraintVisitor& visitor) override
        {
            { visitor.visit_constraint(operation, *this); }
        }
    private:
        Vector axis_local_1;
        Point point_local_1;
        Vector axis_local_2;
        Point point_local_2;
    };

    class BodyState
    {
    public:
        BodyState(Body& body, const Point& position = Point::Zero())
            :body(body), position(position) {}


        Point get_position() const  {return position;}
        Quaternion get_orientation() const {return orientation;}
        Vector get_linear_velocity() const {return linear_velocity;}
        Vector get_angular_velocity() const  {return angular_velocity;}
        Vector get_angular_momentum_world_origin() const;

        Vector get_linear_momentum() const;
        Vector get_angular_momentum() const;
        Real get_kinetic_energy() const;

        const Body& get_body() const { return body; }

        Transform get_pose() const {
            return OB::Translation(get_position()) * get_orientation().toRotationMatrix();
        }

        void set_position(const Point& newPos);
        void set_orientation(const Quaternion& newOri);
        void set_linear_velocity(const Vector& newLinearVelocity);
        void set_angular_velocity(const Vector& newAngularVelocity);

    private:
        Body& body;
        Point position = Point::Zero();
        Quaternion orientation = Quaternion::Identity();
        Vector linear_velocity = Point::Zero();
        Vector angular_velocity = Point::Zero();
    };

    std::ostream& operator<< (std::ostream &out, const BodyState& state);

    class Body
    {
    public:
        Body(const std::string& new_name):name(new_name), initial_state(*this) {}

        Body(const Body&) = delete;
        Body& operator=(const Body&) = delete;

        void set_mass(Real new_mass) { mass = new_mass; }
        Real get_mass() const { return mass; }

        void set_inertia_tensor(Vector new_inertia) { inertia_tensor = new_inertia; };
        const Vector& get_inertia_tensor() const { return inertia_tensor; }

        bool is_static() const { return static_body; }
        void set_static(bool newStatic)  { static_body = newStatic; }

        BoxShape& add_box_shape(const Vector& extents);
        PlaneShape& add_plane_shape(const Plane& plane);
        SphereShape& add_sphere_shape(Real radius);
        ConvexShape& add_convex_shape(const std::vector<Point>& points);

        const std::string& get_name() const { return name; }

        const BodyState& get_initial_state() const { return initial_state; }
        BodyState& get_initial_state() { return initial_state; }

        using ShapeVectorOwning = std::vector<std::unique_ptr<Shape>>;
        const ShapeVectorOwning& get_shapes() const { return shapes; }
    private:
        const std::string name;
        BodyState initial_state;
        Real mass = 0.0;
        Vector inertia_tensor{1.0, 1.0, 1.0};
        bool static_body = false;
        ShapeVectorOwning shapes;
    };

    enum class ShapeVisitorOperation {add };

    class ShapeVisitor
    {
    public:
        virtual void visit_shape(ShapeVisitorOperation operation,
                                 BoxShape& shape, BodyState& state) = 0;
        virtual void visit_shape(ShapeVisitorOperation operation,
                                 ConvexShape& shape, BodyState& state) = 0;
        virtual void visit_shape(ShapeVisitorOperation operation,
                                 PlaneShape& plane, BodyState& state) = 0;
        virtual void visit_shape(ShapeVisitorOperation operation,
                                 SphereShape& sphere, BodyState& state) = 0;
    };


    class Shape
    {
    public:
        virtual ~Shape() = default;
        virtual void accept_visitor(ShapeVisitorOperation operation,
                                    ShapeVisitor& visitor, BodyState& state) = 0;

        const Point& get_position() const { return position; }
        const Quaternion& get_orientation() const { return orientation; }

        void set_position(const Point& newPosition) { position = newPosition; }
        void set_orientation(const Quaternion& newOrientation) { orientation = newOrientation; }

    protected:
        Shape() = default;
        Shape(const Point &position, const Quaternion& orientation = Quaternion::Identity())
            :position(position), orientation(orientation) {}

    private:
        Point position = Point::Zero();
        Quaternion orientation = Quaternion::Identity();

    };


    class BoxShape : public Shape
    {
    public:

        BoxShape(Real extentX, Real extentY, Real extentZ)
        { extents = Vector{extentX, extentY, extentZ}; }

        BoxShape(const Vector& extents):extents(extents) {}

        void accept_visitor(ShapeVisitorOperation operation,
                            ShapeVisitor& visitor, BodyState& state) override
        { visitor.visit_shape(operation, *this, state); }

        Real getExtentX() const { return extents[0]; }
        Real getExtentY() const { return extents[1]; }
        Real getExtentZ() const { return extents[2]; }
    private:
        Vector extents;
    };

    class ConvexShape : public Shape
    {
    public:

        ConvexShape(const std::vector<Point>& points) {
            convex_polytope = std::make_shared<ConvexPolytope>();
            convex_polytope->generate_from_vertices(points);
            assert(convex_polytope->euler_number_correct());
        }

        ~ConvexShape() = default;

        void accept_visitor(ShapeVisitorOperation operation,
                            ShapeVisitor& visitor, BodyState& state) override
        { visitor.visit_shape(operation, *this, state); }

        const std::shared_ptr<ConvexPolytope>& get_convex_polytope() const
        { return convex_polytope; }


        void generate_Ode_data();
        const dReal* get_Ode_planes() const { return _planes.data(); };
        unsigned int get_Ode_plane_count() const {
            return static_cast<unsigned int>(_planes.size() / 4); }
        const dReal* get_Ode_points() const { return _points.data(); }
        unsigned int get_Ode_point_count() const {
            return static_cast<unsigned int>(_points.size() / 3); }
        const unsigned int* get_Ode_polygons() const  { return _polygons.data(); }


    private:
        std::shared_ptr<ConvexPolytope> convex_polytope;

        // I store this here for ode (dCreateConvex)
        std::vector<dReal> _planes;
        std::vector<dReal> _points;
        std::vector<unsigned int> _polygons;
    };

    class PlaneShape : public Shape
    {
    public:
        PlaneShape() = default;
        PlaneShape(const Plane& plane): plane(plane) {}

        void accept_visitor(ShapeVisitorOperation operation,
                            ShapeVisitor& visitor, BodyState& state) override
        { visitor.visit_shape(operation, *this, state); }

        const Plane& get_plane() { return plane; }
    private:
        Plane plane{Vector{0, 1, 0}, 0}; // up Y plane
    };

    class SphereShape : public Shape
    {
    public:
        SphereShape(Real radius) : radius(radius) {}

        void accept_visitor(ShapeVisitorOperation operation,
                            ShapeVisitor& visitor, BodyState& state) override
        { visitor.visit_shape(operation, *this, state); }
        const Real& get_radius() const { return radius; }

    private:
        Real radius;
    };


    // really a "rectangular cuboid"
    Vector get_inertia_tensor_box(const Vector& extents, Real mass);
    Vector get_inertia_tensor_sphere(Real radius, Real mass);
    Vector get_inertia_tensor_spheric_shell(Real radius_big, Real radius_small, Real mass);

}

#endif
