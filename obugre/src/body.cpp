#include "body.hpp"

using namespace OB;


HingeConstraint::HingeConstraint(const std::string& name,
                                 const BodyState &bodystate1,
                                 const BodyState &bodystate2,
                                 const Vector& axis_global_coord,
                                 const Vector& point_global_coord)
    : Constraint(name, bodystate1.get_body(), bodystate2.get_body())
{
    // we want to go from world coordinates to body coordinates

    const Body& body1 = bodystate1.get_body();
    const Body& body2 = bodystate2.get_body();

    Transform toBody1FromWorld = bodystate1.get_pose().inverse(Isometry);
    Transform toBody2FromWorld = bodystate2.get_pose().inverse(Isometry);

    Vector axisNormalized = axis_global_coord.normalized();
    axis_local_1 = toBody1FromWorld.linear() * axisNormalized;
    point_local_1 = toBody1FromWorld * point_global_coord;
    axis_local_2 = toBody2FromWorld.linear() * axisNormalized;
    point_local_2 = toBody2FromWorld * point_global_coord;
}

BoxShape& Body::add_box_shape(const Vector& extents)
{
    std::unique_ptr<Shape> shape = std::unique_ptr<Shape>(new BoxShape(extents));
    shapes.push_back(std::move(shape));
    return dynamic_cast<BoxShape&>(*(shapes.back()));
}

PlaneShape& Body::add_plane_shape(const Plane& plane)
{
    std::unique_ptr<Shape> shape = std::unique_ptr<Shape>(new PlaneShape(plane));
    shapes.push_back(std::move(shape));
    return dynamic_cast<PlaneShape&>(*(shapes.back()));
}


SphereShape& Body::add_sphere_shape(Real radius)
{
    std::unique_ptr<Shape> shape = std::unique_ptr<Shape>(new SphereShape(radius));
    shapes.push_back(std::move(shape));
    return dynamic_cast<SphereShape&>(*(shapes.back()));
}

ConvexShape& Body::add_convex_shape(const std::vector<Point>& points)
{
    std::unique_ptr<Shape> shape = std::unique_ptr<Shape>(new ConvexShape(points));
    shapes.push_back(std::move(shape));
    ConvexShape& convex_shape = dynamic_cast<ConvexShape&>(*(shapes.back()));
    convex_shape.generate_Ode_data();
    return convex_shape;
}


void BodyState::set_position(const Point& newPos)
{
    position = newPos;
}

void BodyState::set_orientation(const Quaternion& newOri)
{
    orientation = newOri;
}

void BodyState::set_linear_velocity(const Vector& newLinearVelocity)
{
    assert(!get_body().is_static());
    linear_velocity = newLinearVelocity;
}

void BodyState::set_angular_velocity(const Vector& newAngularVelocity)
{
    assert(!get_body().is_static());
    angular_velocity = newAngularVelocity;
}

Vector BodyState::get_linear_momentum() const
{
  // p = m * v
  return get_linear_velocity() * body.get_mass();
}


Vector BodyState::get_angular_momentum() const
{
  // should we do it in the inertial frame or not???
  // let's try in the inertial frame
  Matrix3 localInertia = body.get_inertia_tensor().asDiagonal();
  //spdlog::info("localInertia {}", localInertia);
  //spdlog::info("orientation quaternion {}", getOrientation());
  Matrix3 rotation = get_orientation().toRotationMatrix();
  //spdlog::info("rotation {}", rotation);
  // similarity transformation
  Matrix3 worldInertia = rotation * localInertia * rotation.transpose();
  //spdlog::info("worldInertia {}", worldInertia);
  Vector worldAngularVelocity = get_angular_velocity();
  return worldInertia * worldAngularVelocity;
  // throw std::logic_error("not implemented");
}

Real BodyState::get_kinetic_energy() const
{
    Real linearPart = 0.5 * body.get_mass() * get_linear_velocity().dot(get_linear_velocity());

  // too many calculations...
  Matrix3 localInertia = body.get_inertia_tensor().asDiagonal();
  Matrix3 rotation = get_orientation().toRotationMatrix();
  Matrix3 worldInertia = rotation * localInertia *rotation.transpose();

  Vector worldAngularVelocity = get_angular_velocity();
  Real rotationalPart = (worldAngularVelocity.dot(worldInertia * worldAngularVelocity)) * 0.5;

  return (linearPart + rotationalPart);
}

Vector BodyState::get_angular_momentum_world_origin() const
{
  return get_position().cross(get_linear_momentum()) + get_angular_momentum();
}

void ConvexShape::generate_Ode_data()
{
    if (_points.size() > 0) return; // already generated

    for (const auto& vertex: convex_polytope->get_vertices())
    {
        const Point& point = vertex.get_position();
        _points.push_back(static_cast<dReal>(point[0]));
        _points.push_back(static_cast<dReal>(point[1]));
        _points.push_back(static_cast<dReal>(point[2]));
    }

    for (FeatureId i = 0; i < convex_polytope->get_faces().size(); ++i)
    {
        const Face& face = convex_polytope->get_faces()[i];
        Feature faceid = Feature{i, FeatureType::face};
        const Plane& plane = face.get_plane();

        _planes.push_back(static_cast<dReal>(plane.normal()[0]));
        _planes.push_back(static_cast<dReal>(plane.normal()[1]));
        _planes.push_back(static_cast<dReal>(plane.normal()[2]));
        _planes.push_back(static_cast<dReal>(plane.offset()));

        const std::vector<Feature> vertices = convex_polytope->get_face_vertices(faceid);
        _polygons.push_back(static_cast<unsigned int>(vertices.size()));
        for (const Feature& fv : vertices)
        {
            _polygons.push_back(static_cast<unsigned int>(fv.feature));
        }
    }

    // one int for each polygon for the number of vertices.
    // one int for each vertex in the polygon
    // and this way for every polygon (face)
}

//got it from here https://en.wikipedia.org/wiki/List_of_moments_of_inertia
Vector OB::get_inertia_tensor_box(const Vector& extents, Real mass)
{
    return Vector(
                  mass/12.0 * (extents[1]*extents[1] + extents[2]*extents[2]),
                  mass/12.0 * (extents[0]*extents[0] + extents[2]*extents[2]),
                  mass/12.0 * (extents[0]*extents[0] + extents[1]*extents[1])
                  );
}

Vector OB::get_inertia_tensor_sphere(Real radius, Real mass)
{
    Real value = 0.4*mass*radius*radius;
    return Vector(value, value, value);
}

Vector OB::get_inertia_tensor_spheric_shell(Real radius_big, Real radius_small, Real mass)
{
    Real value = 0.4*mass* ((pow(radius_big,5)-pow(radius_small,5))/(pow(radius_big,3)-pow(radius_small,3)));
    return Vector(value, value, value);
}

std::ostream& OB::operator<< (std::ostream &out, const BodyState& state)
{
    out << "{BodyState position: " << state.get_position().transpose()
        << " orientation: " << state.get_orientation() << "}";
    return out;
}
