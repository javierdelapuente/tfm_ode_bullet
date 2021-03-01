#include <map>
#include <ostream>
#include <iostream>
#include <algorithm>
#include <fstream>

#include "polytope.hpp"

#include "libqhullcpp/RboxPoints.h"
#include "libqhullcpp/QhullError.h"
#include "libqhullcpp/QhullQh.h"
#include "libqhullcpp/QhullFacet.h"
#include "libqhullcpp/QhullFacetList.h"
#include "libqhullcpp/QhullFacetSet.h"
#include "libqhullcpp/QhullLinkedList.h"
#include "libqhullcpp/QhullPoint.h"
#include "libqhullcpp/QhullUser.h"
#include "libqhullcpp/QhullVertex.h"
#include "libqhullcpp/QhullVertexSet.h"
#include "libqhullcpp/Qhull.h"

#include "clip_utils.hpp"

#undef min
#undef max

using namespace OB;

Real OB::dist_point_point(const Point& point1, const Point& point2)
{
    return (point1 - point2).norm();
}

Real OB::dist_point_line(const Point& point, const EdgePoints& edge)
{
    Vector edge_vector = edge.vector();
    Real edge_length = (edge_vector).norm();
    Real inv_edge_length = (1.0 / edge_length);
    Vector unit_edge_vector = edge_vector * inv_edge_length;
    Vector tail_point_vec = point - edge.tail;
    Real t = unit_edge_vector.dot(tail_point_vec) * inv_edge_length;
    Vector nearest_point_in_line = edge.tail + edge_vector * t;
    return (nearest_point_in_line - point).norm();
}

Real OB::absdist_point_plane(const Point& point, const Plane& plane)
{
    return plane.absDistance(point);
}

Real OB::dist_line_line(const EdgePoints& edge1, const EdgePoints& edge2)
{
    //check if they are parallel, in case there is not a perpendicular vector between the edges
    Vector edge_vector1 = edge1.vector();
    Vector edge_vector2 = edge2.vector();
    Real edge_length1 = (edge_vector1).norm();
    Real edge_length2 = (edge_vector2).norm();
    Vector unit_edge_vector1 = edge_vector1 * (1.0/edge_length1);
    Vector unit_edge_vector2 = edge_vector2 * (1.0/edge_length2);
    Real unit_edge_directions_dotproduct = unit_edge_vector1.dot(unit_edge_vector2);
    if (is_zero(abs(unit_edge_directions_dotproduct) - 1.0))
    {
        // parallel edges
        return dist_point_line(edge1.head, edge2);
    }
    Vector vector_tail1_tail2 = edge2.tail - edge1.tail;
    Real t = (vector_tail1_tail2.dot(unit_edge_vector1) - (vector_tail1_tail2.dot(unit_edge_vector2))* (unit_edge_directions_dotproduct)) / ( 1 - unit_edge_directions_dotproduct*unit_edge_directions_dotproduct);
    Real s = - (vector_tail1_tail2.dot(unit_edge_vector2) - (vector_tail1_tail2.dot(unit_edge_vector1))* (unit_edge_directions_dotproduct)) / ( 1 - unit_edge_directions_dotproduct*unit_edge_directions_dotproduct);
    Real t_real = t / (1.0 * edge_length1);
    Real s_real = s / (1.0 * edge_length2);
    return ((edge1.tail + t * unit_edge_vector1) - (edge2.tail + s * unit_edge_vector2)).norm();
}


Real OB::dist_point_edge(const Point& point, const EdgePoints& edge)
{
    //from https://stackoverflow.com/questions/27161533/find-the-shortest-distance-between-a-point-and-line-segments-not-line
    Vector edge_vector = edge.vector();
    Real edge_length = (edge_vector).norm();
    Real inv_edge_length = (1.0/edge_length);
    Vector unit_edge_vector = edge_vector * inv_edge_length;
    Vector tail_point_vec = point - edge.tail;
    Real t = unit_edge_vector.dot(tail_point_vec) * inv_edge_length;
    t = std::max(static_cast<Real>(0.0),
                 std::min(t, static_cast<Real>(1.0))); // clip between 0.0 and 1.0
    Vector nearest_point_in_line = edge.tail + edge_vector * t;
    return (nearest_point_in_line - point).norm();
}


Real OB::dist_edge_edge(const EdgePoints& edge1, const EdgePoints& edge2)
{
    // this is quite slow. for a faster version see something like
    // https://www.geometrictools.com/Documentation/DistanceLine3Line3.pdf
    // For line to line distance:
    // https://homepage.univie.ac.at/Franz.Vesely/notes/hard_sticks/hst/hst.html
    Vector edge_vector1 = edge1.vector();
    Vector edge_vector2 = edge2.vector();
    Real edge_length1 = (edge_vector1).norm();
    Real edge_length2 = (edge_vector2).norm();
    Vector unit_edge_vector1 = edge_vector1 * (1.0 / edge_length1);
    Vector unit_edge_vector2 = edge_vector2 * (1.0 / edge_length2);

    // edge1 line parametrically is: edge1(t) = edge1.head + unit_edge_vector1 * t
    // edge2 line parametrically is: edge2(s) = edge2.head + unit_edge_vector2 * s
    Vector vector_tail1_tail2 = edge2.tail - edge1.tail;

    Real unit_edge_directions_dotproduct = unit_edge_vector1.dot(unit_edge_vector2);
    Real t = (vector_tail1_tail2.dot(unit_edge_vector1) - (vector_tail1_tail2.dot(unit_edge_vector2)) * (unit_edge_directions_dotproduct)) / (1 - unit_edge_directions_dotproduct * unit_edge_directions_dotproduct);
    Real s = -(vector_tail1_tail2.dot(unit_edge_vector2) - (vector_tail1_tail2.dot(unit_edge_vector1)) * (unit_edge_directions_dotproduct)) / (1 - unit_edge_directions_dotproduct * unit_edge_directions_dotproduct);

    // if the minimum distance between the lines are in points of both segments, we are done.
    Real t_real = t / (1.0 * edge_length1);
    Real s_real = s / (1.0 * edge_length2);

    if (t_real >= 0.0 && t_real <= 1.0 && s_real >= 0.0 && s_real <= 1.0)
    {
        return (vector_tail1_tail2 + s * unit_edge_vector2 - t * unit_edge_vector1).norm();
    }

    // if they are not, we have to get the minimum from all the combinations:
    // I don't test point-point as our edges are not open sets.
    return std::min({dist_point_edge(edge1.head, edge2),
                     dist_point_edge(edge1.tail, edge2),
                     dist_point_edge(edge2.head, edge1),
                     dist_point_edge(edge2.tail, edge1) });
}

Real OB::absdist_point_face(const Point& point,
    const Plane& face_plane,
    const std::vector<EdgePoints>& face_edgesccw)
{
    // First I check if the face closest point to the point is
    // in the plane of the face. Otherwise it is a edge-point problem.
    bool point_closest_to_plane = true;
    for (const auto& edge : face_edgesccw)
    {
        Plane planePointingToFace{ face_plane.normal().cross(edge.vector()).normalized(),
                                  edge.tail };
        if (planePointingToFace.signedDistance(point) < 0) {
            point_closest_to_plane = false;
            break;
        }
    }

    if (point_closest_to_plane) {
        return face_plane.absDistance(point);
    }

    Real distance = std::numeric_limits<Real>::max();
    for (const auto& edge : face_edgesccw)
    {
        Real pe_distance = dist_point_edge(point, edge);
        if (pe_distance < distance)
            distance = pe_distance;
    }
    return distance;
}


Real OB::absdist_edge_face(const EdgePoints& edge_points,
    const Plane& face_plane,
    const std::vector<EdgePoints>& face_edgesccw)
{
    // I use vclip trick of clipedge to check if points in closed to the
    // plane if needed.

    // create all face voronoi planes.
    std::vector<VoronoiPlane> vps;
    for (const EdgePoints& fep : face_edgesccw)
    {
        Plane p{ face_plane.normal().cross(fep.vector()).normalized(),
                fep.head };
        VoronoiPlane vp(p, Feature());
        vps.push_back(vp);
    }

    Real distance = std::numeric_limits<Real>::max();
    ClipEdge ce = clipedge(edge_points, vps.begin(), vps.end());
    if (!ce.excluded())
    {
        Vector point_lambda_t = edge_points.point_at_lambda(ce.lambda_t);
        Vector point_lambda_h = edge_points.point_at_lambda(ce.lambda_h);
        distance = std::min(distance, absdist_point_plane(point_lambda_t, face_plane));
        distance = std::min(distance, absdist_point_plane(point_lambda_h, face_plane));
        if (face_plane.signedDistance(point_lambda_t) * face_plane.signedDistance(point_lambda_h) < 0)
        {
            distance = 0;
            return distance;
        }
    }

    // this is not necessary if edge totally inside face
    for (const EdgePoints& fep : face_edgesccw)
    {
        distance = std::min(distance, dist_edge_edge(edge_points, fep));
    }

    return distance;
}


Real absdist_between_features_ordered(const WorldConvexPolytope& wcp1, const Feature& f1,
    const WorldConvexPolytope& wcp2, const Feature& f2)
{

    Transform to1_from2(wcp1.get_pose().inverse(Isometry) * wcp2.get_pose());
    Transform to2_from1(to1_from2.inverse(Isometry));

    const ConvexPolytope& pol1 = wcp1.convex_polytope();
    const ConvexPolytope& pol2 = wcp2.convex_polytope();

    if (f1.type == FeatureType::vertex && f2.type == FeatureType::vertex) {
        Point p1 = to2_from1 * pol1.get_vertex_point(f1);
        Point p2 = pol2.get_vertex_point(f2);
        return dist_point_point(p1, p2);
    }
    if (f1.type == FeatureType::vertex && f2.type == FeatureType::edge) {
        Point p1 = to2_from1 * pol1.get_vertex_point(f1);
        EdgePoints ep = pol2.get_edgepoints(f2);
        return dist_point_edge(p1, ep);
    }

    if (f1.type == FeatureType::vertex && f2.type == FeatureType::face) {
        Point p1 = to2_from1 * pol1.get_vertex_point(f1);
        std::vector<EdgePoints> edgePointsFace = pol2.get_face_edgepoints(f2);
        Plane facePlane = pol2.get_face_plane(f2);
        return absdist_point_face(p1,
            facePlane,
            edgePointsFace);
    }

    if (f1.type == FeatureType::edge && f2.type == FeatureType::edge) {
        EdgePoints e1 = pol1.get_edgepoints(f1);
        e1.tail = to2_from1 * e1.tail;
        e1.head = to2_from1 * e1.head;
        EdgePoints e2 = pol2.get_edgepoints(f2);
        return dist_edge_edge(e1, e2);
    }

    if (f1.type == FeatureType::edge && f2.type == FeatureType::face) {
        EdgePoints e1 = pol1.get_edgepoints(f1);
        e1.tail = to2_from1 * e1.tail;
        e1.head = to2_from1 * e1.head;
        std::vector<EdgePoints> edge_points_face = pol2.get_face_edgepoints(f2);
        Plane facePlane = pol2.get_face_plane(f2);
        return absdist_edge_face(e1, facePlane, edge_points_face);
    }

    throw std::logic_error("Distance between those features not implemented");
}


Real OB::absdist_between_features(const WorldConvexPolytope& wcp1, const Feature& f1,
    const WorldConvexPolytope& wcp2, const Feature& f2)
{
    if (f1.type > f2.type) {
        return absdist_between_features_ordered(wcp2, f2, wcp1, f1);
    }
    else {
        return absdist_between_features_ordered(wcp1, f1, wcp2, f2);
    }
}

std::pair<bool, Point> OB::intersect_line_plane(const EdgePoints& edgePoints,
    const Plane& facePlane)
{
    Vector edge_vector = edgePoints.vector();
    Vector edge_vector_normalized = edge_vector.normalized();
    // check line parallel
    if (is_zero(facePlane.normal().dot(edge_vector_normalized))) {
        // check line in plane.
        if (is_zero(facePlane.offset() + edgePoints.tail.dot(facePlane.normal())))
        {
            auto resp = std::pair<bool, Vector>(false, edgePoints.tail);
            return resp;
        }
        return std::make_pair(true, Vector::Zero());
    }

    Real d =
        (-facePlane.offset() - edgePoints.tail.dot(facePlane.normal())) /
        facePlane.normal().dot(edge_vector_normalized);
    Vector point = edgePoints.tail + d * edge_vector_normalized;
    return std::pair<bool, Vector>(true, point);
}

Point OB::closest_point_to_line(const Point& point, const EdgePoints& edge)
{
    Vector edge_vector = edge.vector();
    Real edge_length = (edge_vector).norm();
    Real inv_edge_length = (1.0/edge_length);
    Vector unit_edge_vector = edge_vector * inv_edge_length;
    Vector tail_point_vec = point - edge.tail;
    Real t = unit_edge_vector.dot(tail_point_vec) * inv_edge_length;
    Point nearest_point_in_line = edge.tail + edge_vector * t;
    return nearest_point_in_line;
}

std::pair<Point, Point> OB::closests_line_to_line(const EdgePoints& edge1, const EdgePoints& edge2)
{
    // see dist_edge_edge. it is the case of line line.
    std::pair<Point, Point> resp;

    //check if they are parallel, in case there is not a perpendicular vector between the edges
    Vector edge_vector1 = edge1.vector();
    Vector edge_vector2 = edge2.vector();
    Real edge_length1 = (edge_vector1).norm();
    Real edge_length2 = (edge_vector2).norm();
    Vector unit_edge_vector1 = edge_vector1 * (1.0/edge_length1);
    Vector unit_edge_vector2 = edge_vector2 * (1.0/edge_length2);

    Real unit_edge_directions_dotproduct = unit_edge_vector1.dot(unit_edge_vector2);

    if (is_zero(abs(unit_edge_directions_dotproduct) - 1.0))
    {
        Point p1 = edge1.head;
        Point p2 = closest_point_to_line(p1, edge2);
        resp.first = p1;
        resp.second = p2;
        return resp;
    }
    Vector vector_tail1_tail2 = edge2.tail - edge1.tail;
    Real t = (vector_tail1_tail2.dot(unit_edge_vector1) - (vector_tail1_tail2.dot(unit_edge_vector2))* (unit_edge_directions_dotproduct)) / ( 1 - unit_edge_directions_dotproduct*unit_edge_directions_dotproduct);
    Real s = - (vector_tail1_tail2.dot(unit_edge_vector2) - (vector_tail1_tail2.dot(unit_edge_vector1))* (unit_edge_directions_dotproduct)) / ( 1 - unit_edge_directions_dotproduct*unit_edge_directions_dotproduct);
    Real t_real = t / (1.0 * edge_length1);
    Real s_real = s / (1.0 * edge_length2);

    resp.first = edge1.tail + t * unit_edge_vector1;
    resp.second = edge2.tail + s * unit_edge_vector2;
    return resp;
}



Point Face::get_center(const ConvexPolytope& cv) const
{
    int num_points = 0;
    Point center{0,0,0};
    FeatureId heid = halfedge_id;
    do {
        const HalfEdge& he = cv.get_halfedge(heid);
        const Vertex&ve = cv.get_vertex(he.get_headvertex_id());

        center += ve.get_position();
        num_points++;

        heid = he.get_next_id();
    } while (heid != halfedge_id);

    center = center * (1.0/num_points);
    return center;
}


void ConvexPolytope::generate_from_vertices(const std::vector<Point>& points)
{
    if (generated) {
        throw std::logic_error("ConvexPolytope::generate_from_vertices "
                               "HalfEdge Structure already generated");
    }

    int number_of_points = points.size();
    // be careful. a non constexpr array is not c++!!
    //realT point_coords[number_of_points*3];
    std::vector<realT> point_coords(number_of_points * 3);

    for (size_t i = 0; i < points.size(); i++)
    {
        const Point& p = points[i];
        point_coords[i*3+0] = static_cast<realT>(p(0));
        point_coords[i*3+1] = static_cast<realT>(p(1));
        point_coords[i*3+2] = static_cast<realT>(p(2));
    }

    orgQhull::Qhull q {"", 3, number_of_points, point_coords.data(), "o"};

    // First, store Vertices
    create_vertices(q);

    // Second, store Faces. For each one,
    // create all the half edges in order and link to the other one.
    // Sadly, vertices are not ccw nor cw ordered in QHull.
    orgQhull::QhullFacetList facets = q.facetList();
    for (orgQhull::QhullFacet &facet : q.facetList())
    {
        create_face_and_halfedges(facet);
    }

    pair_halfedges();
    generate_voronoi_planes();
    generated = true;
}

bool ConvexPolytope::euler_number_correct()
{
    if (!generated) {
        throw std::logic_error("ConvexPolytope::euler_number_correct "
                               "HalfEdge Structure not generated");
    }

    // V – E + F = 2
    // two HalfEdges is one edge (E)

    if ( halfedges.size() % 2 != 0 )
    {
        return false;
    }

    return (vertices.size() - (halfedges.size()/2) + faces.size() == 2);
}

void ConvexPolytope::create_vertices(const orgQhull::Qhull& q)
{
    orgQhull::QhullVertexList qhullVertexList = q.vertexList();
    vertices.resize(qhullVertexList.size());
    for (auto& qhull_vertex : qhullVertexList)
    {
        const auto& coordinates = qhull_vertex.point().coordinates();
        vertices.at(qhull_vertex.id()-1) = Vertex(Point(coordinates[0],
                                                       coordinates[1],
                                                       coordinates[2]));
    }
}

void ConvexPolytope::create_face_and_halfedges(orgQhull::QhullFacet& qhullfacet)
{
    orgQhull::QhullHyperplane qhull_hyperplane = qhullfacet.hyperplane();

    const auto& coordscenter = qhullfacet.getCenter().coordinates();
    Vector center(coordscenter[0], coordscenter[1], coordscenter[2]);

    const auto& coordsplane = qhull_hyperplane.coordinates();
    Plane plane{Vector(coordsplane[0], coordsplane[1], coordsplane[2]),
                static_cast<Real>(qhull_hyperplane.offset())};

    orgQhull::QhullVertexSet qhull_facevertices = qhullfacet.vertices();

    orgQhull::QhullVertexSet::iterator itVertex = qhull_facevertices.begin();

    const auto& vertexcoords = (*itVertex).point().coordinates();
    Vector first_vertex(vertexcoords[0], vertexcoords[1], vertexcoords[2]);

    first_vertex = (first_vertex - center);
    first_vertex.normalize();

    std::vector<std::pair<Real, FeatureId> > ordered_vertices;
    ordered_vertices.push_back(std::make_pair(0.0, (*itVertex).id() -1)); //angle 0.0 radians
    itVertex++;
    for (;itVertex < qhull_facevertices.end(); itVertex++)
    {
        const auto& vertexcoords = (*itVertex).point().coordinates();
        Vector current_vertex(vertexcoords[0], vertexcoords[1], vertexcoords[2]);
        current_vertex = current_vertex - center;
        current_vertex.normalize();
        // this is dangerous, sometimes it is slightly smaller than -1.0 or greater than 1.0
        Real dotproduct = std::max(static_cast<Real>(-1.0),
                                   std::min(static_cast<Real>(1.0), first_vertex.dot(current_vertex)));
        Real angle = acos(dotproduct);
        Vector crossproduct_first_current = first_vertex.cross(current_vertex);
        if (!crossproduct_first_current.isZero() && crossproduct_first_current.dot(plane.normal()) < 0)
        {
            angle = -angle + 2 * PI;
        }
        ordered_vertices.push_back(std::make_pair(angle, (*itVertex).id() -1));
    }

    //sort the vertices
    std::sort( std::begin(ordered_vertices), std::end(ordered_vertices),
               []( const std::pair<Real, FeatureId> &left,
                   const std::pair<Real, FeatureId> &right)
               {
                   return  left.first < right.first;
               } );

    // Create the face.
    FeatureId face_id = faces.size();
    faces.push_back(Face(plane));
    Face& face = faces[face_id];

    // Create Edges and update Vertex if necessary

    // this is necessary to update the "pair" in HalfEdges
    // std::pair is from -> to in the HalfEdge.
    std::map< std::pair<FeatureId, FeatureId>, FeatureId > map_vertices_halfedges;

    FeatureId first_vertex_id = InvalidFeatureId;
    FeatureId first_halfedge_id = InvalidFeatureId;
    FeatureId previous_halfedge_id = InvalidFeatureId;
    FeatureId previous_vertex_id = InvalidFeatureId;
    for (int i = 0; i < ordered_vertices.size(); i++)
    {
        FeatureId headvertex_id = ordered_vertices[i].second;
        FeatureId halfedge_id = halfedges.size();
        halfedges.push_back(HalfEdge());
        HalfEdge& edge = halfedges[halfedge_id];
        edge.set_headvertex_id(headvertex_id);
        edge.set_face_id(face_id);

        if (i == 0) {
            first_halfedge_id = halfedge_id;
            first_vertex_id = headvertex_id;
            face.set_halfedge_id(halfedge_id);
        } else {
            halfedges[previous_halfedge_id].set_next_id(halfedge_id);
        }

        if (i == ordered_vertices.size() -1 ) { //last element points to first
            edge.set_next_id(first_halfedge_id);
        }

        previous_halfedge_id = halfedge_id;
        previous_vertex_id = headvertex_id;
    }

}

void ConvexPolytope::pair_halfedges()
{
    std::map< std::pair<FeatureId, FeatureId>, FeatureId > map_vertices_halfedges;
    for (auto &face: faces)
    {
        FeatureId firstedge_id = face.get_halfedge_id();
        FeatureId previousedge_id = firstedge_id;
        FeatureId currentedge_id = get_halfedge(previousedge_id).get_next_id();
        do {
            const HalfEdge& heorig = get_halfedge(previousedge_id);
            const HalfEdge& hedest = get_halfedge(currentedge_id);

            map_vertices_halfedges[std::make_pair(heorig.get_headvertex_id(),
                                                  hedest.get_headvertex_id())] = currentedge_id;

            previousedge_id = currentedge_id;
            currentedge_id = get_halfedge(currentedge_id).get_next_id();
        } while (previousedge_id != firstedge_id);
    }

    for (auto it = map_vertices_halfedges.begin();
         it!=map_vertices_halfedges.end();
         ++it)
    {
        FeatureId tailvertex_id = it->first.first;
        FeatureId headvertex_id = it->first.second;
        FeatureId halfedge_id = it->second;

        // pair the other side.
        FeatureId pairhalfedge_id = map_vertices_halfedges[std::make_pair(headvertex_id, tailvertex_id)];
        halfedges[halfedge_id].set_pair_id(pairhalfedge_id);

        //update vertex HalfEdge if necessary
        if (!vertices[tailvertex_id].has_halfedge()) {
            vertices[tailvertex_id].set_halfedge_id(halfedge_id);
        }
    }


}

void ConvexPolytope::generate_voronoi_planes()
{
    // generate in all the halfedges the voronoi planes for the face (face-edge)
    // and for the head vertex (vertex-edge). The normal of the
    // planes point to the inside of the voronoi region of the edge(halfedge)!!

    // . is dot_product. x is cross_product.

    // the voronoi normal of the (face-edge) is :  edge_direction x face_plane_normal
    // with that normal and the edge vertex we have the plane. n.(X-P) = 0, so
    // the distance is n.P. (n normal, X coordinates and P point).


    // the voronoi normal of the (vertex-edge), the normal is the vector
    // from the vertices of the edge (head - tail).
    // A point in the plane is the current vertex.

    for (auto it = halfedges.begin(); it != halfedges.end(); ++it)
    {
        FeatureId heid = std::distance(halfedges.begin(), it);
        HalfEdge& he = get_halfedge(heid);
        const HalfEdge& pairedge = get_halfedge(he.get_pair_id());

        const Point& headvertex_point = get_vertex(he.get_headvertex_id()).get_position();
        const Point& tailvertex_point = get_vertex(pairedge.get_headvertex_id()).get_position();
        Vector inverse_edge_direction = tailvertex_point - headvertex_point;

        const Plane& face_baseplane = get_face(he.get_face_id()).get_plane();

        //the plane normal must be normalized for Eigen library to work.
        he.set_faceedge_plane(Plane{face_baseplane.normal().cross(inverse_edge_direction).normalized(),
                                  headvertex_point});


        he.set_headvertexedge_plane(Plane{inverse_edge_direction.normalized(),
                                        headvertex_point});
    }
}

std::array<VoronoiPlane, 2> ConvexPolytope::get_vpve_edge(Feature hid) const
{
    assert (hid.type == FeatureType::edge);
    const HalfEdge& he = get_halfedge(hid.feature);
    const HalfEdge& pairhe = get_halfedge(he.get_pair_id());
    return std::array<VoronoiPlane, 2>{
        VoronoiPlane{he.get_headvertexedge_plane(),
                         Feature{he.get_headvertex_id(), FeatureType::vertex}},
            VoronoiPlane{pairhe.get_headvertexedge_plane(),
                             Feature{pairhe.get_headvertex_id(), FeatureType::vertex}}
    };
}

std::array<VoronoiPlane, 2> ConvexPolytope::get_vpfe_edge(Feature hid) const
{
    assert (hid.type == FeatureType::edge);
    std::vector<VoronoiPlane> resp;
    const HalfEdge& he = get_halfedge(hid.feature);
    const HalfEdge& pairhe = get_halfedge(he.get_pair_id());
    return std::array<VoronoiPlane, 2>{
        VoronoiPlane{he.get_faceedge_plane(), Feature{he.get_face_id(), FeatureType::face}},
            VoronoiPlane{pairhe.get_faceedge_plane(), Feature{pairhe.get_face_id(), FeatureType::face}}
    };
}

std::vector<Point> ConvexPolytope::get_face_points(Feature fid) const
{
    assert (fid.type == FeatureType::face);
    std::vector<Point> resp;
    FeatureId firsthalfedge_id = get_face(fid.feature).get_halfedge_id();
    FeatureId currenthalfedge_id = firsthalfedge_id;
    do
    {
        const HalfEdge &he = get_halfedge(currenthalfedge_id);
        resp.emplace_back(get_vertex(he.get_headvertex_id()).get_position());
        currenthalfedge_id = he.get_next_id();
    } while (firsthalfedge_id != currenthalfedge_id);
    return resp;
}


std::vector<Feature> ConvexPolytope::get_face_vertices(Feature fid) const
{
    assert (fid.type == FeatureType::face);
    std::vector<Feature> resp;

    FeatureId firsthalfedge_id = get_face(fid.feature).get_halfedge_id();
    FeatureId currenthalfedge_id = firsthalfedge_id;
    do
    {
        const HalfEdge &he = get_halfedge(currenthalfedge_id);
        resp.emplace_back(he.get_headvertex_id(), FeatureType::vertex);
        currenthalfedge_id = he.get_next_id();
    } while (firsthalfedge_id != currenthalfedge_id);
    return resp;
}


std::vector<EdgePoints> ConvexPolytope::get_face_edgepoints(Feature fid) const
{
    assert (fid.type == FeatureType::face);
    std::vector<EdgePoints> resp;

    FeatureId firsthalfedge_id = get_face(fid.feature).get_halfedge_id();
    FeatureId currenthalfedge_id = firsthalfedge_id;
    do
    {
        const HalfEdge &he = get_halfedge(currenthalfedge_id);
        Point tail_point = get_vertex(he.get_headvertex_id()).get_position();
        currenthalfedge_id = he.get_next_id();
        Point head_point = get_vertex(get_halfedge(currenthalfedge_id).get_headvertex_id()).get_position();
        resp.emplace_back(tail_point, head_point);
    } while (firsthalfedge_id != currenthalfedge_id);
    return resp;
}


void ConvexPolytope::get_local_aabb(Vector& min_point, Vector &max_point) const
{
    // not very efficient. Just local Aabb, not principal moments or
    // something more elaborated
    const Real lowest_real = std::numeric_limits<Real>::lowest();
    const Real highest_real = std::numeric_limits<Real>::max();
    min_point = Vector(highest_real, highest_real, highest_real);
    max_point = Vector(lowest_real, lowest_real, lowest_real);

    for (const Vertex& v: vertices)
    {
        const Point& p = v.get_position();
        min_point[0] = std::min(p[0], min_point[0]);
        min_point[1] = std::min(p[1], min_point[1]);
        min_point[2] = std::min(p[2], min_point[2]);
        max_point[0] = std::max(p[0], max_point[0]);
        max_point[1] = std::max(p[1], max_point[1]);
        max_point[2] = std::max(p[2], max_point[2]);
    }
}

void ConvexPolytope::write(std::ostream& output)
{
    // saves the half edge structure, not the voronoi planes.
    output.precision(20);

    for (auto it = vertices.begin(); it != vertices.end(); ++it)
    {
        FeatureId vid = std::distance(vertices.begin(), it);
        output << "V" << " " << vid << " "
               << it->get_position()[0] << " "
               << it->get_position()[1] << " "
               << it->get_position()[2] << " "
               << it->get_halfedge_id() << "\n";
    }

    for (auto it = halfedges.begin(); it != halfedges.end(); ++it)
    {
        FeatureId heid = std::distance(halfedges.begin(), it);
        output << "HE" << " "  << heid << " "
               << it->get_pair_id() << " "
               << it->get_next_id() << " "
               << it->get_headvertex_id() << " "
               << it->get_face_id()
               << "\n";
    }

    for (auto it = faces.begin(); it != faces.end(); ++it)
    {
        FeatureId fid = std::distance(faces.begin(), it);
        output << "F" << " "  << fid << " "
               << it->get_halfedge_id() << " "
               << it->get_plane().normal()[0] << " "
               << it->get_plane().normal()[1] << " "
               << it->get_plane().normal()[2] << " "
               << it->get_plane().offset()
               << "\n";
    }

}

// TODO CREATE MOVE CONSTRUCTOR AND MOVE ASSIGNMENT
ConvexPolytope ConvexPolytope::create(std::istream& ifs)
{
    ConvexPolytope a;

    std::string type;
    while (ifs >> type)
    {
        FeatureId fid;
        ifs >> fid;
        if (type == "V") {
            Real x,y,z;
            ifs >> x;
            ifs >> y;
            ifs >> z;
            Vertex v{Point{x,y,z}};
            FeatureId he;
            ifs >> he;
            v.set_halfedge_id(he);
            a.vertices.push_back(v);
        } else if (type == "HE") {
            FeatureId pair;
            ifs >> pair;
            FeatureId next;
            ifs >> next;
            FeatureId headVertex;
            ifs >> headVertex;
            FeatureId face;
            ifs >> face;
            HalfEdge he = HalfEdge();
            he.set_pair_id(pair);
            he.set_next_id(next);
            he.set_headvertex_id(headVertex);
            he.set_face_id(face);
            a.halfedges.push_back(he);
        } else if (type == "F") {
            FeatureId halfEdgeId;
            ifs >> halfEdgeId;
            Real x,y,z,offset;
            ifs >> x;
            ifs >> y;
            ifs >> z;
            ifs >> offset;
            Face face{Plane{Vector{x,y,z}, offset}};
            face.set_halfedge_id(halfEdgeId);
            a.faces.push_back(face);
        }
    }

    a.generate_voronoi_planes();
    a.generated = true;
    assert(a.euler_number_correct());
    return a;
}

std::ostream& OB::operator<< (std::ostream &out, const WorldConvexPolytope &wcp)
{
    out << "WorldConvexPolytope(" << wcp.get_name() << std::endl;
    out << wcp.convex_polytope() << std::endl;
    out << "translation:" << wcp.get_pose().translation().transpose() << std::endl;
    out << "rotation:\n" << wcp.get_pose().rotation() << std::endl;
    out << ")";
    return out;
}

std::ostream& OB::operator<< (std::ostream &out, const ConvexPolytope &cp)
{
    out << "ConvexPolytope(" << std::endl;
    for (FeatureId i = 0; i< cp.vertices.size(); i++)
    {
        out << "Vertex" << i << ":" << cp.vertices[i] << std::endl;
        // Voronoi planes
        auto planes = cp.get_vpve_vertex(Feature{i, FeatureType::vertex});
        for (const auto &pair : planes)
        {
            out << "   VP: normal: " << pair.plane.normal().transpose()
                << " offset: " << pair.plane.offset()
                << " Feature: " << pair.feature << std::endl;
        }
    }

    for (FeatureId i = 0; i< cp.faces.size(); i++)
    {
        out << "Face" << i << ":" << cp.faces[i] << std::endl;
        out << " Face Center" << cp.faces[i].get_center(cp).transpose() << std::endl;
    }

    for (FeatureId i = 0; i< cp.halfedges.size(); i++)
    {
        out << "HalfEdge " << i << ":" << cp.halfedges[i] << std::endl;
    }
    out << ")";

    return out;
}

std::ostream& OB::operator<< (std::ostream &out, const Face &face)
{
    out << " Plane " << face.plane.normal().transpose()
        << " Offset " << face.plane.offset() << " HalfEdgeId " << face.halfedge_id;
    return out;
}


std::ostream& OB::operator<< (std::ostream &out, const Vertex &vertex)
{
    out << " Position " << vertex.position.transpose() << " halfedge_id " << vertex.halfedge;
    return out;
}


std::ostream& OB::operator<< (std::ostream &out, const HalfEdge &halfEdge)
{
    out << " nextHalfEdgeId " << halfEdge.get_next_id() << " pairhalfedge_id " << halfEdge.pair
        << " headvertex_id " << halfEdge.get_headvertex_id() << " face_id " << halfEdge.face_id << std::endl;
    out << "    - headVertexEdgePlane normal:"
        << halfEdge.get_headvertexedge_plane().normal().transpose ()
        << " offset: " << halfEdge.get_headvertexedge_plane().offset() << std::endl;
    out << "    - faceEdgePlane normal:"
        << halfEdge.get_faceedge_plane().normal().transpose ()
        << " offset: " << halfEdge.get_faceedge_plane().offset();
    return out;
}

std::ostream& OB::operator<< (std::ostream &out, const Feature &f)
{
    out << "{featureType: " << f.type << " id: " << f.feature << "}";
    return out;
}


std::ostream& OB::operator<< (std::ostream &out, const FeatureType& ft)
{
    switch(ft)
    {
    case FeatureType::vertex: out << "vertex";    break;
    case FeatureType::edge: out << "edge"; break;
    case FeatureType::face: out << "face";  break;
    case FeatureType::empty: out << "empty";  break;
    default: out << "?????";  break; //out.setstate(std::ios_base::failbit);
    }
    return out;
}

std::ostream& OB::operator<< (std::ostream &out, const Plane& p)
{
    out  << " Plane normal: " << p.normal().transpose() << " offset: " << p.offset();
    return out;
}
