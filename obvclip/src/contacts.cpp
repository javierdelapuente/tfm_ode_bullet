#include "contacts.hpp"

using namespace OB;

#undef min
#undef max


ContactDataVClip get_vertex_vertex_contact(WorldConvexPolytope& worldconvexpolytope1,
                                           WorldConvexPolytope& worldconvexpolytope2,
                                           VClipResult &lastresult)
{
    assert(lastresult.state == VClipState::vertex_vertex);
    const ConvexPolytope &convexpolytope1 = worldconvexpolytope1.convex_polytope();
    const ConvexPolytope &convexpolytope2 = worldconvexpolytope2.convex_polytope();

    ContactDataVClip contact;
    Point p1 = worldconvexpolytope1.get_pose()
        * worldconvexpolytope1.convex_polytope().get_vertex_point(lastresult.witness.feature1);
    Point p2 = worldconvexpolytope2.get_pose()
        * worldconvexpolytope2.convex_polytope().get_vertex_point(lastresult.witness.feature2);
    contact.point = (p1 + p2) * 0.5;
    contact.point_in_body_2 = p2;
    // it must point from body 2 to body 1
    contact.normal = (p1 - p2).normalized();
    contact.depth = -lastresult.distance;
    return contact;
}

ContactDataVClip get_vertex_edge_contact(WorldConvexPolytope& worldconvexpolytope1,
                                         WorldConvexPolytope& worldconvexpolytope2,
                                         VClipResult &lastresult)
{
    assert(lastresult.state == VClipState::vertex_edge);
    ContactDataVClip contact;
    bool swap = lastresult.witness.feature1.type > lastresult.witness.feature2.type;
    Point p;
    EdgePoints e;
    if (!swap) {
        p = worldconvexpolytope1.get_pose()
            * worldconvexpolytope1.convex_polytope()
            .get_vertex_point(lastresult.witness.feature1);
        e = worldconvexpolytope2.convex_polytope()
            .get_edgepoints(lastresult.witness.feature2);
        e.head = worldconvexpolytope2.get_pose() * e.head;
        e.tail = worldconvexpolytope2.get_pose() * e.tail;
    } else {
        p = worldconvexpolytope2.get_pose()
            * worldconvexpolytope2.convex_polytope().get_vertex_point(lastresult.witness.feature2);
        e = worldconvexpolytope1.convex_polytope()
            .get_edgepoints(lastresult.witness.feature1);
        e.head = worldconvexpolytope1.get_pose() * e.head;
        e.tail = worldconvexpolytope1.get_pose() * e.tail;
    }

    Point closest_in_line = closest_point_to_line(p, e);
    if (swap)
    {
        contact.point_in_body_2 = p;
    }
    else
    {
        contact.point_in_body_2 = closest_in_line;
    }
    contact.point = (p + closest_in_line) * 0.5;
    // it must point from body 2 to body 1
    contact.normal = (p - closest_in_line).normalized();
    if (swap) {
        contact.normal *= -1;
    }
    contact.depth = -lastresult.distance;
    return contact;
}


ContactDataVClip get_edge_edge_contact(WorldConvexPolytope& worldconvexpolytope1,
                                       WorldConvexPolytope& worldconvexpolytope2,
                                       VClipResult &lastresult)
{
    assert(lastresult.state == VClipState::edge_edge);
    ContactDataVClip contact;
    EdgePoints e1 = worldconvexpolytope1.convex_polytope()
        .get_edgepoints(lastresult.witness.feature1);
    e1.head = worldconvexpolytope1.get_pose() * e1.head;
    e1.tail = worldconvexpolytope1.get_pose() * e1.tail;
    EdgePoints e2 = worldconvexpolytope2.convex_polytope()
        .get_edgepoints(lastresult.witness.feature2);
    e2.head = worldconvexpolytope2.get_pose() * e2.head;
    e2.tail = worldconvexpolytope2.get_pose() * e2.tail;
    std::pair<Point, Point> closest = closests_line_to_line(e1, e2);

    contact.point = (closest.second + closest.first) * 0.5;
    contact.point_in_body_2 = closest.second;
    // TODO wrong if they are intersecting
    // it must point from body 2 to body 1
    contact.normal = (closest.first - closest.second).normalized();
    contact.depth = -lastresult.distance;
    return contact;
}

ContactDataVClip get_vertex_face_contact(WorldConvexPolytope& worldconvexpolytope1,
                                         WorldConvexPolytope& worldconvexpolytope2,
                                         VClipResult &lastresult)
{
    assert(lastresult.state == VClipState::vertex_face);
    ContactDataVClip contact;
    bool swap = lastresult.witness.feature1.type > lastresult.witness.feature2.type;

    Point p;
    Plane plane;
    if (!swap) {
        p = worldconvexpolytope1.get_pose()
            * worldconvexpolytope1.convex_polytope().get_vertex_point(lastresult.witness.feature1);
        plane = worldconvexpolytope2.convex_polytope()
            .get_face_plane(lastresult.witness.feature2);
        plane.transform(worldconvexpolytope2.get_pose(), Isometry);
    } else {
        p = worldconvexpolytope2.get_pose()
            * worldconvexpolytope2.convex_polytope()
            .get_vertex_point(lastresult.witness.feature2);
        plane = worldconvexpolytope1.convex_polytope()
            .get_face_plane(lastresult.witness.feature1);
        plane.transform(worldconvexpolytope1.get_pose(), Isometry);
    }

    Point projection_point = plane.projection(p);
    if (swap) {
        contact.point_in_body_2 = p;
    } else {
        contact.point_in_body_2 = projection_point;
    }
    contact.point = (p + projection_point)*0.5;
    contact.depth = -lastresult.distance;
    contact.normal = plane.normal();
    if (swap) {
        contact.normal *= -1;
    }
    return contact;
}


ContactDataVClip get_edge_face_contact(WorldConvexPolytope& worldconvexpolytope1,
                                       WorldConvexPolytope& worldconvexpolytope2,
                                       VClipResult &lastresult)
{
    assert(lastresult.state == VClipState::edge_face);
    ContactDataVClip contact;
    bool swap = lastresult.witness.feature1.type > lastresult.witness.feature2.type;
    EdgePoints e;
    Plane plane;
    Transform toworld_fromedge;
    Transform toworld_fromface;
    if (!swap) {
        toworld_fromedge = worldconvexpolytope1.get_pose();
        toworld_fromface = worldconvexpolytope2.get_pose();
        e = worldconvexpolytope1.convex_polytope()
            .get_edgepoints(lastresult.witness.feature1);
        plane = worldconvexpolytope2.convex_polytope()
            .get_face_plane(lastresult.witness.feature2);
    } else
    {
        toworld_fromedge = worldconvexpolytope2.get_pose();
        toworld_fromface = worldconvexpolytope1.get_pose();
        e = worldconvexpolytope2.convex_polytope()
            .get_edgepoints(lastresult.witness.feature2);
        plane = worldconvexpolytope1.convex_polytope()
            .get_face_plane(lastresult.witness.feature1);
    }
    e.head = toworld_fromedge * e.head;
    e.tail = toworld_fromedge * e.tail;
    plane.transform(toworld_fromface, Isometry);


    std::pair<bool, Vector> intersect = intersect_line_plane(e, plane);
    if (!intersect.first) {
        // this should not happen, as this is a state of intersection
        throw std::logic_error("VClip::getContactInfo edge parallel to face not intersecting.");
    }

    // option 1. use the intersect point edge face. be careful we use a
    // depth value lower than zero, and that is not really true.
    {
        contact.point = intersect.second;
        contact.point_in_body_2 = intersect.second;
    }
    // option 2. use the point returned from vclip.
    // be careful, it could even not be in the CSO of the objects.
    // use only if penetrating!! or the point is not defined!!
    {
        Point max_penetracion_edge = toworld_fromedge * lastresult.edge_face_max_penetration_point;
        Point projected_max_penetracion = plane.projection(max_penetracion_edge);
        contact.point = (max_penetracion_edge + projected_max_penetracion) * 0.5;
        if (swap) {
            contact.point_in_body_2 = max_penetracion_edge;
        } else {
            contact.point_in_body_2 = projected_max_penetracion;
        }
    }


    contact.normal = plane.normal();
    if (swap) {
        contact.normal *= -1;
    }
    contact.depth = -lastresult.distance;
    // SECOND IDEA. POINT THE INTERSECTION OF PLANE AND FACE WITH DISTANCE 0.0
    // std::pair<bool, Vector> intersect = intersect_line_plane(e, plane);
    // if (!intersect.first)
    //     throw std::logic_error("VClip::getContactInfo edge parallel to face");
    // contact.point = intersect.second;
    // contact.normal = plane.normal();
    // if (swap) { contact.normal *= -1; }
    // contact.depth = 0.0;
    return contact;
}




ContactDataVClip OB::get_simple_contact(WorldConvexPolytope& worldconvexpolytope1,
                                        WorldConvexPolytope& worldconvexpolytope2,
                                        VClipResult &lastresult)
{
    ContactDataVClip contact;
    const ConvexPolytope &convexpolytope1 = worldconvexpolytope1.convex_polytope();
    const ConvexPolytope &convexpolytope2 = worldconvexpolytope2.convex_polytope();

    switch(lastresult.state)
    {
    case VClipState::vertex_vertex:
        contact = get_vertex_vertex_contact(worldconvexpolytope1,
                                            worldconvexpolytope2,
                                            lastresult);
        break;
    case VClipState::vertex_edge:
        contact = get_vertex_edge_contact(worldconvexpolytope1,
                                          worldconvexpolytope2,
                                          lastresult);
        break;
    case VClipState::edge_edge:
        contact = get_edge_edge_contact(worldconvexpolytope1,
                                        worldconvexpolytope2,
                                        lastresult);
        break;
    case VClipState::vertex_face:
        contact = get_vertex_face_contact(worldconvexpolytope1,
                                          worldconvexpolytope2,
                                          lastresult);
        break;
    case VClipState::edge_face:
        contact = get_edge_face_contact(worldconvexpolytope1,
                                        worldconvexpolytope2,
                                        lastresult);
        break;
    default:
        L_(lerror) << "VClip::getContactInfo State " <<  lastresult.state;
        throw std::logic_error("SHOULD NOT BE IN THIS STATE WITH DONE, NON PENETRATION.");
    }
    return contact;
}

//normal and point in world coordinates.
void get_all_features_in_envelope(Feature current_feature,
                                  std::vector<Feature>& features,
                                  Plane contact_plane, Real envelope,
                                  WorldConvexPolytope& worldconvexpolytope)
{
    L_(ldebug) << " get_all_features_in_envelope " << worldconvexpolytope.get_name()
               << " " << current_feature;
    if (std::count(features.begin(), features.end(), current_feature))
    {
        L_(ldebug) << "    already added";
        return;
    }

    const ConvexPolytope& polytope = worldconvexpolytope.convex_polytope();
    if (current_feature.isVertex())
    {
        L_(ldebug) << " in Vertex " ;
        Point vertexpoint = worldconvexpolytope.get_pose()
            * polytope.get_vertex_point(current_feature);
        if (contact_plane.absDistance(vertexpoint) <= envelope)
        {
            L_(ldebug) << "   Vertex in envelope." ;
            features.push_back(current_feature);
            for (auto vp : polytope.get_vpve_vertex(current_feature))
            {
                Feature edge = vp.feature;
                get_all_features_in_envelope(edge,
                                             features,
                                             contact_plane, envelope,
                                             worldconvexpolytope);

                Feature pair = Feature{polytope.get_halfedge(edge.feature).get_pair_id(),
                                       FeatureType::edge};
                get_all_features_in_envelope(pair,
                                             features,
                                             contact_plane, envelope,
                                             worldconvexpolytope);
            }
        }

    } else if (current_feature.isEdge()) {
        L_(ldebug) << " in Edge " ;
        EdgePoints ep = worldconvexpolytope.convex_polytope().get_edgepoints(current_feature);
        ep.head = worldconvexpolytope.get_pose() * ep.head;
        ep.tail = worldconvexpolytope.get_pose() * ep.tail;
        Real distance_head = contact_plane.signedDistance(ep.head);
        Real distance_tail = contact_plane.signedDistance(ep.tail);
        if (abs(distance_head) < envelope || abs(distance_tail) < envelope
            || distance_head*distance_tail < 0)
        {
            L_(ldebug) << "  edge in envelope " ;
            features.push_back(current_feature);

            // put all vertices not in features that are inside the envelope.
            // put both faces not in features.
            for (auto vp: polytope.get_vpve_edge(current_feature))
            {
                Feature vertex = vp.feature;
                get_all_features_in_envelope(vertex, features,
                                             contact_plane, envelope,
                                             worldconvexpolytope);
            }

            for (auto vp : polytope.get_vpfe_edge(current_feature))
            {
                Feature face = vp.feature;

                get_all_features_in_envelope(face, features,
                                             contact_plane, envelope,
                                             worldconvexpolytope);
            }
        }
    } else if (current_feature.isFace()) {
        L_(ldebug) << " in Face " ;
        features.push_back(current_feature);
        for (auto vp : polytope.get_vpfe_face(current_feature))
        {
            Feature edge = vp.feature;
            get_all_features_in_envelope(edge, features,
                                         contact_plane, envelope,
                                         worldconvexpolytope);
        }
    }
}

// delete duplicated points O(n^2)
void delete_duplicated_edges(std::vector<Feature>& features, const ConvexPolytope &polytope)
{
    for (auto it = features.begin(); it != features.end(); ++it)
    {
        if (it->isEdge())
        {
            for (auto it2 = it; it2 != features.end(); ++it2)
            {
                if (polytope.get_halfedge(it->feature).get_pair_id() == it2->feature)
                {
                    it2 = features.erase(it2);
                    break;
                }
            }

        }
    }
}

void add_point_face_projections(std::vector<Point> &projections,
                                Plane& contact_plane, Real envelope,
                                std::vector<Feature> &features1,
                                std::vector<Feature> &features2,
                                WorldConvexPolytope& worldconvexpolytope1,
                                WorldConvexPolytope& worldconvexpolytope2)
{
    for (Feature vertex: features1)
    {
        if (!vertex.isVertex()) continue;
        for (Feature face: features2)
        {
            if (!face.isFace()) continue;
            // check projection of point is in face projection.
            // face projection is all edges projections.

            L_(ldebug) << " checking vertex " << vertex << " against " << face;

            Point vertexpoint = worldconvexpolytope1.get_pose()
                * worldconvexpolytope1.convex_polytope().get_vertex_point(vertex);
            Point vertexpoint_projected = contact_plane.projection(vertexpoint);

            std::vector<EdgePoints> edges_in_face = worldconvexpolytope2.convex_polytope()
                .get_face_edgepoints(face);
            bool face_valid = true;
            Plane faceplane = worldconvexpolytope2.convex_polytope().get_face_plane(face);
            faceplane.transform(worldconvexpolytope2.get_pose(), Isometry);
            for (EdgePoints ep : edges_in_face)
            {
                ep.head = worldconvexpolytope2.get_pose() * ep.head;
                ep.tail = worldconvexpolytope2.get_pose() * ep.tail;
                // projected over contact_plane
                ep.head = contact_plane.projection(ep.head);
                ep.tail = contact_plane.projection(ep.tail);
                if ((Vector(ep.tail - vertexpoint_projected)
                     .cross(ep.vector())).dot(contact_plane.normal()) < 0.0)
                {
                    L_(ldebug) << "  point not in face projection";
                    face_valid = false;
                    break;
                }
            }
            if (face_valid)
            {
                L_(ldebug) << "  point in face projection";

                // check contact in face distance from contact plane.

                //TODO FILTER FOR DISTANCE BETWEEN ORIGINAL VERTEX AND FACE CONTACT POINT < 2*ENVELOPE??
                projections.push_back(vertexpoint_projected);
            }
        }
    }
}


void add_edge_edge_projections(std::vector<Point> &projections,
                               Plane& contact_plane, Real envelope,
                               std::vector<Feature> &features1,
                               std::vector<Feature> &features2,
                               WorldConvexPolytope& worldconvexpolytope1,
                               WorldConvexPolytope& worldconvexpolytope2)
{
    for (Feature edge1: features1)
    {
        if (!edge1.isEdge()) continue;
        for (Feature edge2: features2)
        {
            if (!edge2.isEdge()) continue;
            EdgePoints ep1 = worldconvexpolytope1.convex_polytope().get_edgepoints(edge1);
            ep1.head = worldconvexpolytope1.get_pose() * ep1.head;
            ep1.tail = worldconvexpolytope1.get_pose() * ep1.tail;
            ep1.head = contact_plane.projection(ep1.head);
            ep1.tail = contact_plane.projection(ep1.tail);

            EdgePoints ep2 = worldconvexpolytope2.convex_polytope().get_edgepoints(edge2);
            ep2.head = worldconvexpolytope2.get_pose() * ep2.head;
            ep2.tail = worldconvexpolytope2.get_pose() * ep2.tail;
            ep2.head = contact_plane.projection(ep2.head);
            ep2.tail = contact_plane.projection(ep2.tail);
            // check if projection of edge1 intersects  projection of edge2 and return that point

            Vector edgevector1 = ep1.vector();
            Vector edgevector2 = ep2.vector();
            Real edgelength1 = (edgevector1).norm();
            Real edgelength2 = (edgevector2).norm();
            Vector unitedgevector1 = edgevector1 * (1.0/edgelength1);
            Vector unitedgevector2 = edgevector2 * (1.0/edgelength2);
            Real unitedge_directions_dotproduct = unitedgevector1.dot(unitedgevector2);
            if (is_zero(abs(unitedge_directions_dotproduct) - 1.0)) {
                continue; // parallel lines
            }
            Vector vector_tail1_tail2 = ep2.tail - ep1.tail;
            Real t = (vector_tail1_tail2.dot(unitedgevector1) -
                      (vector_tail1_tail2.dot(unitedgevector2))* (unitedge_directions_dotproduct))
                / ( 1 - unitedge_directions_dotproduct*unitedge_directions_dotproduct);
            if (t < 0 || t > edgelength1)
            {
                continue;
            }
            Real s = - (vector_tail1_tail2.dot(unitedgevector2) -
                        (vector_tail1_tail2.dot(unitedgevector1))* (unitedge_directions_dotproduct))
                / ( 1 - unitedge_directions_dotproduct*unitedge_directions_dotproduct);
            if (s < 0 || s > edgelength2)
            {
                continue;
            }
            Real t_real = t / (1.0 * edgelength1);
            Real s_real = s / (1.0 * edgelength2);

            // ok point found. check if original positions in edges close to contact_plane

            EdgePoints origep1 = worldconvexpolytope1.convex_polytope().get_edgepoints(edge1);
            origep1.head = worldconvexpolytope1.get_pose() * origep1.head;
            origep1.tail = worldconvexpolytope1.get_pose() * origep1.tail;

            EdgePoints origep2 = worldconvexpolytope2.convex_polytope().get_edgepoints(edge2);
            origep2.head = worldconvexpolytope2.get_pose() * origep2.head;
            origep2.tail = worldconvexpolytope2.get_pose() * origep2.tail;

            //t_real and s_real positions are the same for the original and the projection of the edge.
            // unless singular. do not return if both edges are far away from the projection point.

            //TODO BETTER CHANGE FOR DISTANCE BETWEEN EDGES < 2*ENVELOPE??
            if ((absdist_point_plane(origep1.tail + t_real * origep1.vector(), contact_plane) < envelope)
                || (absdist_point_plane(origep2.tail + s_real * origep2.vector(), contact_plane) < envelope))
            {
                projections.push_back(ep1.tail + t * unitedgevector1);
            }
            else
            {
                L_(ldebug) << " discard edge edge contact for dinstance to contact plane";
            }
        }
    }
}


// ideas from Erbelen 14.2 A Geometrical Algorithm.
std::vector<ContactDataVClip> OB::get_contacts_envelope(WorldConvexPolytope& worldconvexpolytope1,
                                                        WorldConvexPolytope& worldconvexpolytope2,
                                                        VClipResult &lastresult,
                                                        Real envelope)
{
    L_(ldebug) << " OB::get_contacts_envelope ";
    std::vector<ContactDataVClip> resp;

    L_(ldebug) << " before get simple contact ";
    L_(ldebug) << " wcp1  " << worldconvexpolytope1.get_name();
    L_(ldebug) << " wcp2  " << worldconvexpolytope2.get_name();
    L_(ldebug) << " f1 " << lastresult.witness.feature1;
    L_(ldebug) << " f2 " << lastresult.witness.feature2;
    L_(ldebug) << " state " << lastresult.state;
    L_(ldebug) << " distance " << lastresult.distance;

    ContactDataVClip base_contact = get_simple_contact(worldconvexpolytope1,
                                                       worldconvexpolytope2,
                                                       lastresult);
    resp.push_back(base_contact);
    L_(ldebug) << " got simple contact point " << base_contact.point
               << " normal " << base_contact.normal;

    Plane contact_plane{base_contact.normal, base_contact.point};
    std::vector<Feature> features_1;
    get_all_features_in_envelope(lastresult.witness.feature1, features_1,
                                 contact_plane, envelope,
                                 worldconvexpolytope1);
    L_(ldebug) << "Features f1 " << features_1.size();
    delete_duplicated_edges(features_1, worldconvexpolytope1.convex_polytope());
    L_(ldebug) << "Features f1 " << features_1.size();
    for (auto f1: features_1)
    {
        L_(ldebug) <<  " " << f1;
    }

    std::vector<Feature> features_2;
    get_all_features_in_envelope(lastresult.witness.feature2, features_2,
                                 contact_plane, envelope,
                                 worldconvexpolytope2);
    L_(ldebug) << "Features f2 " << features_2.size();
    delete_duplicated_edges(features_2, worldconvexpolytope2.convex_polytope());
    L_(ldebug) << "Features f2 " << features_2.size();
    for (auto f2: features_2)
    {
        L_(ldebug) <<  " " << f2;
    }

    // project vertex of f1 against faces of f2
    std::vector<Point> projections;
    L_(ldebug) << " starting " << projections.size();
    add_point_face_projections(projections,
                               contact_plane, envelope,
                               features_1,
                               features_2,
                               worldconvexpolytope1,
                               worldconvexpolytope2);
    L_(ldebug) << " found after 1 " << projections.size();
    add_edge_edge_projections(projections,
                              contact_plane, envelope,
                              features_1,
                              features_2,
                              worldconvexpolytope1,
                              worldconvexpolytope2);
    L_(ldebug) << " found after 2 " << projections.size();
    add_point_face_projections(projections,
                               contact_plane, envelope,
                               features_2,
                               features_1,
                               worldconvexpolytope2,
                               worldconvexpolytope1);
    L_(ldebug) << " found after 3 " << projections.size();


    for (const Point& projection : projections)
    {
        L_(ldebug) << " point before " << projection.transpose();
    }
    // delete duplicated points O(n^2)
    for (auto it = projections.begin(); it != projections.end(); ++it)
    {
        L_(ldebug) << " testing " << (*it).transpose() << " distance "  << std::distance(projections.begin(), it);
        for (auto it2 = it+1; it2 != projections.end(); ++it2)
        {
            L_(ldebug) << " vs " << (*it2).transpose() << " distance "  << std::distance(projections.begin(), it2);
            if ((*it - *it2).isZero()) {
                L_(ldebug) << "  deleted. new distance " << std::distance(projections.begin(), it2);;
                it2 = projections.erase(it2);
                --it2; // so current element can be deleted
            }
        }
    }
    L_(ldebug) << " found after deleting duplicates " << projections.size();

    for (const Point& projection : projections)
    {
        if ((projection - base_contact.point).isZero()) {
            continue; // already added
        }
        L_(ldebug) << " point " << projection.transpose();
        ContactDataVClip cd;
        cd.point = projection;
        cd.depth = 0.0;
        cd.normal = base_contact.normal;
        resp.push_back(cd);
    }
    L_(ldebug4) <<  "  number of contacts found " << resp.size();

    return resp;
}
