#include <iostream>
#include <algorithm>
#include <iomanip>
#include "polytope.hpp"
#include "vclip.hpp"
#include "clip_utils.hpp"

using namespace OB;

#undef min
#undef max

VClipResult VClip::operator()(VClipCache& cache)
{
    VClipCacheKey key{world_cp_1, world_cp_2};

    VClipResult result = solve(cache.has_witness(key) ?
                               cache.get_witness(key) : VClipWitness::FirstVertices());
    cache.set_witness(key, currentwitness);
    return lastresult;
}

VClipResult VClip::solve(VClipWitness witness)
{
    currentwitness = witness;
    // be careful, VClipWitness must equal to WorldConvexPolytope order.
    int iteration = 0;
    VClipStateResult stateresult;

    do {
        stateresult = check_features(currentwitness);
        if (++iteration > max_iterations) {
            L_(lerror) << std::fixed << std::setprecision (15);
            L_(lerror) << " worldconvexpolytope1 " << world_cp_1.get_name();
            L_(lerror) << " worldconvexpolytope2 " << world_cp_2.get_name();
            L_(lerror) << " worldconvexpolytope1 feature " << currentwitness.feature1;
            L_(lerror) << " worldconvexpolytope2 feature " << currentwitness.feature2;
            L_(lerror) << " worldconvexpolytope1 translation "
                       << world_cp_1.get_pose().translation();
            L_(lerror) << " worldconvexpolytope1 rotation "
                       << world_cp_1.get_pose().rotation();
            L_(lerror) << " worldconvexpolytope2 translation "
                       << world_cp_2.get_pose().translation();
            L_(lerror) << " worldconvexpolytope2 rotation "
                       << world_cp_2.get_pose().rotation();
            if (fail_on_max_iterations) {
                throw std::logic_error("Too many iterations in VClip. check for possible errors.");
            } else {
                lastresult.distance = absdist_between_features(world_cp_1, currentwitness.feature1,
                                                               world_cp_2, currentwitness.feature2);
                stateresult = VClipStateResult::finished;
            }
        }

    } while (stateresult == VClipStateResult::not_finished);
    lastresult.witness = currentwitness;
    lastresult.state = state;
    lastresult.result  = stateresult;
    return lastresult;
}

VClipResult VClip::iterate()
{
    Real previous_distance = absdist_between_features(world_cp_1, currentwitness.feature1,
                                                      world_cp_2, currentwitness.feature2);
    VClipWitness previous_witness = currentwitness;
    VClipStateResult stateresult = check_features(currentwitness);
    Real posterior_distance = absdist_between_features(world_cp_1, currentwitness.feature1,
                                                       world_cp_2, currentwitness.feature2);
    L_(linfo) << " Previous distance: " << previous_distance << " current: " << posterior_distance;
    if (stateresult == VClipStateResult::not_finished)
    {
        if (currentwitness.lowerDimension(previous_witness)) {
            if (!is_zero(posterior_distance - previous_distance)) {
                L_(lerror) << " ERROR DISTANCE MUST STAY THE SAME. DIFF " << posterior_distance - previous_distance;
                L_(lerror) << " iszero " << 4.6656e-12 << " > " << is_zero(4.6656e-12);
            }
            assert(is_zero(posterior_distance - previous_distance, 1.0e-6));//GRRR
        } else {
            if (posterior_distance >= previous_distance) {
                assert(posterior_distance < previous_distance);
            }

        }
    }
    lastresult.witness = currentwitness;
    bool swap = currentwitness.feature1.type > currentwitness.feature2.type;
    lastresult.state = update_state(currentwitness, swap);
    lastresult.result  = stateresult;
    return lastresult;
}

VClipStateResult VClip::check_features(VClipWitness& witness)
{
    VClipStateResult stateresult;

    bool swap = witness.feature1.type > witness.feature2.type;
    state = update_state(witness, swap);

    //careful with the order of features
    L_(ldebug3) << " f1: " << world_cp_1.get_name() << ": " << witness.feature1
                << " --- f2: " << world_cp_2.get_name() << ": " << witness.feature2;
    L_(ldebug3) << "State: " << state;
    switch(state)
    {
    case VClipState::vertex_vertex:
        stateresult = vertex_vertex_check(witness);
        break;
    case VClipState::vertex_edge:
        stateresult = vertex_edge_check(witness, swap);
        break;
    case VClipState::edge_edge:
        stateresult = edge_edge_check(witness);
        break;
    case VClipState::vertex_face:
        stateresult = vertex_face_check(witness, swap);
        break;
    case VClipState::edge_face:
        stateresult = edge_face_check(witness, swap);
        break;
    }
    return stateresult;
}


VClipState VClip::update_state(const VClipWitness& witness, bool swap)
{
    const Feature& f1swapped = swap ? witness.feature2 : witness.feature1;
    const Feature& f2swapped = swap ? witness.feature1 : witness.feature2;
    if (f1swapped.type == FeatureType::vertex &&  f2swapped.type == FeatureType::vertex)
    {
        return VClipState::vertex_vertex;
    }
    else if (f1swapped.type == FeatureType::vertex &&  f2swapped.type == FeatureType::edge)
    {
        return VClipState::vertex_edge;
    }
    else if (f1swapped.type == FeatureType::edge &&  f2swapped.type == FeatureType::edge)
    {
        return VClipState::edge_edge;
    }
    else if (f1swapped.type == FeatureType::vertex &&  f2swapped.type == FeatureType::face)
    {
        return VClipState::vertex_face;
    }
    else if (f1swapped.type == FeatureType::edge &&  f2swapped.type == FeatureType::face)
    {
        return VClipState::edge_face;
    }
    else
    {
        std::ostringstream buffer;
        buffer << " VClip::update_state. Wrong State. Feature1: " << f1swapped
               << " Feature2: " << f2swapped << " swap: " << swap;
        throw std::logic_error(buffer.str());
    }
}


VClipStateResult VClip::vertex_vertex_check(VClipWitness &witness)
{
    Point point1local = cp_1.get_vertex_point(witness.feature1);
    Point point1incoord2 = to2_from1 * point1local;

    //check v2 against voronoi planes of v1
    Point point2local = cp_2.get_vertex_point(witness.feature2);
    Point point2incoor1 = to1_from2 * point2local;

    auto itend = cp_1.end_vpve_vertex(witness.feature1);
    for (auto it = cp_1.begin_vpve_vertex(witness.feature1);
         it != itend; ++it)
    {
        VoronoiPlane vp = *it;
        if (vp.plane.signedDistance(point2incoor1) < 0)
        {
            witness.feature1 = vp.feature;
            return VClipStateResult::not_finished;
        }
    }

    auto itend2 = cp_2.end_vpve_vertex(witness.feature2);
    for (auto it = cp_2.begin_vpve_vertex(witness.feature2);
         it != itend2; ++it)
    {
        VoronoiPlane vp = *it;
        if (vp.plane.signedDistance(point1incoord2) < 0)
        {
            witness.feature2 = vp.feature;
            return VClipStateResult::not_finished;
        }
    }

    lastresult.distance = (point2incoor1 - point1local).norm();
    return VClipStateResult::finished;
}

VClipStateResult VClip::vertex_edge_check(VClipWitness &witness, bool swap)
{
    Feature& vertex = swap ? witness.feature2 : witness.feature1;
    Feature& edge = swap ? witness.feature1 : witness.feature2;
    const ConvexPolytope& vertexpolytope = swap ? cp_2 : cp_1;
    const ConvexPolytope& edgepolytope = swap ? cp_1 : cp_2;
    const Transform& tovertex_fromedge = swap ? to2_from1 : to1_from2;
    const Transform& toedge_fromvertex = swap ? to1_from2 : to2_from1;


    Point vertexpoint = vertexpolytope.get_vertex_point(vertex);

    //first vertex-edge planes of the edge
    std::array<VoronoiPlane, 2> voronois = edgepolytope.get_vpve_edge(edge);
    for (auto& v : voronois) {
        Plane& p2 = v.plane;
        p2.transform(tovertex_fromedge, Isometry);
        Real distance_plane_vertex = p2.signedDistance(vertexpoint);
        if (distance_plane_vertex < 0 && !is_zero(distance_plane_vertex))
        {
            edge = v.feature;
            return VClipStateResult::not_finished;
        }
    }

    //second face-edge planes of the edge
    voronois = edgepolytope.get_vpfe_edge(edge);
    for (auto& v : voronois) {
        Plane& p2 = v.plane;
        p2.transform(tovertex_fromedge, Isometry);

        Real distance_plane_vertex = p2.signedDistance(vertexpoint);
        //check not zero to avoid numerical problems
        if (distance_plane_vertex < 0 && !is_zero(distance_plane_vertex))
        {
            edge = v.feature;
            return VClipStateResult::not_finished;
        }
    }

    //clip edge.

    EdgePoints edgepoints = edgepolytope.get_edgepoints(edge);
    //transform edge tail and head to vertex coordinate system
    edgepoints.tail = tovertex_fromedge * edgepoints.tail;
    edgepoints.head = tovertex_fromedge * edgepoints.head;

    auto begin_planesvertex = vertexpolytope.begin_vpve_vertex(vertex);
    auto end_planesvertex = vertexpolytope.end_vpve_vertex(vertex);
    ClipEdge ce = clipedge(edgepoints, begin_planesvertex, end_planesvertex);

    if (ce.simply_excluded())
    {
        vertex = ce.feature_t;
        return VClipStateResult::not_finished;
    }

    //derivative check *working in vertex coordinate system
    if (ce.has_feature_t)
    {
        DerivativeResult dcheck = derivative_check_vertex_positive(vertexpoint,
                                                                   edgepoints,
                                                                   ce.lambda_t);
        if (dcheck == DerivativeResult::invalid) {
            lastresult.distance = 0.0;
            return VClipStateResult::penetration;
        } else if (dcheck == DerivativeResult::positive) {
            //update to FeatureT
            vertex = ce.feature_t;
            return VClipStateResult::not_finished;
        }
    }
    if (ce.has_feature_h)
    {
        DerivativeResult dcheck = derivative_check_vertex_positive(vertexpoint,
                                                                   edgepoints,
                                                                   ce.lambda_h);
        if (dcheck == DerivativeResult::invalid) {
            lastresult.distance = 0.0;
            return VClipStateResult::penetration;
        } else if (dcheck == DerivativeResult::negative) {
            //update to FeatureH
            vertex = ce.feature_h;
            return VClipStateResult::not_finished;
        }
    }

    lastresult.distance = dist_point_line(vertexpoint, edgepoints);
    return VClipStateResult::finished;
}

VClipStateResult VClip::edge_edge_check(VClipWitness &witness)
{
    SubCheckState subcheck = edge_edge_subcheck(cp_1, witness.feature1,
                                                cp_2, witness.feature2,
                                                to1_from2);
    if (subcheck == SubCheckState::feature_updated) {
        return VClipStateResult::not_finished;
    } else if (subcheck == SubCheckState::penetration) {
        lastresult.distance = 0.0;
        return VClipStateResult::penetration;
    }

    subcheck = edge_edge_subcheck(cp_2, witness.feature2,
                                  cp_1, witness.feature1,
                                  to2_from1);

    if (subcheck == SubCheckState::feature_updated) {
        return VClipStateResult::not_finished;
    } else if (subcheck == SubCheckState::penetration) {
        lastresult.distance = 0.0;
        return VClipStateResult::penetration;
    }

    EdgePoints edge1points = cp_1.get_edgepoints(witness.feature1);
    EdgePoints edge2points = cp_2.get_edgepoints(witness.feature2);
    edge2points.head = to1_from2 * edge2points.head;
    edge2points.tail = to1_from2 * edge2points.tail;
    lastresult.distance = dist_line_line(edge1points, edge2points);
    return VClipStateResult::finished;
}

// returns true on update
VClip::SubCheckState VClip::edge_edge_subcheck(const ConvexPolytope& polytope1,
                                               Feature& edge1,
                                               const ConvexPolytope& polytope2,
                                               Feature& edge2,
                                               const Transform &toEdge1FromEdge2)
{
    L_(ldebug4) << " VClip::edge_edge_subcheck ";
    L_(ldebug4) << "   edge1:  " << edge1;
    L_(ldebug4) << "   edge2:  " << edge2;
    EdgePoints edge2points = polytope2.get_edgepoints(edge2);
    // in coordinates of edge 1
    edge2points.tail = toEdge1FromEdge2 * edge2points.tail;
    edge2points.head = toEdge1FromEdge2 * edge2points.head;

    std::array<VoronoiPlane, 2> voronois1 = polytope1.get_vpve_edge(edge1);

    ClipEdge ce = clipedge(edge2points, voronois1.begin(), voronois1.end());

    // simply excluded.
    if (ce.simply_excluded())
    {
        L_(ldebug4) << "     simply excluded. update edge1 to  " << ce.feature_t;
        edge1 = ce.feature_t; // T and H are equal
        return SubCheckState::feature_updated;
    }
    else
    {
        //check derivatives, possibly update E 1 [Algo. 2] (Mirtich)
        if (ce.has_feature_t)
        {
            Point point = polytope1.get_vertex_point(ce.feature_t);
            DerivativeResult dcheck = derivative_check_vertex_positive(point,
                                                                       edge2points,
                                                                       ce.lambda_t);
            L_(ldebug4) << "    derivative test for Tail edge-vertex : " << dcheck;
            if (dcheck == DerivativeResult::invalid) {
                lastresult.distance = 0;
                return SubCheckState::penetration;
            } else if (dcheck == DerivativeResult::positive) {
                edge1 = ce.feature_t;
                return SubCheckState::feature_updated;
            }
        }
        if (ce.has_feature_h)
        {
            Point point = polytope1.get_vertex_point(ce.feature_h);
            DerivativeResult dcheck = derivative_check_vertex_positive(point,
                                                                       edge2points,
                                                                       ce.lambda_h);
            L_(ldebug4) << "    derivative test for Head edge-vertex  : " << dcheck;
            if (dcheck == DerivativeResult::invalid) {
                lastresult.distance = 0;
                return SubCheckState::penetration;
            } else if (dcheck == DerivativeResult::negative) {
                edge1 = ce.feature_h;
                return SubCheckState::feature_updated;
            }
        }
    }

    // Duplicated code here.
    // Now the same against face-edge planes
    voronois1 = polytope1.get_vpfe_edge(edge1);
    ce = clipedge(edge2points, voronois1.begin(), voronois1.end());

    // simply excluded.
    if (ce.simply_excluded())
    {
        edge1 = ce.feature_t; // T and H are equal
        return SubCheckState::feature_updated;
    }
    else
    {
        //check derivatives, possibly update E 1 [Algo. 2] (Mirtich)
        if (ce.has_feature_t)
        {
            Plane faceplane = polytope1.get_face_plane(ce.feature_t);
            DerivativeResult dcheck = derivative_check_face_positive(faceplane,
                                                                     edge2points,
                                                                     ce.lambda_t);
            if (dcheck == DerivativeResult::invalid) {
                lastresult.distance = 0;
                return SubCheckState::penetration;
            } else if (dcheck == DerivativeResult::positive) {
                edge1 = ce.feature_t;
                return SubCheckState::feature_updated;
            }
        }
        if (ce.has_feature_h)
        {
            Plane faceplane = polytope1.get_face_plane(ce.feature_h);
            DerivativeResult dcheck = derivative_check_face_positive(faceplane,
                                                                     edge2points,
                                                                     ce.lambda_h);
            if (dcheck == DerivativeResult::invalid) {
                lastresult.distance = 0;
                return SubCheckState::penetration;
            } else if (dcheck == DerivativeResult::negative) {
                edge1 = ce.feature_h;
                return SubCheckState::feature_updated;
            }
        }
    }

    return SubCheckState::feature_not_updated;
}

VClipStateResult VClip::vertex_face_check(VClipWitness &witness, bool swap)
{
    Feature& vertex = swap ? witness.feature2 : witness.feature1;
    Feature& face = swap ? witness.feature1 : witness.feature2;
    const ConvexPolytope& vertexpolytope = swap ? cp_2 : cp_1;
    const ConvexPolytope& facepolytope = swap ? cp_1 : cp_2;
    const Transform& tovertex_fromface = swap ? to2_from1 : to1_from2;
    const Transform& toface_fromvertex = swap ? to1_from2 : to2_from1;

    //first, search for the VP(f2,E) that v1 violates "maximally"
    Point vertexpoint = vertexpolytope.get_vertex_point(vertex);
    Point vertexPointCoordFace = toface_fromvertex * vertexpoint;

    Real min_distance = 0.0;
    // be careful, face can be updated in the for loop
    auto it_end = facepolytope.end_vpfe_face(face);
    for (auto it = facepolytope.begin_vpfe_face(face);
         it != it_end; it++) {
        VoronoiPlane v = *it;
        Plane& p2 = v.plane;
        Real distance = p2.signedDistance(vertexPointCoordFace);

        if (distance < min_distance)
        {
            min_distance = distance;
            face = v.feature;
        }
    }
    if (min_distance < 0.0)
    {
        return VClipStateResult::not_finished;
    }

    Plane faceplane = facepolytope.get_face_plane(face);
    // check all edges incident to v1 that point to faceplane.
    // now from vertex coordinate system.
    faceplane.transform(tovertex_fromface, Isometry);

    Real distance_p1face = faceplane.signedDistance(vertexpoint);

    for (auto it = vertexpolytope.begin_edgepoints_vertex(vertex);
         it != vertexpolytope.end_edgepoints_vertex(vertex); it++)
    {
        auto pair1 = *it;
        Real distance_pedge1_face = faceplane.signedDistance(pair1.second);

        // This is distinct from Mirtich algorithm. I believe there
        // is an error if the edge crosses the face.
        if ((abs(distance_p1face) > abs(distance_pedge1_face))
            || (distance_p1face * distance_pedge1_face < 0))
        {
            vertex = pair1.first;
            return VClipStateResult::not_finished;
        }
    }

    Real signedistance_p1_face = faceplane.signedDistance(vertexpoint);
    if (signedistance_p1_face > 0)
    {
        lastresult.distance = signedistance_p1_face;
        return VClipStateResult::finished;
    }

    // Here distance negative from face to vertex.
    // could be a local minimum or interpenetration.
    // This is handleLocalMin in Mirtich.

    // this updates the face if necessary.
    Real dmax = std::numeric_limits<Real>::lowest();
    Feature bestbace;
    for (auto it = facepolytope.begin_planes_faces(); it != facepolytope.end_planes_faces(); it++)
    {
        auto pair = *it;
        Real distplanepoint = pair.second.signedDistance(vertexPointCoordFace);
        if (distplanepoint > dmax)
        {
            dmax = distplanepoint;
            bestbace = pair.first;
        }
    }
    if (dmax <= 0)
    {
        lastresult.distance = dmax;
        // In Mirtich original algoritm the face is not updated in this casa.
        // This is a problem as it is not the best face, and also because the
        // cache will have a state that could run in O(n) continuously.
        face = bestbace;
        return VClipStateResult::penetration;
    }
    face = bestbace;
    return VClipStateResult::not_finished;
}

VClipStateResult VClip::edge_face_check(VClipWitness &witness, bool swap)
{
    Feature& edge = swap ? witness.feature2 : witness.feature1;
    Feature& face = swap ? witness.feature1 : witness.feature2;
    const ConvexPolytope& edgepolytope = swap ? cp_2 : cp_1;
    const ConvexPolytope& facepolytope = swap ? cp_1 : cp_2;
    const Transform& toedge_fromface = swap ? to2_from1 : to1_from2;
    const Transform& toface_fromedge = swap ? to1_from2 : to2_from1;
    Plane faceplane = facepolytope.get_face_plane(face);

    L_(ldebug4) << " VClip::edge_face_check";

    EdgePoints edgepoints = edgepolytope.get_edgepoints(edge);
    edgepoints.tail = toface_fromedge * edgepoints.tail;
    edgepoints.head = toface_fromedge * edgepoints.head;

    ClipEdge ce = clipedge(edgepoints,
                           facepolytope.begin_vpfe_face(face),
                           facepolytope.end_vpfe_face(face));


    if (!ce.not_completely_clipped)
    {
        L_(ldebug4) << " completely clipped. iterate over perimeter.";

        // We are repeating tests and we are applying the checks
        // to all features of the face, and that is not needed.
        // It could be enough to test the edges that are excluded from the face
        // face-edge voronoi planes (and their adjecent vertices)

        // The best way would be to get one feature in ce (clipedge)
        // and advance using the derivative check. This is a bit problematic
        // with the half-edge structure, as going over the half edges in clockwise
        // mode is not so direct.

        auto it_end = facepolytope.end_scan_face(face);
        for (auto it = facepolytope.begin_scan_face(face);
             it != it_end; it++)
        {
            const auto &perimeterfeature = *it;

            ClipEdge ceperimeter = clipedge(edgepoints,
                                            perimeterfeature.vertexedge_vps.begin(),
                                            perimeterfeature.vertexedge_vps.end());


            // the edge is not in this region.
            if (ceperimeter.excluded()) {
                continue;
            }

            if (perimeterfeature.feature.isEdge()) {
                // We should  not treat edges that are in the other side of
                // the tested face. This is not necessary in Mirtich.
                // What if it passes exactly over one of the face vertices?

                Vector point_lambda_t = edgepoints.point_at_lambda(ceperimeter.lambda_t);
                Vector point_lambda_h = edgepoints.point_at_lambda(ceperimeter.lambda_h);
                Real distancetail = perimeterfeature.faceedge_plane.signedDistance(point_lambda_t);
                Real distancehead = perimeterfeature.faceedge_plane.signedDistance(point_lambda_h);

                if ((distancetail > 0 || is_zero(distancetail))  && (distancehead > 0  || is_zero(distancehead)))
                {
                    continue;
                }
            }


            if (!ceperimeter.has_feature_t && !ceperimeter.has_feature_h)
            {
                face = perimeterfeature.feature;
                return VClipStateResult::not_finished;
            }

            DerivativeResult dcheck_t = DerivativeResult::invalid;
            if (ceperimeter.has_feature_t)
            {
                // if it is a vertex, derivative check against the vertex.
                // if it is an edge, we need the vertex of the edge.

                Point vertexpoint;
                if (perimeterfeature.feature.isVertex()) {
                    vertexpoint = facepolytope.get_vertex_point(perimeterfeature.feature);
                } else {
                    vertexpoint = facepolytope.get_vertex_point(ceperimeter.feature_t);
                }
                dcheck_t = derivative_check_vertex_positive(vertexpoint,
                                                            edgepoints,
                                                            ceperimeter.lambda_t);

                if (!ceperimeter.has_feature_h && (dcheck_t == DerivativeResult::negative)) {
                    // extreme limit.
                    face = perimeterfeature.feature;
                    return VClipStateResult::not_finished;
                }
            }

            DerivativeResult dcheck_h = DerivativeResult::invalid;
            if (ceperimeter.has_feature_h)
            {

                Point vertexpoint;
                if (perimeterfeature.feature.isVertex()) {
                    vertexpoint = facepolytope.get_vertex_point(perimeterfeature.feature);
                } else {
                    vertexpoint = facepolytope.get_vertex_point(ceperimeter.feature_h);
                }

                dcheck_h = derivative_check_vertex_positive(vertexpoint,
                                                            edgepoints,
                                                            ceperimeter.lambda_h);

                if (!ceperimeter.has_feature_t && (dcheck_h == DerivativeResult::positive)) {
                    // extreme limit.
                    face = perimeterfeature.feature;
                    return VClipStateResult::not_finished;
                }
            }

            if ((ceperimeter.has_feature_t) &&  (ceperimeter.has_feature_h))
            {
                if (dcheck_t != dcheck_h)
                {
                    // extreme limit.
                    face = perimeterfeature.feature;
                    return VClipStateResult::not_finished;
                }
            }
        }
        throw std::logic_error("we should not be here. VClip::edge_face_check.");
    }

    // edge is not excluded from the face. check if it crosses the face
    Vector point_lambda_t = edgepoints.point_at_lambda(ce.lambda_t);
    Real distance_to_lambda_t = faceplane.signedDistance(point_lambda_t);
    Vector point_lambda_h = edgepoints.point_at_lambda(ce.lambda_h);
    Real distance_to_lambda_h = faceplane.signedDistance(point_lambda_h);


    if ((distance_to_lambda_t * distance_to_lambda_h) < 0 ||
        is_zero(distance_to_lambda_t) || is_zero(distance_to_lambda_h)) //grrr.
    {
        // penetration
        if (distance_to_lambda_t < distance_to_lambda_h)
        {
            lastresult.distance = distance_to_lambda_t;
            lastresult.edge_face_max_penetration_point = toedge_fromface * point_lambda_t;
        } else {
            lastresult.distance = distance_to_lambda_h;
            lastresult.edge_face_max_penetration_point = toedge_fromface * point_lambda_h;
        }
        return VClipStateResult::penetration;
    }

    DerivativeResult dcheck = derivative_check_face_positive(faceplane,
                                                             edgepoints,
                                                             ce.lambda_t);

    if (dcheck == DerivativeResult::positive)
    {
        if (ce.has_feature_t)
        {
            face = ce.feature_t;
        }
        else
        {
            edge = edgepolytope.get_edge_tailvertex(edge);
        }
    }
    else
    {
        // check H
        if (ce.has_feature_h)
        {
            face = ce.feature_h;
        }
        else
        {
            edge = edgepolytope.get_edge_headvertex(edge);
        }
    }

    return VClipStateResult::not_finished;
}


std::ostream& OB::operator<< (std::ostream &out, const VClipState& state)
{
    switch(state)
    {
    case VClipState::vertex_vertex: out << "vertex_vertex";    break;
    case VClipState::vertex_edge: out << "vertex_edge";    break;
    case VClipState::edge_edge: out << "edge_edge";    break;
    case VClipState::vertex_face: out << "vertex_face";    break;
    case VClipState::edge_face: out << "edge_face";    break;
    default: out.setstate(std::ios_base::failbit);
    }
    return out;
}


std::ostream& OB::operator<< (std::ostream &out, const VClipResult &res)
{
    out << "{ VClipResult: " << std::endl;
    out << "    witness: " << res.witness << std::endl;
    out << "    penetrating: " << res.result << std::endl;
    out << "    distance: " << res.distance << std::endl;
    out << "}" << std::endl;
    return out;
}

std::ostream& OB::operator<< (std::ostream &out, const VClipWitness &w)
{
    out << "    feature1: " << w.feature1 << "    feature2: " << w.feature2;
    return out;
}

std::ostream& OB::operator<< (std::ostream &out, const VClipStateResult& state)
{
    switch(state)
    {
    case VClipStateResult::not_finished: out << "not_finished";    break;
    case VClipStateResult::finished: out << "finished NO PENETRATION";    break;
    case VClipStateResult::penetration: out << "penetration";    break;
    default: out.setstate(std::ios_base::failbit);
    }
    return out;
}
