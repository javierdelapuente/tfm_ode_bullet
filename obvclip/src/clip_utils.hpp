#ifndef OBVCLIP_CLIPUTILS_HPP
#define OBVCLIP_CLIPUTILS_HPP

#include "polytope.hpp"
#include "oblog.hpp"

namespace OB
{
    enum class DerivativeResult { positive, negative, zero, invalid };

    struct ClipEdge
    {
        Real lambda_t = 0.0; //closer to tail, \underline{lambda} in Mirtich
        Real lambda_h = 1.0; //closer to head, \overline{lambda} in Mirtich
        Feature feature_t; // \underline{N} in Mirtich
        Feature feature_h; // \overline{N} in Mirtich
        bool has_feature_t = false;
        bool has_feature_h = false;
        bool not_completely_clipped = false;
        friend std::ostream& operator<< (std::ostream &out, const ClipEdge &ce)
        {
            out << "{ClipEdge lambda_t: "<<ce.lambda_t<<" lambda_h:" << ce.lambda_h
                <<" feature_t:" << ce.feature_t << " feature_h:" << ce.feature_h
                <<" has_feature_t:" << ce.has_feature_t
                <<" has_feature_h:" << ce.has_feature_h
                << " not_completely_clipped:" << ce.not_completely_clipped << "}";
            return out;
        }

        bool excluded()
        {
            return !not_completely_clipped;
        }

        bool simply_excluded()
        {
            if (!not_completely_clipped && has_feature_t && has_feature_h &&
                feature_t == feature_h)
                return true;
            return false;
        }

        bool compoundly_excluded()
        {
            if (!not_completely_clipped && has_feature_t && has_feature_h &&
                feature_t != feature_h)
                return true;
            return false;
        }
    };

    // Planes and edge in the same coordinate system.
    template<class iterator_type>
    ClipEdge clipedge(EdgePoints edge_points,
                      iterator_type begin, iterator_type end)
    {
        ClipEdge res;
        for (iterator_type it = begin; it != end; ++it)
        {
            const VoronoiPlane& p = *it;
            const Feature& feature = p.feature;
            const Plane& plane = p.plane;
            Real dist_to_t = plane.signedDistance(edge_points.tail);
            Real dist_to_h = plane.signedDistance(edge_points.head);
            if ((dist_to_t < 0 && !is_zero(dist_to_t))  && (dist_to_h < 0 && !is_zero(dist_to_h))) {
                res.feature_t = feature;
                res.has_feature_t = true;
                res.feature_h = feature;
                res.has_feature_h = true;
                res.not_completely_clipped = false;
                return res;
            } else if (dist_to_t < 0  && !is_zero(dist_to_t)) {
                Real lambda = dist_to_t / (dist_to_t - dist_to_h);
                if (lambda > res.lambda_t) {
                    res.lambda_t = lambda;
                    res.has_feature_t = true;
                    res.feature_t = feature;
                    if (res.lambda_t > res.lambda_h) {
                        res.not_completely_clipped = false;
                        return res;
                    }
                }
            } else if (dist_to_h < 0 && !is_zero(dist_to_h)) {
                Real lambda = dist_to_t / (dist_to_t - dist_to_h);
                if (lambda < res.lambda_h) {
                    res.lambda_h = lambda;
                    res.has_feature_h = true;
                    res.feature_h = feature;
                    if (res.lambda_t > res.lambda_h) {
                        res.not_completely_clipped = false;
                        return res;
                    }
                }
            }
        }
        res.not_completely_clipped = true;
        return res;
    }



    inline DerivativeResult derivative_check_vertex_positive(Point vertex,
                                                             EdgePoints edge_points,
                                                             Real lambda)
    {
        Point e_lambda_minus_vertex = edge_points.point_at_lambda(lambda) - vertex;
        if (e_lambda_minus_vertex.isZero())
        {
            return DerivativeResult::invalid;
        }

        Real ep_dot_el = edge_points.vector().dot(e_lambda_minus_vertex);

        //should we compare with is_zero?
        if (ep_dot_el == 0.0) return DerivativeResult::zero;
        return ( ep_dot_el > 0) ?
            DerivativeResult::positive:
            DerivativeResult::negative;
    }

    inline DerivativeResult derivative_check_face_positive(Plane face_plane,
                                                           EdgePoints edge_points,
                                                           Real lambda)
    {
        Point e_lambda = edge_points.point_at_lambda(lambda);
        Real distance_plane = face_plane.signedDistance(e_lambda);

        if (is_zero(distance_plane)) return DerivativeResult::invalid;

        Real u_dot_n = (edge_points.vector()).dot(face_plane.normal());

        //should we compare with is_zero?
        if (u_dot_n == 0.0) return DerivativeResult::zero;
        if (distance_plane > 0)
        {
            return (u_dot_n > 0) ? DerivativeResult::positive : DerivativeResult::negative;
        }
        //negative distance
        return (u_dot_n < 0) ? DerivativeResult::positive : DerivativeResult::negative;
    }


    inline std::ostream& operator<< (std::ostream &out, const DerivativeResult &dr)
    {
        switch(dr)
        {
        case DerivativeResult::positive: out << "positive";    break;
        case DerivativeResult::negative: out << "negative"; break;
        case DerivativeResult::invalid: out << "invalid";  break;
        case DerivativeResult::zero: out << "zero";  break;
        default: out << "?????";  break; //out.setstate(std::ios_base::failbit);
        }
        return out;
    }

}

#endif
