#ifndef OBVCLIP_VCLIP_HPP
#define OBVCLIP_VCLIP_HPP

#include <iostream>
#include <unordered_map>
#include "polytope.hpp"

namespace OB
{

    constexpr int max_iterations = 5000;
    constexpr bool fail_on_max_iterations = false;

    // in world coordinates
    struct ContactDataVClip
    {
        Point point;
        Vector normal;
        Real depth;
        Point point_in_body_2;
    };

    enum class VClipState { vertex_vertex,
                            vertex_edge,
                            edge_edge,
                            vertex_face,
                            edge_face };

    // same as in Mirtich
    enum class VClipStateResult {
        not_finished, finished, penetration
    };

    struct VClipCacheKey
    {
        VClipCacheKey(const WorldConvexPolytope& wcpa, const WorldConvexPolytope& wcpb):
            wcp1(wcpa), wcp2(wcpb) {}
        const WorldConvexPolytope& first() const { return wcp1; }
        const WorldConvexPolytope& second() const { return wcp2; }

        VClipCacheKey swapped() const {
            return VClipCacheKey{second(), first()};
        }

        bool ordered() const {
            return first() <= second();
        }

        bool operator == ( const VClipCacheKey& other) const
        {
            return first() == other.first() && second() == other.second();
        }

    private:
        const WorldConvexPolytope& wcp1;
        const WorldConvexPolytope& wcp2;
    };
}

//Is there other way of doing this not using std namespace?
namespace std
{
    template<> struct hash<::OB::VClipCacheKey> {
        size_t operator()(const ::OB::VClipCacheKey& val) const{
            std::size_t h1 = std::hash< ::OB::WorldConvexPolytope>{}(val.first());
            std::size_t h2 = std::hash< ::OB::WorldConvexPolytope>{}(val.second());
            return h1 ^ (h2 << 1);
        }
    };
}

namespace OB
{


    struct VClipWitness
    {
        VClipWitness () : feature1{0, FeatureType::empty},
                          feature2{0, FeatureType::empty}
        {}
        VClipWitness(Feature f1, Feature f2) : feature1(f1),
                                               feature2(f2)
        {}

        static VClipWitness FirstVertices()
        {
            return VClipWitness{Feature{0, FeatureType::vertex}, Feature{0, FeatureType::vertex}};
        }

        bool lowerDimension(const VClipWitness& o)
        {
            if (((feature1.type < o.feature1.type) && (feature2.type == o.feature2.type)) ||
                ((feature1.type == o.feature1.type) && (feature2.type < o.feature2.type)))
            {
                return true;
            }
            return false;
        }

        Feature feature1;
        Feature feature2;
        VClipWitness swapped() const { return VClipWitness{feature2, feature1}; }
        friend std::ostream& operator<< (std::ostream &out, const VClipWitness &w);
    };


    class VClipCache
    {
    public:
        bool has_witness(const VClipCacheKey& key)
        {
            return ((key.ordered() ? cache.find(key) : cache.find(key.swapped())) != cache.end());
        }

        VClipWitness get_witness(const VClipCacheKey& key)
        {
            if (key.ordered())
            {
                return cache.at(key);
            }
            else
            {
                return cache.at(key.swapped()).swapped();
            }
        }

        void set_witness(const VClipCacheKey& key, const VClipWitness& witness)
        {
            if (key.ordered())
            {
                cache[key] = witness;
            }
            else
            {
                cache[key.swapped()] = witness.swapped();
            }
        }

    private:
        std::unordered_map<VClipCacheKey, VClipWitness> cache;
    };

    struct VClipResult
    {
        VClipWitness witness;
        VClipState state;
        VClipStateResult result = VClipStateResult::not_finished;
        Real distance = std::numeric_limits<Real>::quiet_NaN();
        //for edge-face in edge local coordinates. clipping point negative
        Point edge_face_max_penetration_point{0,0,0};

        bool penetrating() const {
            return result == VClipStateResult::penetration;
        }
        friend std::ostream& operator<< (std::ostream &out, const VClipResult &res);

    };


    // worlds objects should not be CHANGED AT ALL after VClip created.
    // VClip does not own its members.
    class VClip
    {
    public:
        // be careful with the order of the members in the class
        VClip(const WorldConvexPolytope& wcp1,
              const WorldConvexPolytope& wcp2) :world_cp_1(wcp1),
                                                world_cp_2(wcp2),
                                                cp_1(wcp1.convex_polytope()),
                                                cp_2(wcp2.convex_polytope()),
                                                to1_from2(wcp1.get_pose().inverse(Isometry) * wcp2.get_pose()),
                                                to2_from1(to1_from2.inverse(Isometry))
        {}

        VClipResult operator () (VClipCache& cache);
        VClipResult solve(VClipWitness witness);

        VClipResult iterate();
        const VClipWitness& get_currentwitness() { return currentwitness; }
        void set_currentwitness(const VClipWitness& w) { currentwitness = w ; }

        const VClipResult& get_lastresult() const { return lastresult; }

    private:

        const WorldConvexPolytope& world_cp_1;
        const WorldConvexPolytope& world_cp_2;
        const ConvexPolytope& cp_1;
        const ConvexPolytope& cp_2;
        const Transform to1_from2;
        const Transform to2_from1;
        VClipWitness currentwitness;
        VClipResult lastresult;
        VClipState state;

        //update features. return true if finished
        VClipStateResult check_features(VClipWitness &witness);

        // feature is a reference to the result. it is overwritten.
        VClipStateResult vertex_vertex_check(VClipWitness &witness);
        VClipStateResult vertex_edge_check(VClipWitness &witness, bool swap);
        VClipStateResult edge_edge_check(VClipWitness &witness);
        VClipStateResult vertex_face_check(VClipWitness &witness, bool swap);
        VClipStateResult edge_face_check(VClipWitness &witness, bool swap);

        // return true on update feature
        enum class SubCheckState {
            feature_updated, feature_not_updated, penetration
        };

        SubCheckState edge_edge_subcheck(const ConvexPolytope& polytope1,
                                       Feature& edge1,
                                       const ConvexPolytope& polytope2,
                                       Feature& edge2,
                                       const Transform &toedge1_fromedge2);



        VClipState update_state(const VClipWitness& witness, bool swap);
    };

    std::ostream& operator<< (std::ostream &out, const VClipState& state);
    std::ostream& operator<< (std::ostream &out, const VClipStateResult& state);
    std::ostream& operator<< (std::ostream &out, const VClipResult &res);
    std::ostream& operator<< (std::ostream &out, const VClipWitness &w);

}

#endif
