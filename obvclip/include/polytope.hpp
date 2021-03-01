#ifndef OBVCLIP_POLYTOPE_HPP
#define OBVCLIP_POLYTOPE_HPP

#include <Eigen/Dense>
#include <vector>
#include <limits>
#include <iterator>
#include <memory>
#include <iostream>

#include <oblog.hpp>
#include <obconfig.hpp>


//forward declarations
namespace orgQhull {class Qhull; class QhullFacet;}

namespace OB
{

    class ConvexPolytope;

    constexpr FeatureId InvalidFeatureId = (std::numeric_limits<FeatureId>::max)();

    enum class FeatureType
        {
         vertex, edge, face, empty
        };


    struct Feature
    {
        Feature() {}
        Feature(FeatureId fid, FeatureType t)
            :feature(fid), type(t) {}
        FeatureId feature = -1;
        FeatureType type = FeatureType::empty;

        bool isVertex() const { return type == FeatureType::vertex; }
        bool isEdge() const { return type == FeatureType::edge; }
        bool isFace() const { return type == FeatureType::face; }

        friend bool operator<(const Feature& l, const Feature& r)
        {
            return std::tie(l.type, l.feature) < std::tie(r.type, r.feature);
        }
        friend bool operator<=(const Feature& l, const Feature& r)
        {
            return std::tie(l.type, l.feature) <= std::tie(r.type, r.feature);
        }

        friend bool operator==(const Feature& l, const Feature& r)
        {
            return (l.type == r.type && l.feature == r.feature);
        }
        friend bool operator!=(const Feature& l, const Feature& r)
        {
            return !(l==r);
        }

        friend std::ostream& operator<< (std::ostream &out, const Feature &f);
    };

    struct EdgePoints
    {
        EdgePoints() = default;
        EdgePoints(const Point& t, const Point& h)
            :tail(t), head(h)
        {}
        Point tail = Point::Zero();
        Point head = Point::Zero();
        friend std::ostream& operator<< (std::ostream &out, const EdgePoints &f)
        {
            out << "{ tail: " << f.tail.transpose() << " head: " << f.head.transpose() << "}";
            return out;
        }
        Point vector() const
        {
            return head - tail;
        }
        Point midpoint() const
        {
            return 0.5 * (head + tail);
        }

        Point point_at_lambda(Real lambda) const {
            return (1-lambda) * tail + lambda * head;
        }
    };

    std::ostream& operator<< (std::ostream &out, const Plane& p);
    struct VoronoiPlane
    {
        VoronoiPlane(const Plane& p, Feature f): plane(p), feature(f) {}

        Plane plane;
        Feature feature; // feature on the other side of the plane
        friend std::ostream& operator<< (std::ostream &out, const VoronoiPlane &vp) {
            out << " {VoronoiPlane plane: " << vp.plane << " feature: " << vp.feature << " }";
            return out;
        }
    };

    struct FacePerimeterFeature {
        FacePerimeterFeature(const Feature& f,
                             const std::array<VoronoiPlane, 2>& ps)
            : feature(f), vertexedge_vps(ps) {}

        FacePerimeterFeature(const Feature& f,
                             const std::array<VoronoiPlane, 2>& ps,
                             const Plane& fep)
            : feature(f), vertexedge_vps(ps), faceedge_plane(fep)  {}

        Feature feature; // vertex or edge
        std::array<VoronoiPlane, 2> vertexedge_vps; //voronoi planes (edge-vertex)
        Plane faceedge_plane{Vector::Zero(),0}; // ONLY FOR EDGES GRRRR
    };

    // some simple helper functions. 
    Real dist_point_point(const Point& point1, const Point& point2);
    Real dist_point_line(const Point& point1, const EdgePoints& edge);
    // absolute distance
    Real absdist_point_plane(const Point& point, const Plane& plane);
    Real dist_line_line(const EdgePoints& edge1, const EdgePoints& edge2);
    Real dist_point_edge(const Point& point, const EdgePoints& edge);
    Real dist_edge_edge(const EdgePoints& edge1, const EdgePoints& edge2);
    // convex face, determined by list of edges in CCW order.
    // returns an absolut distance. Very slow.
    Real absdist_point_face(const Point& point,
                           const Plane& facePlane,
                           const std::vector<EdgePoints>& faceEdgesCcw);
    // very slow.
    Real absdist_edge_face(const EdgePoints& edgePoints,
                          const Plane& facePlane,
                          const std::vector<EdgePoints>& faceEdgesCcw);

    class WorldConvexPolytope;
    Real absdist_between_features(const WorldConvexPolytope& wcp1, const Feature& f1,
                                  const WorldConvexPolytope& wcp2, const Feature& f2);


    // return true if there is an intersection point.
    std::pair<bool, Point> intersect_line_plane(const EdgePoints& edgePoints,
                                                const Plane& facePlane);

    //edge is a line. returns a point in that line, not clipped
    Point closest_point_to_line(const Point& point, const EdgePoints& edge);
    std::pair<Point, Point> closests_line_to_line(const EdgePoints& e1, const EdgePoints& e2);



    class WorldConvexPolytope
    {
    public:
        WorldConvexPolytope(const WorldConvexPolytope&) = delete;
        WorldConvexPolytope& operator=(const WorldConvexPolytope&) = delete;

        WorldConvexPolytope(const std::string& name, std::shared_ptr<ConvexPolytope> cv)
            :name(name), polytope(std::move(cv)) {}

        WorldConvexPolytope(const std::string& name,
                            std::shared_ptr<ConvexPolytope> cv,
                            const Transform& p)
            : name(name), polytope(std::move(cv)), pose(p) {}


        const ConvexPolytope& convex_polytope() const { return *polytope; }

        void set_pose (const Transform& newPose) { pose = newPose; }
        void set_pose (const Translation& translation, const Quaternion& q)
        { pose = translation * q; }

        const Transform& get_pose() const { return pose; }
        const std::string& get_name() const { return name; }

        friend std::ostream& operator<< (std::ostream &out, const WorldConvexPolytope &wcp);

        // there is only one object equal to this one. This one.
        //This weird thing is for vclip cache comparison
        bool operator == ( const WorldConvexPolytope& other) const
        { return this == std::addressof(other); }
        bool operator != (const WorldConvexPolytope &other) const
        { return this != std::addressof(other); }
        //Arbitrary order, this is for the cache.
        bool operator> (const WorldConvexPolytope &other) const
        { return this > std::addressof(other); }
        bool operator<= (const WorldConvexPolytope &other) const
        { return this <= std::addressof(other); }
        bool operator< (const WorldConvexPolytope &other) const
        { return this < std::addressof(other); }
        bool operator>= (const WorldConvexPolytope &other) const
        { return this >= std::addressof(other); }


    private:
        const std::string name;
        const std::shared_ptr<ConvexPolytope> polytope;
        Transform pose = Transform::Identity();
    };
}

namespace std
{
    template<> struct hash<::OB::WorldConvexPolytope> {
        size_t operator()(const ::OB::WorldConvexPolytope& val) const{
            return std::hash<const ::OB::WorldConvexPolytope*>{}(std::addressof(val));
        }
    };
}

namespace OB
{
    class Vertex
    {
    public:
        Vertex() = default;
        Vertex(const Point& pos):position{pos} {}
        bool has_halfedge() const { return halfedge != InvalidFeatureId; }
        FeatureId get_halfedge_id() const {return halfedge; }
        void set_halfedge_id(FeatureId new_halfedge_id)  { halfedge = new_halfedge_id; }

        const Point& get_position() const { return position; }
    private:
        Point position;
        // TODO we could put this field in the fourth position of vector3d.
        // Outgoing half edge (this vertex is the tail vertex of the half edge)
        FeatureId halfedge{InvalidFeatureId};
        friend std::ostream& operator<< (std::ostream &out, const Vertex &vertex);
    };
    using VertexList = std::vector<Vertex>;

    class HalfEdge
    {
    public:
        //Pair half edge
        void set_pair_id(FeatureId new_pair_id) { pair = new_pair_id; }
        FeatureId get_pair_id() const { return pair; }

        void set_next_id(FeatureId new_next) { next_halfedge_id = new_next; }
        FeatureId get_next_id() const { return next_halfedge_id; }

        void set_headvertex_id(FeatureId new_headvertex_id) { headvertex_id = new_headvertex_id; }
        FeatureId get_headvertex_id() const { return headvertex_id; }

        void set_face_id(FeatureId new_face_id) { face_id = new_face_id; }
        FeatureId get_face_id() const { return face_id; }

        const Plane& get_faceedge_plane() const { return faceedge_plane; }
        void set_faceedge_plane(const Plane& p) { faceedge_plane = p; }

        const Plane& get_headvertexedge_plane() const  { return headvertexedge_plane; }
        void set_headvertexedge_plane(const Plane& p) { headvertexedge_plane = p; }

    private:
        //we could have this implicit, having the list
        //of halfedges with the pairs in consecutive positions.
        //for now do it this way.
        FeatureId pair;
        // next in face, counter clock wise
        FeatureId next_halfedge_id;
        FeatureId headvertex_id;
        FeatureId face_id;

        // the hyperplanes normals point to the Voronoi Region
        // of the Edge. Must be flipped when necessary.
        Plane faceedge_plane;
        Plane headvertexedge_plane; // plane vertex-edge for the head vertex
        friend std::ostream& operator<< (std::ostream &out, const HalfEdge &halfEdge);
    };
    using HalfEdgeList = std::vector<HalfEdge>;

    class Face
    {
    public:
        Face(const Plane& plane) : plane(plane) {}
        void set_halfedge_id(FeatureId new_halfedge_id) { halfedge_id = new_halfedge_id; }
        FeatureId get_halfedge_id() const { return halfedge_id; }

        const Plane& get_plane() const { return plane; }

        //this functions are slow, for debugging.
        Point get_center(const ConvexPolytope& cv) const;
    private:
        FeatureId halfedge_id{InvalidFeatureId};
        Plane plane;
        friend std::ostream& operator<< (std::ostream &out, const Face &face);
    };
    using FaceList = std::vector<Face>;


    class ConvexPolytope
    {
    public:
        ConvexPolytope():generated{false} {}
        ConvexPolytope(const ConvexPolytope& o) = default;
        void generate_from_vertices(const std::vector<Point>& points);
        bool euler_number_correct();

        //This are the functions used in vclip, that are do not
        //depend on the half edge structure (could be winged edge or other)

        //get voronoi planes vertex edge VP(V,E) for a vertex
        std::vector<VoronoiPlane> get_vpve_vertex(Feature vid) const {
            return std::vector<VoronoiPlane>(begin_vpve_vertex(vid), end_vpve_vertex(vid));
        }

        //VP(V,E), voronoi plane vertex edge (vpve).
        class it_vpve_vertex;
        it_vpve_vertex begin_vpve_vertex(Feature vertex_id) const {
            return it_vpve_vertex(*this, vertex_id, true);
        }
        it_vpve_vertex end_vpve_vertex(Feature vertex_id) const  {
            return it_vpve_vertex(*this, vertex_id, false);
        }

        //VP(V,E)
        std::array<VoronoiPlane, 2> get_vpve_edge(Feature hid) const; 

        //VP(F,E)
        std::array<VoronoiPlane, 2> get_vpfe_edge(Feature hid) const; 

         //VP(F,E)
        std::vector<VoronoiPlane> get_vpfe_face(Feature fid) const {
            return std::vector<VoronoiPlane>(begin_vpfe_face(fid),
                end_vpfe_face(fid));
        }
        //VP(F,E)
        class it_vpfe_face;
        it_vpfe_face begin_vpfe_face(Feature fid) const {
            return it_vpfe_face(*this, fid, true);
        }
        it_vpfe_face end_vpfe_face(Feature fid) const {
            return it_vpfe_face(*this, fid, false);
        }

        Plane get_face_plane(Feature fid) const {
            assert (fid.type == FeatureType::face);
            return get_face(fid.feature).get_plane();
        }

        // first = tail, second = head
        EdgePoints get_edgepoints(Feature heid) const {
            assert (heid.type == FeatureType::edge);
            const HalfEdge &he = get_halfedge(heid.feature);
            const Point& head = get_vertex(he.get_headvertex_id()).get_position();
            const Point& tail =
                get_vertex(get_halfedge(he.get_pair_id()).get_headvertex_id()).get_position();
            return EdgePoints{tail, head};
        }

        // edges (outgoind half edge) from a vertex with the other point in edge
        std::vector<std::pair<Feature, Point>> get_edgepoints_for_vertex(Feature vid) const {
            return  std::vector<std::pair<Feature, Point>>(begin_edgepoints_vertex(vid),
                end_edgepoints_vertex(vid));
        }

        // edges (outgoind half edge) from a vertex with the other point in edge
        class it_ep_vertex;
        it_ep_vertex begin_edgepoints_vertex(Feature vid) const {
            return it_ep_vertex(*this, vid, true);
        }
        it_ep_vertex end_edgepoints_vertex(Feature vid) const {
            return it_ep_vertex(*this, vid, false);
        }

        Point get_vertex_point(Feature v) const {
            assert (v.type == FeatureType::vertex);
            return get_vertex(v.feature).get_position();
        }

        std::vector<Point> get_face_points(Feature fid) const;
        std::vector<Feature> get_face_vertices(Feature fid) const;
        std::vector<EdgePoints> get_face_edgepoints(Feature fid) const;

        // slow and too much memory allocated, rethink this
        std::vector<std::pair<Feature, Plane>> get_faces_with_planes() const {
            std::vector<std::pair<Feature, Plane>> resp;
            for (auto it = faces.begin(); it != faces.end(); ++it)
            {
                FeatureId fid = std::distance(faces.begin(), it);
                resp.emplace_back(Feature{fid, FeatureType::face}, it->get_plane());
            }
            L_(linfo) << "get_faces_with_planes " << resp.size();
            return resp;
        };

        class it_planes_faces;
        it_planes_faces begin_planes_faces() const { return it_planes_faces(faces, faces.begin()); }
        it_planes_faces end_planes_faces() const { return it_planes_faces(faces, faces.end()); }

        // For face voronoi region exclusion. Mirtich 3.3.3
        // the VoronoiPlane info is the opposite of the normal way.
        // the feature it includes is the current one, not the other side one
        std::vector<FacePerimeterFeature> get_scan_face(Feature fid) const {
            return  std::vector<FacePerimeterFeature >(begin_scan_face(fid),
                end_scan_face(fid));
        }

        class it_scanning_face;
        it_scanning_face begin_scan_face(Feature fid) const {
            return it_scanning_face(*this, fid, true);
        }
        it_scanning_face end_scan_face(Feature fid) const {
            
            return it_scanning_face(*this, fid, false);
        }


        Feature get_edge_headvertex(Feature fe) const {
            assert (fe.type == FeatureType::edge);
            return Feature{get_halfedge(fe.feature).get_headvertex_id(), FeatureType::vertex };
        }

        Feature get_edge_tailvertex(Feature fe) const {
            assert (fe.type == FeatureType::edge);
            return Feature{get_halfedge(get_halfedge(fe.feature).get_pair_id()).get_headvertex_id(),
                    FeatureType::vertex };
        }



        // maybe private?
        const Vertex& get_vertex(FeatureId vId) const{
            return vertices[vId];
        }
        const HalfEdge& get_halfedge(FeatureId heId) const{
            return halfedges[heId];
        }
        HalfEdge& get_halfedge(FeatureId heId){
            return halfedges[heId];
        }
        const Face& get_face(FeatureId faceId) const{
            return faces[faceId];
        }
        Face& get_face(FeatureId faceId) {
            return faces[faceId];
        }
        FaceList& get_faces() { return faces; }
        const FaceList& get_faces() const { return faces; }
        VertexList& get_vertices() { return vertices; }
        const VertexList& get_vertices() const { return vertices; }
        HalfEdgeList& get_halfedges() { return halfedges; }

        void get_local_aabb(Vector& min_point, Vector &max_point) const;

        // this is just for testing
        void write(std::ostream& ost);
        static ConvexPolytope create(std::istream& ifs);


        // IDEAS ON ITERATORS:
        // https://blog.cppse.nl/cpp-multiple-iterators-with-traits
        // QUITAR EL std::iterator, que está deprecado!
        class it_vpve_vertex : public std::iterator<std::input_iterator_tag, VoronoiPlane>
        {
        public:
            explicit it_vpve_vertex(const ConvexPolytope& polytope, Feature vertex_id, bool first)
                :polytope(polytope), halfedge_id(polytope.get_vertex(vertex_id.feature).get_halfedge_id()), first(first)
            {
                assert(vertex_id.type == FeatureType::vertex);
            };
            value_type operator*() const
            {
                FeatureId nextPairId = polytope.get_halfedge(halfedge_id).get_pair_id();
                const HalfEdge& incomingHalfEdge = polytope.get_halfedge(nextPairId);
                VoronoiPlane vp =  VoronoiPlane(incomingHalfEdge.get_headvertexedge_plane(),
                                                Feature{nextPairId, FeatureType::edge});
                vp.plane.coeffs()*=-1;
                return vp;

            }
            it_vpve_vertex operator++() {
                it_vpve_vertex i = *this;
                first = false;
                advance();
                return i;
            }
            it_vpve_vertex operator++(int junk) {
                first = false;
                advance();
                return *this;
            }
            bool operator!=(const it_vpve_vertex &o) const
            {
                return halfedge_id != o.halfedge_id || first != o.first;
            }
        private:
            inline void advance() {
                halfedge_id = polytope.get_halfedge(halfedge_id).get_pair_id();
                halfedge_id = polytope.get_halfedge(halfedge_id).get_next_id();
            }
            const ConvexPolytope& polytope;
            FeatureId halfedge_id;
            bool first;
        };

        class it_vpfe_face : public std::iterator<std::input_iterator_tag, VoronoiPlane>
        {
        public:
            explicit it_vpfe_face(const ConvexPolytope& polytope, Feature fid, bool first)
                :polytope(polytope), halfedge_id(polytope.get_face(fid.feature).get_halfedge_id())
                ,first(first) {
                assert(fid.type == FeatureType::face);
            }
            value_type operator*() const
            {
                const HalfEdge &he = polytope.get_halfedge(halfedge_id);
                VoronoiPlane vp =  VoronoiPlane(he.get_faceedge_plane(),
                                                Feature{halfedge_id, FeatureType::edge});
                vp.plane.coeffs()*=-1;
                return vp;
            }
            it_vpfe_face operator++() {
                it_vpfe_face i = *this;
                advance();
                return i;
            }
            it_vpfe_face operator++(int junk) {
                advance();
                return *this;
            }
            bool operator!=(const it_vpfe_face &o) const
            {
                return halfedge_id != o.halfedge_id || first != o.first;
            }
        private:
            inline void advance() {
                first = false;
                halfedge_id = polytope.get_halfedge(halfedge_id).get_next_id();
            }
            const ConvexPolytope& polytope;
            FeatureId halfedge_id;
            bool first;
        };


        //ep is edge point. this iterates over all the points connected by an edge to that vertex,
        //except the vertex original point.
        class it_ep_vertex : public std::iterator<std::input_iterator_tag, std::pair<Feature, Point>>
        {
        public:
            explicit it_ep_vertex(const ConvexPolytope& polytope, Feature vertex_id, bool first)
                :polytope(polytope), 
                halfedge_id(polytope.get_vertex(vertex_id.feature).get_halfedge_id()),
                first(first) {
                assert(vertex_id.type == FeatureType::vertex);
            }
            value_type operator*() const
            {
                const HalfEdge& outgoingHalfEdge = polytope.get_halfedge(halfedge_id);
                Point otherPoint = polytope.get_vertex(outgoingHalfEdge.get_headvertex_id()).get_position();
                return std::make_pair(Feature{halfedge_id, FeatureType::edge}, otherPoint);
            }
            it_ep_vertex operator++() {
                it_ep_vertex i = *this;
                advance();
                return i;
            }
            it_ep_vertex operator++(int junk) {
                advance();
                return *this;
            }
            bool operator!=(const it_ep_vertex &o) const
            {
                return halfedge_id != o.halfedge_id || first != o.first;
            }
        private:
            inline void advance() {
                first = false;
                const HalfEdge& outgoingHalfEdge = polytope.get_halfedge(halfedge_id);
                halfedge_id = outgoingHalfEdge.get_pair_id(); //ingoing
                halfedge_id = polytope.get_halfedge(halfedge_id).get_next_id(); //outgoing
            }
            const ConvexPolytope& polytope;
            FeatureId halfedge_id;
            bool first;
        };

        class it_scanning_face : public std::iterator<std::input_iterator_tag, FacePerimeterFeature>
        {
        public:
            explicit it_scanning_face(const ConvexPolytope& polytope,
                                      Feature face_id, bool first)
                :polytope(polytope), 
                halfedge_id(polytope.get_face(face_id.feature).get_halfedge_id()), first(first) {
                assert(face_id.type == FeatureType::face);
            }

            value_type operator*() const
            {
                const HalfEdge &he = polytope.get_halfedge(halfedge_id);
                const HalfEdge &pairhe = polytope.get_halfedge(he.get_pair_id());
                Feature fedge{halfedge_id, FeatureType::edge};
                Feature fvertex{he.get_headvertex_id(), FeatureType::vertex };
                if (in_edge)
                {
                    const Vertex &vertex = polytope.get_vertex(fvertex.feature);
                    Feature bvertex{pairhe.get_headvertex_id(), FeatureType::vertex };
                    Plane backPlane = pairhe.get_headvertexedge_plane();
                    Plane frontPlane = he.get_headvertexedge_plane();
                    Plane faceEdgePlane = he.get_faceedge_plane();
                    faceEdgePlane.coeffs()*=-1;
                    return FacePerimeterFeature{
                        fedge, std::array<VoronoiPlane, 2>{VoronoiPlane(backPlane, bvertex),
                                VoronoiPlane(frontPlane, fvertex)}, faceEdgePlane
                    };
                }
                else
                {
                    Feature fNextEdge{he.get_next_id(), FeatureType::edge};
                    const HalfEdge &nextPairHe = polytope.get_halfedge(polytope.get_halfedge(he.get_next_id()).get_pair_id());
                    Plane backNextEdgePlane = nextPairHe.get_headvertexedge_plane();
                    Plane backPlane = pairhe.get_headvertexedge_plane();
                    Plane frontPlane = he.get_headvertexedge_plane();
                    frontPlane.coeffs()*=-1;
                    backNextEdgePlane.coeffs()*=-1;
                    return FacePerimeterFeature{
                        fvertex,
                            std::array<VoronoiPlane, 2>{VoronoiPlane(frontPlane, fedge),
                                VoronoiPlane(backNextEdgePlane, fNextEdge)}
                    };
                }
            }
            it_scanning_face operator++() {
                it_scanning_face i = *this;
                advance();
                return i;
            }
            it_scanning_face operator++(int junk) {
                advance();
                return *this;
            }
            bool operator!=(const it_scanning_face &o) const
            {
                return (halfedge_id != o.halfedge_id ||
                        first != o.first || in_edge != o.in_edge);
            }

        private:
            inline void advance() {
                first = false;
                if (!in_edge) {
                    const HalfEdge &he = polytope.get_halfedge(halfedge_id);
                    halfedge_id = he.get_next_id();
                }
                in_edge = !in_edge;
            }
            const ConvexPolytope& polytope;
            bool in_edge = true;
            FeatureId halfedge_id;
            bool first;
        };


        class it_planes_faces : public std::iterator<std::input_iterator_tag,
                                                     std::pair<Feature, Plane>>
        {
        public:
            explicit it_planes_faces(const FaceList& list, FaceList::const_iterator it)
                :list(list), it(it) {}

            value_type operator*() const
            {
                FeatureId fid = std::distance(list.begin(), it);
                return std::make_pair(Feature{fid, FeatureType::face}, it->get_plane());
            }
            it_planes_faces operator++() {
                it_planes_faces i = *this;
                ++it;
                return i;
            }
            it_planes_faces operator++(int junk) {
                ++it;
                return *this;
            }
            bool operator!=(const it_planes_faces &o) const
            {
                return it!=o.it;
            }

        private:
            const FaceList& list;
            FaceList::const_iterator it;
        };


    private:
        bool generated;
        VertexList vertices;
        HalfEdgeList halfedges;
        FaceList faces;
        //TODO MAYBE CENTER/ORIGIN/VOLUME OR THE CONVEX POLYTOPE??

        void create_vertices(const orgQhull::Qhull&);
        void create_face_and_halfedges(orgQhull::QhullFacet&);
        void pair_halfedges();
        void generate_voronoi_planes();


        friend std::ostream& operator<< (std::ostream &out, const ConvexPolytope &point);
    };

    std::ostream& operator<< (std::ostream &out, const Vertex &vertex);
    std::ostream& operator<< (std::ostream &out, const Face &face);
    std::ostream& operator<< (std::ostream &out, const HalfEdge &halfEdge);
    std::ostream& operator<< (std::ostream &out, const ConvexPolytope &cp);
    std::ostream& operator<< (std::ostream &out, const WorldConvexPolytope &wcp);
    std::ostream& operator<< (std::ostream &out, const Feature& f);
    std::ostream& operator<< (std::ostream &out, const FeatureType& ft);


}


#endif
