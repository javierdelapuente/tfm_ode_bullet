#ifndef OBVCLIP_POLYTOPE_EXAMPLES_HPP
#define OBVCLIP_POLYTOPE_EXAMPLES_HPP

#include "polytope.hpp"
#include "oblog.hpp"
#include <libqhullcpp/RboxPoints.h>

namespace OB


{

    std::vector<Point> get_cube_points(Real halflength = 1)
    {
        std::vector<Point> points;
        points.push_back(Point(-1 , -1, -1) * halflength);
        points.push_back(Point(-1 , -1, -1) * halflength);
        points.push_back(Point(-1, -1, 1) * halflength);
        points.push_back(Point(-1, 1, -1) * halflength);
        points.push_back(Point(-1, 1, 1) * halflength);
        points.push_back(Point(1, -1, -1) * halflength);
        points.push_back(Point(1, -1, 1) * halflength);
        points.push_back(Point(1, 1, -1) * halflength);
        points.push_back(Point(1, 1, 1) * halflength);
        return points;

    }

    // center 0,0,0. half sides of length 1.
    inline OB::ConvexPolytope create_cube_polytope (Real halflength = 1) {
        std::vector<Point> points = get_cube_points(halflength);

        OB::ConvexPolytope poly;
        poly.generate_from_vertices(points);
        return poly;
    }

    std::vector<Point> get_pyramid_points(int baseNumberOfSides,
                                          Real height = 1.0,
                                          Real baseradius = 1.0)
    {
        assert (baseNumberOfSides > 2);
        std::vector<Point> points;
        Real angleBetweenVertices = (2 * PI) / baseNumberOfSides;
        for (int i = 0; i < baseNumberOfSides; ++i)
        {
            points.push_back(Point(baseradius * sin(angleBetweenVertices * i),
                                   0, baseradius * cos(angleBetweenVertices * i)));
        }
        // apex
        points.push_back(Point(0, height, 0));
        return points;
    }


    // base center 0,0,0. base vertices in sphere of radius one. height 1.
    inline OB::ConvexPolytope create_pyramid_polytope (int baseNumberOfSides,
                                                     Real height = 1.0,
                                                     Real baseradius = 1.0) {
        assert (baseNumberOfSides > 2);

        std::vector<Point> points = get_pyramid_points(baseNumberOfSides,
                                                       height,
                                                       baseradius);

        OB::ConvexPolytope poly;
        poly.generate_from_vertices(points);
        assert(poly.euler_number_correct());
        return poly;
    }



    std::vector<Point> get_prism_points(int polygonNumberOfSides,
                                      Real height = 1.0,
                                      Real baseradius = 1.0)
    {
        assert (polygonNumberOfSides > 2);
        std::vector<Point> points;

        Real angleBetweenVertices = (2 * PI) / polygonNumberOfSides;
        for (int i = 0; i < polygonNumberOfSides; ++i)
        {
            Point p1{static_cast<Real>(baseradius * sin(angleBetweenVertices * i)),
                     0,
                     static_cast<Real>(baseradius * cos(angleBetweenVertices * i))};
            Point p2{static_cast<Real>(baseradius * sin(angleBetweenVertices * i)),
                     height,
                     static_cast<Real>(baseradius * cos(angleBetweenVertices * i))};
            points.push_back(p1);
            points.push_back(p2);
        }

        return points;
    }

    // prism with height 1 and base vertices in sphere of radius one.
    inline OB::ConvexPolytope create_prism_polytope (int polygonNumberOfSides,
                                                   Real height = 1.0,
                                                   Real baseradius = 1.0) {
        OB::ConvexPolytope poly;
        poly.generate_from_vertices(get_prism_points(polygonNumberOfSides,
                                                                  height, baseradius));
        assert(poly.euler_number_correct());
        return poly;
    }


    inline OB::ConvexPolytope create_tetrahedron_polytope (Real baseradius = 1.0) {
        Real sidelength = 2.0 * baseradius * sin ( (2.0 * PI) / 6.0);
        L_(ldebug) << " side length " << sidelength;
        Real height = sqrt (2.0/3.0) * sidelength;
        return create_pyramid_polytope(3, height, baseradius);
    }


    std::vector<Point> get_bunny_points();
    inline OB::ConvexPolytope createBunnyPolytope () {

        std::vector<OB::Point> points = get_bunny_points();

        OB::ConvexPolytope poly;
        poly.generate_from_vertices(points);
        assert(poly.euler_number_correct());

        return poly; //check move constructor!
    }

    ///examples "1000 s D3", 1000 points in a 3d sphere
    inline std::vector<Point> get_rbox_points(const std::string& rboxcommand) {
        orgQhull::RboxPoints rbox {rboxcommand.c_str()}; 

        std::vector<Point> points;

        for (auto it = rbox.beginCoordinates(); it != rbox.endCoordinates(); ++it) {
            OB::Real x = static_cast<OB::Real>(*(it++));
            OB::Real y = static_cast<OB::Real>(*(it++));
            OB::Real z = static_cast<OB::Real>(*(it));

            points.push_back(OB::Point(x, y, z));
        }

        return points;
    }


    inline OB::ConvexPolytope create_rbox_polytope ( std::string& rboxcommand) {
        std::vector<Point> points = get_rbox_points(rboxcommand);
        OB::ConvexPolytope poly;
        poly.generate_from_vertices(points);
        assert(poly.euler_number_correct());

        return poly; //check move constructor!

    }

    // data points copied from ODE, file name "convex_bunny_geom.h"
    inline std::vector<Point> get_bunny_points()
    {
        std::vector<Real> coordinates
            {
             -0.459488, -0.093017, -0.311341,
             0.466635, -0.094416, -0.305669,
             -0.309239, 0.776868, 0.304726,
             -0.004458, -0.042526, 1.01567,
             8.40779e-45, 3.00321e-39, 2.8026e-44,
             0.007957, 0.282241, -0.93168,
             0.204445, -0.66438, 0.513353,
             -0.303961, 0.054199, 0.625921,
             0.265619, 0.756464, 0.504187,
             -0.402162, 0.133528, -0.443247,
             8.40779e-45, 3.00321e-39, 2.8026e-44,
             -0.266772, 0.64233, 0.602061,
             8.40779e-45, 3.00321e-39, 2.8026e-44,
             8.40779e-45, 3.00321e-39, 2.8026e-44,
             0.411612, 0.132299, -0.438264,
             0.31148, 0.775931, 0.308527,
             0.300086, 0.053287, 0.62962,
             -0.414624, 0.164083, -0.278254,
             -0.248382, 0.255825, -0.627493,
             -0.216201, -0.126776, -0.886936,
             0.267564, -0.666174, -0.654834,
             -0.135892, -0.03552, 0.945455,
             -0.265837, 0.757267, 0.500933,
             -0.003873, 0.161605, 0.970499,
             8.40779e-45, 3.00321e-39, 2.8026e-44,
             -0.282599, -0.663393, 0.412411,
             0.007237, 0.361687, -0.794439,
             0.093627, 0.258494, -0.920589,
             0.422146, 0.162819, -0.27313,
             0.279163, -0.664604, 0.417328,
             0.263086, 0.512567, 0.637832,
             -0.099875, 0.310931, -0.799381,
             -0.446838, -0.118517, -0.466159,
             -0.168842, 0.102387, -0.920381,
             0.455805, -0.119881, -0.460632,
             0.337743, -0.666396, -0.074503,
             -0.134547, -0.119852, -0.959004,
             -0.183807, 0.19697, 0.84448,
             0.264969, 0.641527, 0.605317,
             -0.209063, -0.663393, 0.509344,
             -0.364126, -0.200299, 0.202388,
             -0.253475, -0.081797, 0.756541,
             0.260471, 0.255056, -0.624378,
             0.114248, 0.310608, -0.79807,
             0.364663, -0.201399, 0.20685,
             0.127847, -0.035919, 0.94707,
             8.40779e-45, 3.00321e-39, 2.8026e-44,
             -0.381071, -0.629723, -0.350777,
             -0.339884, -0.04115, -0.668211,
             -0.077913, 0.258753, -0.92164,
             0.184061, 0.101854, -0.91822,
             -0.335166, -0.66538, -0.078623,
             0.386561, -0.625221, -0.21687,
             8.40779e-45, 3.00321e-39, 2.8026e-44,
             -0.241585, 0.527592, 0.669296,
             -0.086969, 0.133224, 0.947633,
             -0.003127, 0.28407, 0.87887,
             -0.004433, -0.146642, 0.985872,
             8.40779e-45, 3.00321e-39, 2.8026e-44,
             -0.138444, -0.10425, 0.945975,
             -0.265676, 0.513366, 0.634594,
             8.40779e-45, 3.00321e-39, 2.8026e-44,
             0.247593, -0.082554, 0.75961,
             0.07941, 0.132973, 0.948652,
             0.238615, 0.526867, 0.672237,
             8.40779e-45, 3.00321e-39, 2.8026e-44,
             8.40779e-45, 3.00321e-39, 2.8026e-44,
             -0.382112, -0.62406, -0.221577,
             -0.104072, 0.177278, -0.95253,
             0.351567, -0.042194, -0.663976,
             0.138234, -0.293905, -0.897958,
             0.119916, 0.17694, -0.951159,
             -0.371322, -0.665382, -0.35362,
             -0.263384, -0.663396, 0.466604,
             0.376722, -0.666513, -0.219833,
             0.387086, -0.630883, -0.346073,
             -0.125544, 0.140012, 0.917678,
             -0.070612, 0.036849, 0.975733,
             -0.083497, -0.084934, 0.979607,
             0.259286, -0.664547, 0.471281,
             8.40779e-45, 3.00321e-39, 2.8026e-44,
             0.074888, -0.085173, 0.980577,
             0.152305, 0.125256, 0.890786,
             0.130184, -0.104656, 0.94762,
             -0.004249, 0.046042, 1.00324,
             0.062419, 0.036648, 0.976547,
             8.40779e-45, 3.00321e-39, 2.8026e-44,
             -0.392666, -0.488581, -0.427494,
             0.230315, -0.12745, -0.884202,
             8.40779e-45, 3.00321e-39, 2.8026e-44,
             0.193434, -0.665946, -0.715325,
             0.007865, 0.122104, -0.956137,
             8.40779e-45, 3.00321e-39, 2.8026e-44,
             -0.257884, -0.665381, -0.658052,
             0.377265, -0.666513, -0.349036,
             -0.372362, -0.665381, -0.22442,
             0.400045, -0.489778, -0.42264,
             -0.159174, 0.125726, 0.888878,
             0.118369, 0.139643, 0.919173,
             -0.124463, -0.293508, -0.899566,
             0.21172, -0.302754, -0.843303,
             0.149571, -0.120281, -0.957264,
             -0.183019, -0.665378, -0.71763,
             0.177696, 0.196424, 0.846693,
             -0.198638, -0.302135, -0.845816,
            };
        std::vector<Point> points;
        for ( int i = 0; i < coordinates.size(); i+=3 )
        {
            points.push_back(Point(coordinates[i],
                                   coordinates[i+1],
                                   coordinates[i+2]));
        }
        return points;
    }

}

#endif
