
#include <ostream>

#include "gtest/gtest.h"
#include <Eigen/Dense>

#include "libqhull_r/qhull_ra.h"

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
#include "polytope.hpp"


TEST(TESTQHull, basicTest) {
    constexpr int dimension = 3;

    // box
    constexpr int numberOfPoints  = 9;
    double points[numberOfPoints*dimension]{-1, -1, -1,
            -1, -1, 1,
            -1, 1, -1,
            -1, 1, 1,
            1, -1, -1,
            1, -1, 1,
            1, 1, -1,
            1, 1, 1,
            0, 0, 0};
    orgQhull::Qhull q {"", dimension, numberOfPoints, points, "o"};
    orgQhull::QhullFacetList facets= q.facetList();
    //std::cout << facets << std::endl;

    for (auto &facet : q.facetList())
    {
        std::cout << "Facet> " << facet;
        orgQhull::QhullVertexSet vertices = facet.vertices();
        for (const auto &vertex : vertices)
      	{
            std::cout << "Vertex> " << vertex;
      	}

        Eigen::Vector3d center(facet.getCenter().coordinates());
        std::cout << " center: " << center << std::endl;
        Eigen::Vector3d normal(facet.hyperplane().coordinates());
        std::cout << " normal: " << normal << std::endl;
        orgQhull::QhullVertexSet::iterator it = vertices.begin();
        Eigen::Vector3d firstVertex((*it).point().coordinates());
        std::cout << " firstVertex: " << firstVertex  << std::endl;
        firstVertex = (firstVertex - center);
        firstVertex.normalize();
        std::cout << " firstVertex: " << firstVertex  << std::endl;;
        std::vector<std::pair<double, orgQhull::QhullVertex> > orderedVertices;
        orderedVertices.push_back(std::pair<double, orgQhull::QhullVertex> (0.0, (*it)));
        it++;
        // first element is the angle.

        for (;it < vertices.end(); it++) {
            Eigen::Vector3d vertex((*it).point().coordinates());
            std::cout << " vertex: " << vertex << " id: " << (*it).id()  << std::endl;
            vertex = vertex - center;
            vertex.normalize();
            std::cout << " vertex: " << vertex << std::endl;
            std::cout << " cos(angle) " << firstVertex.dot(vertex) << std::endl;
            std::cout << " (angle) " << acos(firstVertex.dot(vertex)) << std::endl;
            double angle = acos(firstVertex.dot(vertex));
            Eigen::Vector3d crossProductFirstAndCurrent = firstVertex.cross(vertex);
            std::cout << " cross product " << crossProductFirstAndCurrent << " is zero " << crossProductFirstAndCurrent.isZero() << std::endl;
            std::cout << " dot(crossProductFirstAndCurrent, normal) " << crossProductFirstAndCurrent.dot(normal) << std::endl;
            if (!crossProductFirstAndCurrent.isZero() && crossProductFirstAndCurrent.dot(normal) < 0)
            {
                angle = -angle + 2 * EIGEN_PI;
            }
            //std::cout << " final angle: " << angle << std::endl;
            std::pair<double, orgQhull::QhullVertex> pair(angle, (*it));
            orderedVertices.push_back(std::pair<double, orgQhull::QhullVertex> (angle, (*it)));
        }

        std::sort( std::begin(orderedVertices), std::end(orderedVertices),
                   []( const std::pair<double, orgQhull::QhullVertex> &left, const std::pair<double, orgQhull::QhullVertex> &right)
                   {
                       return  left.first < right.first;
                   } );
        for (auto &v : orderedVertices)
        {
            std::cout << " ordered vertex: " << v.second;
        }

        std::cout << " ALL vertices: " << std::endl;
        for (auto v = q.beginVertex(); v != q.endVertex(); v = v.next()){
            std::cout << "ID: " << v.id() << "->" << v;
        }

        std::cout << "ALL vertices again: " << q.vertexList() << std::endl;
        std::cout << "ALL points: " << q.points() << std::endl;

    }

    //spdlog::info("A basic test");
    //spdlog::info("sizeof realT {} double {}", sizeof(realT), sizeof(double));
    // orgQhull::RboxPoints eg("100");
    // orgQhull::Qhull q(eg, "");


    ASSERT_EQ(true, true);
}

void print_summary(qhT *qh) {
    facetT *facet;
    vertexT *vertex, **vertexp;
    int k;

    printf("\n%d vertices and %d facets with normals:\n",
           qh->num_vertices, qh->num_facets);
    FORALLfacets {
        printf("FACET\n");
        for (k=0; k < qh->hull_dim; k++)
        {
            printf("%6.2g ", facet->normal[k]);

        }
        printf("\n");
        printf("center "); for (k=0; k < qh->hull_dim; k++) {printf("%6.2g", facet->center[k]);} printf("\n");
        printf(" toporient %i\n", facet->toporient);
        FOREACHvertex_(facet->vertices) {
            printf("vertex ");
            for (int k=0; k < qh->hull_dim; k++)
                printf("%5.2f ", vertex->point[k]);
            printf("\n");
        }
    }


}


TEST(TESTQHull, DISABLED_cTest) {
    constexpr int dim = 3;
    constexpr int numpoints = 8;
    coordT points[numpoints*dim] = {-1, -1, -1,
                                    -1, -1, 1,
                                    -1, 1, -1,
                                    -1, 1, 1,
                                    1, -1, -1,
                                    1, -1, 1,
                                    1, 1, -1,
                                    1, 1, 1};

    boolT ismalloc= False; /* True if qhull should free points in qh_freeqhull() or reallocation */
    //char flags[] = "qhull QJ";
    char flags[255];
    sprintf(flags, "qhull s Tcv Fx");
    FILE *outfile= stdout;
    FILE *errfile= stderr;
    int exitcode;
    facetT *facet;
    int curlong, totlong;

    qhT qh_qh;
    qhT *qh= &qh_qh;

    QHULL_LIB_CHECK
    
        qh_zero(qh, errfile);

    exitcode= qh_new_qhull(qh, dim, numpoints, points, ismalloc,
                           flags, outfile, errfile);
    fflush(NULL);

    if (!exitcode) {                  /* if no error */
        print_summary(qh);
        facetT *facet;
        vertexT *vertex, **vertexp;
        FORALLfacets {
            /* ... your code ... */
            FOREACHvertex_(facet->vertices) {
                // for (int k=0; k < dim; k++)
                //       printf("%5.2f ", vertex->point[k]);
            }
        }
    }

    qh_freeqhull(qh, !qh_ALL);                   /* free long memory  */
    qh_memfreeshort(qh, &curlong, &totlong);    /* free short memory and memory allocator */
    if (curlong || totlong)
        fprintf(errfile, "qhull internal warning (user_eg, #1): did not free %d bytes of long memory (%d pieces)\n", totlong, curlong);
}


TEST(TESTQHull, qhullPrism) {
    constexpr int dimension = 3;

    // box
    constexpr int numberOfPoints  = 6;
    double points[numberOfPoints*dimension]{
                     0, 0, 1,
                         0, 1, 1,
                         0.866025, 0, -0.5,
                         0.866025, 1, -0.5,
                         -0.866025, 0, -0.5,
                         -0.866025, 1, -0.5};
    orgQhull::Qhull q {"", dimension, numberOfPoints, points, "o"};


    std::cout << " ALL facets: " << std::endl;
    orgQhull::QhullFacetList facets= q.facetList();
    std::cout << facets << std::endl;

    std::cout << " ALL vertices: " << std::endl;
    for (auto v = q.beginVertex(); v != q.endVertex(); v = v.next()){
        std::cout << "ID: " << v.id() << "->" << v;
    }

}


TEST(TESTQHull, rbox)
{
    orgQhull::RboxPoints rbox {"10 s D3"}; /// 10 points in a 3d sphere

    auto itbegin = rbox.beginCoordinates();
    auto itend = rbox.endCoordinates();

    int count = 0;
    for (auto it = rbox.beginCoordinates(); it != rbox.endCoordinates(); ++it)
    {
        std::cout << "coordinate: " << (*it) << std::endl;
        count++;
    }
    std::cout << "number coordinates: " << count;

    orgQhull::Qhull q {rbox, "o"};

    std::vector<OB::Point> points;
    for (auto it = rbox.beginCoordinates(); it != rbox.endCoordinates(); ++it) {
        OB::Real x = static_cast<OB::Real>(*(it++));
        OB::Real y = static_cast<OB::Real>(*(it++));
        OB::Real z = static_cast<OB::Real>(*(it));

        points.push_back(OB::Point(x, y, z));

    }

    for (auto& p : points) {
        std::cout << " Points " << p.transpose() << std::endl;
    }

    int numberOfPoints = points.size();
    std::vector<realT> pointCoordinates(numberOfPoints * 3);

    for (size_t i = 0; i < points.size(); i++)
    {
        const OB::Point& p = points[i];
        pointCoordinates[i*3+0] = static_cast<realT>(p(0));
        pointCoordinates[i*3+1] = static_cast<realT>(p(1));
        pointCoordinates[i*3+2] = static_cast<realT>(p(2));
    }
    orgQhull::Qhull q2 {"", 3, numberOfPoints, pointCoordinates.data(), "o"};


    // std::cout << " ALL facets: " << std::endl;
    // orgQhull::QhullFacetList facets= q.facetList();
    // std::cout << facets << std::endl;

    // std::cout << " ALL vertices: " << std::endl;
    // for (auto v = q.beginVertex(); v != q.endVertex(); v = v.next()){
    //     std::cout << "ID: " << v.id() << "->" << v;
    // }
}
