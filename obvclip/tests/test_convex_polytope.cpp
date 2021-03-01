#include "gtest/gtest.h"
#include <fstream>

#include "polytope.hpp"
#include "polytope_examples.hpp"

using namespace OB;

TEST(TESTConvexPolytope, basicTest) {
  std::vector<Point> points;
  points.push_back(Point(-1, -1, -1));
  points.push_back(Point(-1, -1, 1));
  points.push_back(Point(-1, 1, -1));
  points.push_back(Point(-1, 1, 1));
  points.push_back(Point(1, -1, -1));
  points.push_back(Point(1, -1, 1));
  points.push_back(Point(1, 1, -1));
  points.push_back(Point(1, 1, 1));

  ConvexPolytope poly;
  poly.generate_from_vertices(points);
  ASSERT_EQ(poly.euler_number_correct(), true);

  //TODO ASSERT, 8 VERTICES, 6 FACES
  //TODO CHECK ONE FACE CCW.
  //TODO ASSERT EULER NUMBER

  std::vector<std::pair<Feature, Plane>> allfacesplanes = poly.get_faces_with_planes();
  ASSERT_EQ(allfacesplanes.size(), 6);

  Feature f0 = allfacesplanes[0].first;
  ASSERT_EQ(poly.get_face_edgepoints(f0).size(), 4);

  ASSERT_EQ(poly.get_scan_face(f0).size(), 8);

  std::cout << poly << std::endl;
}

TEST(TESTConvexPolytope, testPrismPolytope) {
  std::shared_ptr<OB::ConvexPolytope> prism = std::make_shared<OB::ConvexPolytope>(create_prism_polytope (3));
  //std::cout << "PRISM: " << *prism << std::endl;
}


TEST(TESTConvexPolytope, shortestBetweenPointEdge) {

  // distance equal segment and line
  Point point{1,1,1};
  Point tail{7, 7, 7};
  Point head{1, 0, -1};
  Vector director = head - tail;
  EdgePoints edge {tail, head};
  std::cout << "distance from point " << point.transpose()
            << " to segment: " << edge << std::endl;
  ASSERT_FLOAT_EQ(dist_point_edge(point, edge), 1.2040201117631721);

  // now one in which the distance is smaller in the head
  head = head + (tail - director*0.3);
  edge = EdgePoints{tail, head};
  std::cout << "distance from point " << point.transpose()
            << " to segment: " << edge << std::endl;
  ASSERT_FLOAT_EQ(dist_point_edge(point, edge), 10.392304);

  // now one in which the distance is smaller in the tail
  head = Point{1, 0, -1} + (tail + director*10);
  tail = tail  + (tail + director*10);
  edge = EdgePoints{tail, head};
  std::cout << "distance from point " << point.transpose()
            << " to segment: " << edge << std::endl;
  ASSERT_FLOAT_EQ(dist_point_edge(point, edge), 99.73465);

  // one with the point in the line
  point = Point {1,1,1};
  edge = EdgePoints{{0,1,1}, {2,1,1}};
  ASSERT_FLOAT_EQ(dist_point_edge(point, edge), 0.0);

  // one with the point in the line not in segment
  point = Point {0,1,1};
  edge = EdgePoints{{1,1,1}, {2,1,1}};
  ASSERT_FLOAT_EQ(dist_point_edge(point, edge), 1.0);

}


TEST(TESTConvexPolytope, shortestBetweenEdgeEdge) {
  // for line - line, we can use https://keisan.casio.com/exec/system/1223531414#


  // caso degenerado, segmentos paralelos
  EdgePoints edge1{Point{0, 0, 1}, Point{0, 2, 1}};
  EdgePoints edge2{Point{1, 1, 0}, Point{1, -1, 0}};
  std::cout << "distance from segment: " << edge1
            << " to segment: " << edge2 << std::endl;
  ASSERT_FLOAT_EQ(dist_edge_edge(edge1, edge2), 1.4142135);


  // otro caso degenerado, pero el punto mas cercano
  // no pertenece a los segmentos.
  edge1 = EdgePoints{Point{0, 0, 1}, Point{0, 2, 1}};
  edge2 = EdgePoints{Point{1, -1, 0}, Point{1, -2, 0}};
  std::cout << "distance from segment: " << edge1
            << " to segment: " << edge2 << std::endl;
  ASSERT_FLOAT_EQ(dist_edge_edge(edge1, edge2), 1.7320508);

  // mismo segmento
  edge1 = EdgePoints{Point{0, 0, 1}, Point{0, 2, 1}};
  edge2 = EdgePoints{Point{0, 0, 1}, Point{0, 2, 1}};
  std::cout << "distance from segment: " << edge1
            << " to segment: " << edge2 << std::endl;
  ASSERT_FLOAT_EQ(dist_edge_edge(edge1, edge2), 0.0);


  // minimum points inside both segments
  edge1 = EdgePoints{Point{0,4,2}, Point{0,0,2}};
  edge2 = EdgePoints{Point{-1,3,0}, Point{2,0,0}};
  std::cout << "distance from segment: " << edge1
            << " to segment: " << edge2 << std::endl;
  ASSERT_FLOAT_EQ(dist_edge_edge(edge1, edge2), 2.0);


  // minimum points in only one segment and one of the edges
  edge1 = EdgePoints{Point{0,1.5,2}, Point{0,0,2}};
  edge2 = EdgePoints{Point{-1,3,0}, Point{2,0,0}};
  std::cout << "distance from segment: " << edge1
            << " to segment: " << edge2 << std::endl;
  ASSERT_FLOAT_EQ(dist_edge_edge(edge1, edge2), 2.0310097);

  // minimum points in two edge points
  edge1 = EdgePoints{Point{0,0,1}, Point{0,0,0}};
  edge2 = EdgePoints{Point{1,1,-1}, Point{1,2,-1}};
  std::cout << "distance from segment: " << edge1
            << " to segment: " << edge2 << std::endl;
  ASSERT_FLOAT_EQ(dist_edge_edge(edge1, edge2), 1.7320508);

}



TEST(TESTConvexPolytope, readwritefromfile) {
  std::ostringstream oss;
  OB::ConvexPolytope tetrahedron = create_tetrahedron_polytope(1.0);
  tetrahedron.write(oss);
  std::string first = oss.str();

  std::istringstream iss{first};
  OB::ConvexPolytope leido = OB::ConvexPolytope::create(iss);

  std::ostringstream oss2;
  leido.write(oss2);
  std::string second = oss2.str();

  ASSERT_EQ(first, second);

}

