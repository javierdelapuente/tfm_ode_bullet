#include <ostream>
#include <fstream>

#include "gtest/gtest.h"
#include "polytope.hpp"
#include "polytope_examples.hpp"
#include "vclip.hpp"
#include "ogrevclip_app.hpp"

using namespace OB;

void display (OB::WorldConvexPolytope& wcp0,
              OB::WorldConvexPolytope& wcp1,
              OB::VClipWitness& witness)
{
  OB::OgreVClipApp app;
  app.initApp();
  app.addWorldConvexPolytope(wcp0);
  app.addWorldConvexPolytope(wcp1);
  app.setVClipPair(wcp0, wcp1, witness);
  app.getRoot()->startRendering();
  app.closeApp();
}



class VClipTest : public ::testing::Test {
protected:
  void SetUp() override {
    {
      std::ifstream ifs{"./data/tetrahedron.txt"};
      if(ifs.fail()){
          throw std::logic_error("./data/tetrahedron.txt file error");
      }
      tetrahedron = std::make_shared<OB::ConvexPolytope>(ConvexPolytope::create(ifs));
    }
    {
      std::ifstream ifs{"./data/pyramid5.txt"};
      if(ifs.fail()){
          throw std::logic_error("./data/tetrahedron.txt file error");
      }
      pyramid5 = std::make_shared<OB::ConvexPolytope>(ConvexPolytope::create(ifs));
    }
    {
      std::ifstream ifs{"./data/prism6.txt"};
      if(ifs.fail()){
          throw std::logic_error("./data/tetrahedron.txt file error");
      }
      prism6 = std::make_shared<OB::ConvexPolytope>(ConvexPolytope::create(ifs));
    }
    {
      std::ifstream ifs{"./data/cube.txt"};
      if(ifs.fail()){
          throw std::logic_error("./data/tetrahedron.txt file error");
      }
      cube = std::make_shared<OB::ConvexPolytope>(ConvexPolytope::create(ifs));
    }
    {
      std::ifstream ifs{"./data/prism20.txt"};
      if(ifs.fail()){
          throw std::logic_error("./data/tetrahedron.txt file error");
      }
      prism20 = std::make_shared<OB::ConvexPolytope>(ConvexPolytope::create(ifs));
    }

  }

  // void TearDown() override {}

  std::shared_ptr<OB::ConvexPolytope> tetrahedron;
  std::shared_ptr<OB::ConvexPolytope> pyramid5;
  std::shared_ptr<OB::ConvexPolytope> prism6;
  std::shared_ptr<OB::ConvexPolytope> prism20;
  std::shared_ptr<OB::ConvexPolytope> cube;

};

TEST_F(VClipTest, vertexvertexTest) {

  //1. test v1 and v2 in voronoi region
  {
    OB::WorldConvexPolytope wcp0{"obj-1", tetrahedron};
    OB::WorldConvexPolytope wcp1{"obj-2", pyramid5};

    OB::Transform t;
    t = OB::Translation(0, -3 ,0);
    wcp0.set_pose(t);
    t = OB::Translation(0, 3 ,0) * OB::AngleAxis(OB::PI, OB::Vector::UnitZ().normalized());
    wcp1.set_pose(t);
    VClip vclip(wcp0, wcp1);
    VClipWitness witness = VClipWitness(Feature{2, FeatureType::vertex},
                                        Feature{2, FeatureType::vertex});
    //display(wcp0, wcp1, witness);
    vclip.set_currentwitness(witness);
    VClipStateResult result = vclip.iterate().result;
    ASSERT_EQ(result, VClipStateResult::finished);
  }

  //2. v2 not in VP(v1). updates obj1
  {
    OB::Transform t;
    t = OB::Translation(2, -3 ,0) * OB::AngleAxis(OB::PI, OB::Vector::UnitZ().normalized());
    OB::WorldConvexPolytope wcp0{"obj-1", tetrahedron, t};
    t = OB::Translation(0, 3 ,0) * OB::AngleAxis(OB::PI, OB::Vector::UnitZ().normalized());
    OB::WorldConvexPolytope wcp1{"obj-2", pyramid5, t};
    VClipWitness witness = VClipWitness(Feature{2, FeatureType::vertex}, Feature{0, FeatureType::vertex});
    //display(wcp0, wcp1, witness);
    VClip vclip(wcp0, wcp1);
    vclip.set_currentwitness(witness);
    VClipStateResult result = vclip.iterate().result;
    const VClipWitness& finalWitness = vclip.get_currentwitness();
    ASSERT_EQ(result, VClipStateResult::not_finished);
    ASSERT_EQ(finalWitness.feature1, Feature(0, FeatureType::edge));
    ASSERT_EQ(finalWitness.feature2, Feature(0, FeatureType::vertex));
  }


  //3. v2 in VP(v1). v1 not in VP(v2). updates obj2
  {
    OB::Transform t;
    t = OB::Translation(1, -3 ,0);
    OB::WorldConvexPolytope wcp0{"obj-1", tetrahedron, t};
    t = OB::Translation(2, 3 ,0) * OB::AngleAxis(1.03 * OB::PI,
                                                 OB::Vector(0.1, 1, 1).normalized());
    OB::WorldConvexPolytope wcp1{"obj-2", pyramid5, t};
    VClipWitness witness = VClipWitness(Feature{2, FeatureType::vertex},
                                        Feature{1, FeatureType::vertex});
    //display(wcp0, wcp1, witness);
    VClip vclip(wcp0, wcp1);
    vclip.set_currentwitness(witness);
    VClipStateResult result = vclip.iterate().result;
    const VClipWitness& finalWitness = vclip.get_currentwitness();
    ASSERT_EQ(result, VClipStateResult::not_finished);
    ASSERT_EQ(finalWitness.feature1, Feature(2, FeatureType::vertex));
    ASSERT_EQ(finalWitness.feature2, Feature(5, FeatureType::edge));
  }


}


TEST_F(VClipTest, vertexedgeTest) {
  //1. v violates e voronio plane VP(E), type vertex-edge.
  {
    OB::Transform t;
    t = OB::Translation(1, -3 ,0) * OB::AngleAxis(1.03 * OB::PI, OB::Vector(0.8, 0.2, 0.4).normalized());
    OB::WorldConvexPolytope wcp0{"obj-1", tetrahedron, t};
    t = OB::Translation(2, 3 ,0) * OB::AngleAxis(1.03 * OB::PI, OB::Vector(0.1, 1, 1).normalized());
    OB::WorldConvexPolytope wcp1{"obj-2", prism6, t};
    VClipWitness witness(Feature{0, FeatureType::edge}, Feature{1, FeatureType::vertex});
    //display(wcp0, wcp1, witness);
    VClip vclip(wcp0, wcp1);
    vclip.set_currentwitness(witness);
    VClipStateResult result = vclip.iterate().result;
    const VClipWitness& finalWitness = vclip.get_currentwitness();
    ASSERT_EQ(result, VClipStateResult::not_finished);
    ASSERT_EQ(finalWitness.feature1, Feature(0, FeatureType::vertex));
    ASSERT_EQ(finalWitness.feature2, Feature(1, FeatureType::vertex));
  }

  //2. v violates e voronio plane VP(E), type edge-face. (v does not violate VP(E))
  {
    OB::Transform t;
    t = OB::Translation(-1, -3 ,0) * OB::AngleAxis(1.03 * OB::PI, OB::Vector(0.8, 0.2, 0.4).normalized());
    OB::WorldConvexPolytope wcp0{"obj-1", tetrahedron, t};
    t = OB::Translation(2, 3 ,0) * OB::AngleAxis(1.03 * OB::PI, OB::Vector(0.1, 1, 1).normalized());
    OB::WorldConvexPolytope wcp1{"obj-2", prism6, t};
    VClipWitness witness(Feature{0, FeatureType::edge}, Feature{1, FeatureType::vertex});
    //display(wcp0, wcp1, witness);
    VClip vclip(wcp0, wcp1);
    vclip.set_currentwitness(witness);
    VClipStateResult result = vclip.iterate().result;
    const VClipWitness& finalWitness = vclip.get_currentwitness();
    ASSERT_EQ(result, VClipStateResult::not_finished);
    ASSERT_EQ(finalWitness.feature1, Feature(0, FeatureType::face));
    ASSERT_EQ(finalWitness.feature2, Feature(1, FeatureType::vertex));
  }

  //3. v does not violate plane and e simply excluded
  {
    OB::Transform t;
    t = OB::Transform::Identity();
    OB::WorldConvexPolytope wcp0{"obj-1", cube, t};
    t = OB::Translation(4, -0.5, -0.5) * OB::AngleAxis(-0.25* OB::PI, OB::Vector::UnitY().normalized());
    OB::WorldConvexPolytope wcp1{"obj-2", cube, t};
    VClipWitness witness(Feature{9, FeatureType::edge}, Feature{3, FeatureType::vertex});
    //display(wcp0, wcp1, witness);
    VClip vclip(wcp0, wcp1);
    vclip.set_currentwitness(witness);
    VClipStateResult result = vclip.iterate().result;
    const VClipWitness& finalWitness = vclip.get_currentwitness();
    ASSERT_EQ(result, VClipStateResult::not_finished);
    ASSERT_EQ(finalWitness.feature1, Feature(9, FeatureType::edge));
    ASSERT_EQ(finalWitness.feature2, Feature(21, FeatureType::edge));
  }


  //4. v does not violate plane and e not simply excluded and derivatives mark T
  {
    OB::Transform t;
    t = OB::Transform::Identity();
    OB::WorldConvexPolytope wcp0{"obj-1", cube, t};
    t = OB::Translation(3.1, -2.2, 1) *
      OB::AngleAxis(0.25 * OB::PI, OB::Vector(-1, 1, -1).normalized());
    OB::WorldConvexPolytope wcp1{"obj-2", cube, t};
    VClipWitness witness(Feature{9, FeatureType::edge}, Feature{2, FeatureType::vertex});
    //display(wcp0, wcp1, witness);
    VClip vclip(wcp0, wcp1);
    vclip.set_currentwitness(witness);
    VClipStateResult result = vclip.iterate().result;
    const VClipWitness& finalWitness = vclip.get_currentwitness();
    ASSERT_EQ(result, VClipStateResult::not_finished);
    ASSERT_EQ(finalWitness.feature1, Feature(9, FeatureType::edge));
    ASSERT_EQ(finalWitness.feature2, Feature(3, FeatureType::edge));
  }

  //5. v does not violate plane and e not simply excluded and derivatives mark H
  {
    OB::Transform t;
    t = OB::Transform::Identity();
    OB::WorldConvexPolytope wcp0{"obj-1", cube, t};
    t = OB::Translation(3.1, -2.2, 1) *
      OB::AngleAxis(0.25 * OB::PI, OB::Vector(-1, 1, -1).normalized());
    OB::WorldConvexPolytope wcp1{"obj-2", cube, t};
    VClipWitness witness(Feature{4, FeatureType::edge}, Feature{2, FeatureType::vertex});
    //display(wcp0, wcp1, witness);
    VClip vclip(wcp0, wcp1);
    vclip.set_currentwitness(witness);
    VClipStateResult result = vclip.iterate().result;
    const VClipWitness& finalWitness = vclip.get_currentwitness();
    ASSERT_EQ(result, VClipStateResult::not_finished);
    ASSERT_EQ(finalWitness.feature1, Feature(4, FeatureType::edge));
    ASSERT_EQ(finalWitness.feature2, Feature(3, FeatureType::edge));
  }


  //6. v does not violate plane and e not simply excluded and no update
  {
    OB::Transform t;
    t = OB::Transform::Identity();
    OB::WorldConvexPolytope wcp0{"obj-1", cube, t};
    t = OB::Translation(3.3, -1.2, -0.1)
      * OB::AngleAxis(-0.25* OB::PI, OB::Vector::UnitY().normalized())
      * OB::AngleAxis(-0.25* OB::PI, OB::Vector::UnitZ().normalized());
    OB::WorldConvexPolytope wcp1{"obj-2", cube, t};
    VClipWitness witness(Feature{9, FeatureType::edge}, Feature{3, FeatureType::vertex});
    //display(wcp0, wcp1, witness);
    VClip vclip(wcp0, wcp1);
    vclip.set_currentwitness(witness);
    VClipStateResult result = vclip.iterate().result;
    const VClipWitness& finalWitness = vclip.get_currentwitness();
    ASSERT_EQ(result, VClipStateResult::finished);
    ASSERT_EQ(finalWitness.feature1, Feature(9, FeatureType::edge));
    ASSERT_EQ(finalWitness.feature2, Feature(3, FeatureType::vertex));
  }
}

TEST_F(VClipTest, vertexfaceTest) {

  // think about this a bit.

  //1. v maximally violates a VP(F)
  {
    OB::Transform t;
    t = OB::Transform::Identity();
    OB::WorldConvexPolytope wcp0{"obj-1", prism6, t};
    t = OB::Translation(2, -3.5, 3);
    OB::WorldConvexPolytope wcp1{"obj-2", tetrahedron, t};
    VClipWitness witness(Feature{2, FeatureType::face}, Feature{2, FeatureType::vertex});
    //display(wcp0, wcp1, witness);
    VClip vclip(wcp0, wcp1);
    vclip.set_currentwitness(witness);
    VClipStateResult result = vclip.iterate().result;
    const VClipWitness& finalWitness = vclip.get_currentwitness();
    ASSERT_EQ(result, VClipStateResult::not_finished);
    ASSERT_EQ(finalWitness.feature1, Feature(12, FeatureType::edge));
    ASSERT_EQ(finalWitness.feature2, Feature(2, FeatureType::vertex));
  }

  // v does not violate f vp.
  //2. one vertex point to the face.
  {
    OB::Transform t;
    t = OB::Transform::Identity();
    OB::WorldConvexPolytope wcp0{"obj-1", prism6, t};
    t = OB::Translation(0, 3, 0);
    OB::WorldConvexPolytope wcp1{"obj-2", tetrahedron, t};
    VClipWitness witness(Feature{5, FeatureType::face}, Feature{2, FeatureType::vertex});
    //display(wcp0, wcp1, witness);
    VClip vclip(wcp0, wcp1);
    vclip.set_currentwitness(witness);
    VClipStateResult result = vclip.iterate().result;
    const VClipWitness& finalWitness = vclip.get_currentwitness();
    ASSERT_EQ(result, VClipStateResult::not_finished);
    ASSERT_EQ(finalWitness.feature1, Feature(5, FeatureType::face));
    ASSERT_EQ(finalWitness.feature2, Feature(8, FeatureType::edge));
  }

  //8. no vertex point to the face and vertex in positive side of face. check DONE
  {
    OB::Transform t;
    t = OB::Transform::Identity();
    OB::WorldConvexPolytope wcp0{"obj-1", prism6, t};
    t = OB::Translation(0, -3.5, 0);
    OB::WorldConvexPolytope wcp1{"obj-2", tetrahedron, t};
    VClipWitness witness(Feature{2, FeatureType::face}, Feature{2, FeatureType::vertex});
    //display(wcp0, wcp1, witness);
    VClip vclip(wcp0, wcp1);
    vclip.set_currentwitness(witness);
    VClipStateResult result = vclip.iterate().result;
    const VClipWitness& finalWitness = vclip.get_currentwitness();
    ASSERT_EQ(result, VClipStateResult::finished);
    ASSERT_EQ(finalWitness.feature1, Feature(2, FeatureType::face));
    ASSERT_EQ(finalWitness.feature2, Feature(2, FeatureType::vertex));
  }


  // handleLocalMin
  //9. one handleLocalMin with penetration. updates to edge
  {
    OB::Transform t;
    t = OB::Transform::Identity();
    OB::WorldConvexPolytope wcp0{"obj-1", prism6, t};
    t = OB::Translation(0, -2.5, 0);
    OB::WorldConvexPolytope wcp1{"obj-2", tetrahedron, t};
    VClipWitness witness(Feature{2, FeatureType::face}, Feature{2, FeatureType::vertex});
    //display(wcp0, wcp1, witness);
    VClip vclip(wcp0, wcp1);
    vclip.set_currentwitness(witness);
    VClipStateResult result = vclip.iterate().result;
    const VClipWitness& finalWitness = vclip.get_currentwitness();
    ASSERT_EQ(result, VClipStateResult::not_finished);
    ASSERT_EQ(finalWitness.feature1, Feature(2, FeatureType::face));
    ASSERT_EQ(finalWitness.feature2, Feature(8, FeatureType::edge));
  }

  //10. one handleLocalMin without penetration. check closest face updated.
  {
    OB::Transform t;
    t = OB::Transform::Identity();
    OB::WorldConvexPolytope wcp0{"obj-1", prism6, t};
    t = OB::Translation(0, 6.5, 0)
      * OB::AngleAxis(1 * OB::PI, OB::Vector::UnitZ().normalized());
    OB::WorldConvexPolytope wcp1{"obj-2", tetrahedron, t};
    VClipWitness witness(Feature{2, FeatureType::face}, Feature{2, FeatureType::vertex});
    //display(wcp0, wcp1, witness);
    VClip vclip(wcp0, wcp1);
    vclip.set_currentwitness(witness);
    VClipStateResult result = vclip.iterate().result;
    const VClipWitness& finalWitness = vclip.get_currentwitness();
    ASSERT_EQ(result, VClipStateResult::not_finished);
    ASSERT_EQ(finalWitness.feature1, Feature(5, FeatureType::face));
    ASSERT_EQ(finalWitness.feature2, Feature(2, FeatureType::vertex));
  }


}


TEST_F(VClipTest, edgeedgeTest) {
  // checks e1 -> e2

  // vertex - edge planes of VR(e1)
  //1. e2 simply excluded from e1
  {
    OB::Transform t;
    t = OB::Transform::Identity();
    OB::WorldConvexPolytope wcp0{"obj-1", cube, t};
    t = OB::Translation(6, 6.5, 0);
    OB::WorldConvexPolytope wcp1{"obj-2", pyramid5, t};
    VClipWitness witness(Feature{2, FeatureType::edge}, Feature{2, FeatureType::edge});
    //display(wcp0, wcp1, witness);
    VClip vclip(wcp0, wcp1);
    vclip.set_currentwitness(witness);
    VClipStateResult result = vclip.iterate().result;
    const VClipWitness& finalWitness = vclip.get_currentwitness();
    ASSERT_EQ(result, VClipStateResult::not_finished);
    ASSERT_EQ(finalWitness.feature1, Feature(1, FeatureType::vertex));
    ASSERT_EQ(finalWitness.feature2, Feature(2, FeatureType::edge));
  }

  //2. derivative check update to T (only T clips)
  {
    OB::Transform t;
    t = OB::AngleAxis(0.05* OB::PI,
                      OB::Vector::UnitZ().normalized());;OB::Transform::Identity();
    OB::WorldConvexPolytope wcp0{"obj-1", cube, t};
    t = OB::Translation(1.5, 6.5, 0);
    OB::WorldConvexPolytope wcp1{"obj-2", pyramid5, t};
    VClipWitness witness(Feature{0, FeatureType::edge}, Feature{2, FeatureType::edge});
    //display(wcp0, wcp1, witness);
    VClip vclip(wcp0, wcp1);
    vclip.set_currentwitness(witness);
    VClipStateResult result = vclip.iterate().result;
    const VClipWitness& finalWitness = vclip.get_currentwitness();
    ASSERT_EQ(result, VClipStateResult::not_finished);
    ASSERT_EQ(finalWitness.feature1, Feature(5, FeatureType::vertex));
    ASSERT_EQ(finalWitness.feature2, Feature(2, FeatureType::edge));
  }
  //3. derivative check update to H (both T and H clip)
  {
    OB::Transform t;
    t = OB::AngleAxis(-0.05* OB::PI,
                      OB::Vector::UnitZ().normalized());;OB::Transform::Identity();
    OB::WorldConvexPolytope wcp0{"obj-1", cube, t};
    t = OB::Translation(1.5, 6.5, 0);
    OB::WorldConvexPolytope wcp1{"obj-2", pyramid5, t};
    VClipWitness witness(Feature{0, FeatureType::edge}, Feature{2, FeatureType::edge});
    //display(wcp0, wcp1, witness);
    VClip vclip(wcp0, wcp1);
    vclip.set_currentwitness(witness);
    VClipStateResult result = vclip.iterate().result;
    const VClipWitness& finalWitness = vclip.get_currentwitness();
    ASSERT_EQ(result, VClipStateResult::not_finished);
    ASSERT_EQ(finalWitness.feature1, Feature(2, FeatureType::vertex));
    ASSERT_EQ(finalWitness.feature2, Feature(2, FeatureType::edge));
  }

  // face - edge planes of VR(e1)
  //5. simply excluded.
  {
    OB::Transform t;
    t = OB::AngleAxis(-0.05* OB::PI,
                      OB::Vector::UnitZ().normalized());;OB::Transform::Identity();
    OB::WorldConvexPolytope wcp0{"obj-1", cube, t};
    t = OB::Translation(2, 6.5, 0.5);
    OB::WorldConvexPolytope wcp1{"obj-2", pyramid5, t};
    VClipWitness witness(Feature{0, FeatureType::edge}, Feature{2, FeatureType::edge});
    //display(wcp0, wcp1, witness);
    VClip vclip(wcp0, wcp1);
    vclip.set_currentwitness(witness);
    VClipStateResult result = vclip.iterate().result;
    const VClipWitness& finalWitness = vclip.get_currentwitness();
    ASSERT_EQ(result, VClipStateResult::not_finished);
    ASSERT_EQ(finalWitness.feature1, Feature(4, FeatureType::face));
    ASSERT_EQ(finalWitness.feature2, Feature(2, FeatureType::edge));
  }


  //6. derivative check update to T (only T clips)
  {
    OB::Transform t;
    t = OB::AngleAxis(-0.2* OB::PI, OB::Vector::UnitY().normalized()) *
      OB::AngleAxis(-0.02* OB::PI, OB::Vector::UnitZ().normalized());
    OB::WorldConvexPolytope wcp0{"obj-1", cube, t};
    t = OB::Translation(2, 6.5, 0.5);
    OB::WorldConvexPolytope wcp1{"obj-2", pyramid5, t};
    VClipWitness witness(Feature{0, FeatureType::edge}, Feature{5, FeatureType::edge});
    //display(wcp0, wcp1, witness);
    VClip vclip(wcp0, wcp1);
    vclip.set_currentwitness(witness);
    VClipStateResult result = vclip.iterate().result;
    const VClipWitness& finalWitness = vclip.get_currentwitness();
    ASSERT_EQ(result, VClipStateResult::not_finished);
    ASSERT_EQ(finalWitness.feature1, Feature(4, FeatureType::face));
    ASSERT_EQ(finalWitness.feature2, Feature(5, FeatureType::edge));
  }

  //7. derivative check update to H (only H clips)
  {
    OB::Transform t;
    t = OB::AngleAxis(-0.2* OB::PI, OB::Vector::UnitY().normalized()) *
      OB::AngleAxis(-0.02* OB::PI, OB::Vector::UnitZ().normalized());
    OB::WorldConvexPolytope wcp0{"obj-1", cube, t};
    t = OB::Translation(2, 6.5, 0.5);
    OB::WorldConvexPolytope wcp1{"obj-2", pyramid5, t};
    VClipWitness witness(Feature{0, FeatureType::edge}, Feature{2, FeatureType::edge});
    //display(wcp0, wcp1, witness);
    VClip vclip(wcp0, wcp1);
    vclip.set_currentwitness(witness);
    VClipStateResult result = vclip.iterate().result;
    const VClipWitness& finalWitness = vclip.get_currentwitness();
    ASSERT_EQ(result, VClipStateResult::not_finished);
    ASSERT_EQ(finalWitness.feature1, Feature(4, FeatureType::face));
    ASSERT_EQ(finalWitness.feature2, Feature(2, FeatureType::edge));
  }


  //8. derivative check update to H (both T and H clip)
  // TODO THINK IF THIS IS POSSIBLE?? MAYBE IF THE EDGE IS UNDER THE OTHER EDGE?


  // checks e2 -> e1. no check e1->e2 passed.
  //9. ? something in VR vertex-edge
  {
    OB::Transform t;
    t = OB::Transform::Identity();
    t = OB::AngleAxis(-0.02* OB::PI, OB::Vector::UnitZ().normalized());
    OB::WorldConvexPolytope wcp0{"obj-1", cube, t};
    t = OB::Translation(1.4, 3.5, -1.5)
      * OB::AngleAxis(-0.4* OB::PI, OB::Vector::UnitX().normalized());
    OB::WorldConvexPolytope wcp1{"obj-2", pyramid5, t};
    VClipWitness witness(Feature{0, FeatureType::edge}, Feature{2, FeatureType::edge});
    //display(wcp0, wcp1, witness);
    VClip vclip(wcp0, wcp1);
    vclip.set_currentwitness(witness);
    VClipStateResult result = vclip.iterate().result;
    const VClipWitness& finalWitness = vclip.get_currentwitness();
    ASSERT_EQ(result, VClipStateResult::not_finished);
    ASSERT_EQ(finalWitness.feature1, Feature(0, FeatureType::edge));
    ASSERT_EQ(finalWitness.feature2, Feature(0, FeatureType::vertex));
  }


  // check done
  //11. check no test passes, check DONE.
  {
    OB::Transform t;
    t = OB::Transform::Identity();
    //t = OB::AngleAxis(-0.02* OB::PI, OB::Vector::UnitZ().normalized());
    OB::WorldConvexPolytope wcp0{"obj-1", cube, t};
    t = OB::Translation(2.0, 3, 0.5)
      * OB::AngleAxis(-0.25 * OB::PI, OB::Vector::UnitX().normalized())
      * OB::AngleAxis(-0.1 * OB::PI, OB::Vector::UnitZ().normalized());
    OB::WorldConvexPolytope wcp1{"obj-2", cube, t};
    VClipWitness witness(Feature{18, FeatureType::edge}, Feature{2, FeatureType::edge});
    //display(wcp0, wcp1, witness);
    VClip vclip(wcp0, wcp1);
    vclip.set_currentwitness(witness);
    VClipStateResult result = vclip.iterate().result;
    const VClipWitness& finalWitness = vclip.get_currentwitness();
    ASSERT_EQ(result, VClipStateResult::finished);
    ASSERT_EQ(finalWitness.feature1, Feature(18, FeatureType::edge));
    ASSERT_EQ(finalWitness.feature2, Feature(2, FeatureType::edge));
  }
}


TEST_F(VClipTest, edgefaceTest) {

  // e excluded from VR(f)
  //1. closes is T edge
  {
    OB::Transform t;
    t = OB::Transform::Identity();
    t = OB::Translation(0, 0, -2)
      * OB::AngleAxis(-0.06* OB::PI, OB::Vector::UnitZ().normalized());;
    OB::WorldConvexPolytope wcp0{"obj-1", cube, t};
    t = OB::Translation(0.1, -5, 0);
    OB::WorldConvexPolytope wcp1{"obj-2", prism20, t};
    VClipWitness witness(Feature{7, FeatureType::edge}, Feature{19, FeatureType::face});
    //display(wcp0, wcp1, witness);
    VClip vclip(wcp0, wcp1);
    vclip.set_currentwitness(witness);
    VClipStateResult result = vclip.iterate().result;
    const VClipWitness& finalWitness = vclip.get_currentwitness();
    ASSERT_EQ(result, VClipStateResult::not_finished);
    ASSERT_EQ(finalWitness.feature1, Feature(7, FeatureType::edge));
    ASSERT_EQ(finalWitness.feature2, Feature(95, FeatureType::edge));
  }


  //2. closes is T vertex
  {
    OB::Transform t;
    t = OB::Transform::Identity();
    t = OB::Translation(0, 0, -2)
      * OB::AngleAxis(-0.06* OB::PI, OB::Vector::UnitZ().normalized());;
    OB::WorldConvexPolytope wcp0{"obj-1", cube, t};
    t = OB::Translation(0, -5, 0);
    OB::WorldConvexPolytope wcp1{"obj-2", prism20, t};
    VClipWitness witness(Feature{7, FeatureType::edge}, Feature{19, FeatureType::face});
    //display(wcp0, wcp1, witness);
    VClip vclip(wcp0, wcp1);
    vclip.set_currentwitness(witness);
    VClipStateResult result = vclip.iterate().result;
    const VClipWitness& finalWitness = vclip.get_currentwitness();
    ASSERT_EQ(result, VClipStateResult::not_finished);
    ASSERT_EQ(finalWitness.feature1, Feature(7, FeatureType::edge));
    ASSERT_EQ(finalWitness.feature2, Feature(36, FeatureType::vertex));
  }

  //3. closes is intermediate edge
  // TODO CAN THIS BE POSSIBLE WITH T AND H??

  //4. closes is intermediate vertex
  {
    OB::Transform t;
    t = OB::Transform::Identity();
    t = OB::Translation(0, 0, -2);
    //  * OB::AngleAxis(-0.06* OB::PI, OB::Vector::UnitZ().normalized());;
    OB::WorldConvexPolytope wcp0{"obj-1", cube, t};
    t = OB::Translation(0, -5, 0);
    OB::WorldConvexPolytope wcp1{"obj-2", prism20, t};
    VClipWitness witness(Feature{7, FeatureType::edge}, Feature{19, FeatureType::face});
    //display(wcp0, wcp1, witness);
    VClip vclip(wcp0, wcp1);
    vclip.set_currentwitness(witness);
    VClipStateResult result = vclip.iterate().result;
    const VClipWitness& finalWitness = vclip.get_currentwitness();
    ASSERT_EQ(result, VClipStateResult::not_finished);
    ASSERT_EQ(finalWitness.feature1, Feature(7, FeatureType::edge));
    ASSERT_EQ(finalWitness.feature2, Feature(4, FeatureType::vertex));
  }


  //5. closes is H edge
  {
    OB::Transform t;
    t = OB::Transform::Identity();
    t = OB::Translation(0, 0, -3)
      * OB::AngleAxis(0.4* OB::PI, OB::Vector::UnitY().normalized());;
    OB::WorldConvexPolytope wcp0{"obj-1", cube, t};
    t = OB::Translation(0, -5, 0);
    OB::WorldConvexPolytope wcp1{"obj-2", prism20, t};
    VClipWitness witness(Feature{7, FeatureType::edge}, Feature{19, FeatureType::face});
    //display(wcp0, wcp1, witness);
    VClip vclip(wcp0, wcp1);
    vclip.set_currentwitness(witness);
    VClipStateResult result = vclip.iterate().result;
    const VClipWitness& finalWitness = vclip.get_currentwitness();
    ASSERT_EQ(result, VClipStateResult::not_finished);
    ASSERT_EQ(finalWitness.feature1, Feature(7, FeatureType::edge));
    ASSERT_EQ(finalWitness.feature2, Feature(97, FeatureType::edge));
  }


  //6. closes is H vertex
  {
    OB::Transform t;
    t = OB::Transform::Identity();
    t = OB::Translation(0.5, 0, -3)
      * OB::AngleAxis(0.4* OB::PI, OB::Vector::UnitY().normalized());;
    OB::WorldConvexPolytope wcp0{"obj-1", cube, t};
    t = OB::Translation(0, -5, 0);
    OB::WorldConvexPolytope wcp1{"obj-2", prism20, t};
    VClipWitness witness(Feature{7, FeatureType::edge}, Feature{19, FeatureType::face});
    //display(wcp0, wcp1, witness);
    VClip vclip(wcp0, wcp1);
    vclip.set_currentwitness(witness);
    VClipStateResult result = vclip.iterate().result;
    const VClipWitness& finalWitness = vclip.get_currentwitness();
    ASSERT_EQ(result, VClipStateResult::not_finished);
    ASSERT_EQ(finalWitness.feature1, Feature(7, FeatureType::edge));
    ASSERT_EQ(finalWitness.feature2, Feature(34, FeatureType::vertex));
  }


  // TODO  CAREFUL, WHAT IF THE EDGE (EXCLUDED) IS BELOW THE FACE PLANE?
  // TODO  CAREFUL, WHAT IF THE EDGE (EXCLUDED) IS CROSSING THE FACE PLANE?

  //7. PENETRATION.
  {
    OB::Transform t;
    t = OB::Transform::Identity();
    t = OB::Translation(0.5, 0, -1)
      * OB::AngleAxis(0.4* OB::PI, OB::Vector::UnitY().normalized());;
    OB::WorldConvexPolytope wcp0{"obj-1", cube, t};
    t = OB::Translation(0, -1, 0);
    OB::WorldConvexPolytope wcp1{"obj-2", cube, t};
    VClipWitness witness(Feature{3, FeatureType::face}, Feature{11, FeatureType::edge});
    //display(wcp0, wcp1, witness);
    VClip vclip(wcp0, wcp1);
    vclip.set_currentwitness(witness);
    VClipStateResult result = vclip.iterate().result;
    const VClipWitness& finalWitness = vclip.get_currentwitness();
    ASSERT_EQ(result, VClipStateResult::penetration);
    ASSERT_EQ(finalWitness.feature1, Feature(3, FeatureType::face));
    ASSERT_EQ(finalWitness.feature2, Feature(11, FeatureType::edge));
  }

  // vertex pointing away from face
  //8. update to N

  //9. update to tail of e

  // vertex pointing to face (or parallel)
  //9. update to N
  {
    OB::Transform t;
    t = OB::Transform::Identity();
    t = OB::Translation(0.0, 0, -1)
      * OB::AngleAxis(0.25* OB::PI, OB::Vector::UnitY().normalized());;
    OB::WorldConvexPolytope wcp0{"obj-1", cube, t};
    t = OB::Translation(4, -1, -4);
    OB::WorldConvexPolytope wcp1{"obj-2", cube, t};
    VClipWitness witness(Feature{3, FeatureType::face}, Feature{11, FeatureType::edge});
    //display(wcp0, wcp1, witness);
    VClip vclip(wcp0, wcp1);
    vclip.set_currentwitness(witness);
    VClipStateResult result = vclip.iterate().result;
    const VClipWitness& finalWitness = vclip.get_currentwitness();
    ASSERT_EQ(result, VClipStateResult::not_finished);
    ASSERT_EQ(finalWitness.feature1, Feature(12, FeatureType::edge));
    ASSERT_EQ(finalWitness.feature2, Feature(11, FeatureType::edge));
  }

  //10. update to head of e
  {
    OB::Transform t;
    t = OB::Transform::Identity();
    t = OB::Translation(0.0, 0, -1)
      * OB::AngleAxis(0.25* OB::PI, OB::Vector::UnitY().normalized());;
    OB::WorldConvexPolytope wcp0{"obj-1", cube, t};
    t = OB::Translation(0, -1, -4);
    OB::WorldConvexPolytope wcp1{"obj-2", cube, t};
    VClipWitness witness(Feature{3, FeatureType::face}, Feature{11, FeatureType::edge});
    //display(wcp0, wcp1, witness);
    VClip vclip(wcp0, wcp1);
    vclip.set_currentwitness(witness);
    VClipStateResult result = vclip.iterate().result;
    const VClipWitness& finalWitness = vclip.get_currentwitness();
    ASSERT_EQ(result, VClipStateResult::not_finished);
    ASSERT_EQ(finalWitness.feature1, Feature(3, FeatureType::face));
    ASSERT_EQ(finalWitness.feature2, Feature(4, FeatureType::vertex));
  }

}
