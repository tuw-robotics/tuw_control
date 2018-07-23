
#include <tuw_control/state_map/state_map.hpp>

// gtest
#include <gtest/gtest.h>

// Eigen
#include <eigen3/Eigen/Eigen>

#include <tuw_control/param_func_new/param_func_spline/param_func_spline0_dist.hpp>

using namespace std;
using namespace Eigen;

namespace tuw
{
TEST(TestStateMap, CopyAssignmentOperatorArray)
{
  using NestedArrayLvl0 = StateMapArray<double, double, 4>;
  using NestedArrayLvl1 = StateMapArray<double, NestedArrayLvl0, 6>;

  NestedArrayLvl1 nState0, nState1;

  for (int i = 0; i < nState0.data().size(); ++i)
  {
    nState0.data()(i) = i + 1;
  }
  nState1 = nState0;
  for (int i = 0; i < nState1.data().size(); ++i)
  {
    nState1.data()(i) = 0;
  }
  for (int i = 0; i < nState0.data().size(); ++i)
  {
    EXPECT_EQ(i + 1, nState0.data()(i));
  }

  for (int i = 0; i < nState0.data().size(); ++i)
  {
    nState0.data()(i) = i + 1;
  }
  nState1.sub(0) = nState0.sub(0);
  for (int i = 0; i < nState1.sub(0).data().size(); ++i)
  {
    nState1.sub(0).data()(i) = 0;
  }
  for (int i = 0; i < nState0.sub(0).data().size(); ++i)
  {
    EXPECT_EQ(i + 1, nState0.sub(0).data()(i));
  }

  try
  {
    nState1 = NestedArrayLvl1(nState0);
    FAIL() << "Expected std::runtime_error";
  }
  catch (std::runtime_error const &err)
  {
    EXPECT_EQ(std::string("Copy-constructor not allowed"), err.what());
  }
  catch (...)
  {
    FAIL() << "Expected std::runtime_error";
  }

  try
  {
    nState1.sub(0) = NestedArrayLvl0(nState0.sub(0));
    FAIL() << "Expected std::runtime_error";
  }
  catch (std::runtime_error const &err)
  {
    EXPECT_EQ(std::string("Copy-constructor not allowed"), err.what());
  }
  catch (...)
  {
    FAIL() << "Expected std::runtime_error";
  }

  EXPECT_EQ(24, nState1.data().size());
  for (int i = 0; i < nState1.data().size(); ++i)
  {
    EXPECT_EQ(0, nState1.data()(i));
  }
}
TEST(TestStateMap, CopyAssignmentOperatorVector)
{
  using NestedVectorLvl0 = StateMapVector<double, double>;
  using NestedVectorLvl1 = StateMapVector<double, NestedVectorLvl0>;

  NestedVectorLvl1 nState0, nState1;
  nState0.subResize(2);
  nState0.sub(0).subResize(2);
  for (int i = 0; i < nState0.data().size(); ++i)
  {
    nState0.data()(i) = i + 1;
  }

  nState1 = nState0;
  for (int i = 0; i < nState1.data().size(); ++i)
  {
    nState1.data()(i) = 0;
  }
  for (int i = 0; i < nState0.data().size(); ++i)
  {
    EXPECT_EQ(i + 1, nState0.data()(i));
  }

  nState1.subResize(5);

  nState1 = nState0;
  for (int i = 0; i < nState1.data().size(); ++i)
  {
    nState1.data()(i) = 0;
  }
  for (int i = 0; i < nState0.data().size(); ++i)
  {
    EXPECT_EQ(i + 1, nState0.data()(i));
  }

  nState1.sub(0).subResize(7);

  nState1 = nState0;
  for (int i = 0; i < nState1.data().size(); ++i)
  {
    nState1.data()(i) = 0;
  }
  for (int i = 0; i < nState0.data().size(); ++i)
  {
    EXPECT_EQ(i + 1, nState0.data()(i));
  }

  nState1 = nState0;
  for (int i = 0; i < nState1.data().size(); ++i)
  {
    nState1.data()(i) = 0;
  }
  for (int i = 0; i < nState0.data().size(); ++i)
  {
    EXPECT_EQ(i + 1, nState0.data()(i));
  }

  for (int i = 0; i < nState0.data().size(); ++i)
  {
    nState0.data()(i) = i + 1;
  }

  nState1.sub(0) = nState0.sub(0);
  for (int i = 0; i < nState1.data().size(); ++i)
  {
    nState1.data()(i) = 0;
  }
  for (int i = 0; i < nState0.data().size(); ++i)
  {
    EXPECT_EQ(i + 1, nState0.data()(i));
  }

  try
  {
    nState1 = NestedVectorLvl1(nState0);
    FAIL() << "Expected std::runtime_error";
  }
  catch (std::runtime_error const &err)
  {
    EXPECT_EQ(std::string("Copy-constructor not allowed"), err.what());
  }
  catch (...)
  {
    FAIL() << "Expected std::runtime_error";
  }

  try
  {
    nState1.sub(0) = NestedVectorLvl0(nState0.sub(0));
    FAIL() << "Expected std::runtime_error";
  }
  catch (std::runtime_error const &err)
  {
    EXPECT_EQ(std::string("Copy-constructor not allowed"), err.what());
  }
  catch (...)
  {
    FAIL() << "Expected std::runtime_error";
  }

  EXPECT_EQ(2, nState1.data().size());
  for (int i = 0; i < nState1.data().size(); ++i)
  {
    EXPECT_EQ(0, nState1.data()(i));
  }
}
TEST(TestStateMap, CopyAssignmentOperatorTuple)
{
  using NestedVectorLvl0 = StateMapVector<double, double>;
  using NestedArrayLvl0 = StateMapArray<double, double, 4>;
  using NestedTupleLvl1 = StateMapTuple<double, NestedArrayLvl0, NestedVectorLvl0>;
  using NestedTupleLvl2 = StateMapTuple<double, NestedTupleLvl1, NestedArrayLvl0>;

  NestedTupleLvl2 nState0, nState1;
  nState0.sub<0>().sub<1>().subResize(2);
  for (int i = 0; i < nState0.data().size(); ++i)
  {
    nState0.data()(i) = i + 1;
  }

  nState1 = nState0;
  for (int i = 0; i < nState1.data().size(); ++i)
  {
    nState1.data()(i) = 0;
  }
  for (int i = 0; i < nState0.data().size(); ++i)
  {
    EXPECT_EQ(i + 1, nState0.data()(i));
  }

  nState1.sub<0>().sub<1>().subResize(7);

  nState1 = nState0;
  for (int i = 0; i < nState1.data().size(); ++i)
  {
    nState1.data()(i) = 0;
  }
  for (int i = 0; i < nState0.data().size(); ++i)
  {
    EXPECT_EQ(i + 1, nState0.data()(i));
  }

  for (int i = 0; i < nState0.data().size(); ++i)
  {
    nState0.data()(i) = i + 1;
  }
  nState1.sub<0>() = nState0.sub<0>();
  for (int i = 0; i < nState1.sub<0>().data().size(); ++i)
  {
    nState1.sub<0>().data()(i) = 0;
  }
  for (int i = 0; i < nState0.sub<0>().data().size(); ++i)
  {
    EXPECT_EQ(i + 1, nState0.sub<0>().data()(i));
  }

  for (int i = 0; i < nState0.data().size(); ++i)
  {
    nState0.data()(i) = i + 1;
  }
  nState1.sub<0>().sub<0>() = nState0.sub<0>().sub<0>();
  for (int i = 0; i < nState1.sub<0>().sub<0>().data().size(); ++i)
  {
    nState1.sub<0>().sub<0>().data()(i) = 0;
  }
  for (int i = 0; i < nState0.sub<0>().sub<0>().data().size(); ++i)
  {
    EXPECT_EQ(i + 1, nState0.sub<0>().sub<0>().data()(i));
  }

  try
  {
    nState1 = NestedTupleLvl2(nState0);
    FAIL() << "Expected std::runtime_error";
  }
  catch (std::runtime_error const &err)
  {
    EXPECT_EQ(std::string("Copy-constructor not allowed"), err.what());
  }
  catch (...)
  {
    FAIL() << "Expected std::runtime_error";
  }

  try
  {
    nState1.sub<0>() = NestedTupleLvl1(nState0.sub<0>());
    FAIL() << "Expected std::runtime_error";
  }
  catch (std::runtime_error const &err)
  {
    EXPECT_EQ(std::string("Copy-constructor not allowed"), err.what());
  }
  catch (...)
  {
    FAIL() << "Expected std::runtime_error";
  }

  EXPECT_EQ(10, nState1.data().size());
  for (int i = 0; i < nState1.data().size(); ++i)
  {
    EXPECT_EQ(0, nState1.data()(i));
  }
}

class Bla : public StateMapArray<double, double, 3>
{
public:
  using StateMapArray::StateMapArray;
  void asd()
  {
  }
};

TEST(TestStateMap, ExtendedClassInStateMap)
{
  using NestedExtClass = StateMapVector<double, Bla>;
  NestedExtClass test;
  test.subResize(1);
  test.sub(0).asd();
}

TEST(TestStateMap, NestedArrayVectorOneLvl)
{
  constexpr const size_t sizeLvl0 = 4;
  using NestedArrayLvl0 = StateMapArray<double, double, sizeLvl0>;
  using NestedVectorLvl0 = StateMapVector<double, double>;

  NestedArrayLvl0 nArrLvl0;
  NestedVectorLvl0 nVecLvl0;
}

TEST(TestStateMap, NestedArraysCompileTimeSize)
{
  constexpr const size_t sizeLvl0 = 4;
  constexpr const size_t sizeLvl1 = 3;
  using NestedLvl0 = StateMapArray<double, double, sizeLvl0>;
  using NestedLvl1 = StateMapArray<double, NestedLvl0, sizeLvl1>;

  NestedLvl0 nArrLvl0;
  EXPECT_EQ(sizeLvl0, nArrLvl0.MapSize);
  NestedLvl1 nArrLvl1;
  EXPECT_EQ(sizeLvl1 * sizeLvl0, nArrLvl1.MapSize);
}

TEST(TestStateMap, NestedVectorsCompileTimeSize)
{
  using NestedLvl0 = StateMapVector<double, double>;
  using NestedLvl1 = StateMapVector<double, NestedLvl0>;

  NestedLvl0 nLvl0;
  NestedLvl1 nLvl1;
  EXPECT_EQ(Eigen::Dynamic, nLvl0.MapSize);
  EXPECT_EQ(Eigen::Dynamic, nLvl1.MapSize);
}

TEST(TestStateMap, NestedArraysVectorsCompileTimeSize)
{
  using NestedVector = StateMapVector<double, double>;
  using NestedArraySz4 = StateMapArray<double, double, 4>;
  using NestedArray6Vector = StateMapArray<double, NestedVector, 6>;
  using NestedVectorArray4 = StateMapVector<double, NestedArraySz4>;

  NestedArray6Vector nA6V;
  EXPECT_EQ(Eigen::Dynamic, nA6V.MapSize);
  EXPECT_EQ(Eigen::Dynamic, nA6V.sub(0).MapSize);
  NestedVectorArray4 nVA5;
  EXPECT_EQ(Eigen::Dynamic, nVA5.MapSize);
  nVA5.subResize(5);
  EXPECT_EQ(Eigen::Dynamic, nVA5.MapSize);
  EXPECT_EQ(4, nVA5.sub(1).MapSize);
}

TEST(TestStateMap, NestedTupleCompileTimeSize)
{
  using NestedVector = StateMapVector<double, double>;
  using NestedArraySz4 = StateMapArray<double, double, 4>;
  using NestedArraySz6ArraySz4 = StateMapArray<double, NestedArraySz4, 6>;
  using NestedArray6Vector = StateMapArray<double, NestedVector, 6>;
  using NestedArray6Array6Vector = StateMapArray<double, NestedArray6Vector, 6>;
  using NestedVectorArray4 = StateMapVector<double, NestedArraySz4>;
  using NestedTupleConst = StateMapTuple<double, NestedArraySz6ArraySz4, NestedArraySz4, NestedArraySz6ArraySz4>;
  using NestedTupleDynInLvl1 = StateMapTuple<double, NestedTupleConst, NestedArray6Vector, NestedVectorArray4>;
  using NestedTupleDynInLvl2 =
      StateMapTuple<double, NestedTupleConst, NestedArray6Array6Vector, NestedArray6Array6Vector>;

  NestedTupleConst nConst;
  EXPECT_EQ(24 + 4 + 24, nConst.MapSize);
  EXPECT_EQ(24, nConst.sub<0>().MapSize);
  EXPECT_EQ(4, nConst.sub<1>().MapSize);
  EXPECT_EQ(24, nConst.sub<2>().MapSize);
  EXPECT_EQ(4, nConst.sub<0>().sub(0).MapSize);
  EXPECT_EQ(4, nConst.sub<2>().sub(0).MapSize);

  NestedTupleDynInLvl1 nDynLvl1;
  nDynLvl1.sub<2>().subResize(2);
  EXPECT_EQ(Eigen::Dynamic, nDynLvl1.MapSize);
  EXPECT_EQ(24 + 4 + 24, nDynLvl1.sub<0>().MapSize);
  EXPECT_EQ(Eigen::Dynamic, nDynLvl1.sub<1>().MapSize);
  EXPECT_EQ(Eigen::Dynamic, nDynLvl1.sub<2>().MapSize);
  EXPECT_EQ(4, nDynLvl1.sub<2>().sub(0).MapSize);

  NestedArray6Array6Vector nDynArArVec;
  nDynArArVec.sub(0).sub(0).subResize(2);
  EXPECT_EQ(Eigen::Dynamic, nDynArArVec.MapSize);
  EXPECT_EQ(Eigen::Dynamic, nDynArArVec.sub(0).MapSize);
  EXPECT_EQ(Eigen::Dynamic, nDynArArVec.sub(0).sub(0).MapSize);

  NestedTupleDynInLvl2 nDynLvl2;
  nDynLvl2.sub<1>().sub(0).sub(0).subResize(2);
  EXPECT_EQ(Eigen::Dynamic, nDynLvl2.MapSize);
  EXPECT_EQ(24 + 4 + 24, nDynLvl2.sub<0>().MapSize);
  EXPECT_EQ(Eigen::Dynamic, nDynLvl2.sub<1>().MapSize);
  EXPECT_EQ(Eigen::Dynamic, nDynLvl2.sub<2>().MapSize);
  EXPECT_EQ(Eigen::Dynamic, nDynLvl2.sub<1>().sub(0).MapSize);
  EXPECT_EQ(Eigen::Dynamic, nDynLvl2.sub<1>().sub(0).sub(0).MapSize);
  EXPECT_EQ(Eigen::Dynamic, nDynLvl2.sub<2>().sub(0).MapSize);
  EXPECT_EQ(Eigen::Dynamic, nDynLvl2.sub<2>().sub(0).sub(0).MapSize);
}

TEST(TestStateMap, NestedStateResize)
{
  using NestedVector = StateMapVector<double, double>;
  using NestedArraySz4 = StateMapArray<double, double, 4>;
  using NestedArraySz6 = StateMapArray<double, double, 6>;
  using NestedVectorArray6 = StateMapVector<double, NestedArraySz6>;
  using NestedArray6Vector = StateMapArray<double, NestedVector, 6>;
  using NestedArray6Array6Vector = StateMapArray<double, NestedArray6Vector, 6>;
  using NestedVectorArray4 = StateMapVector<double, NestedArraySz4>;
  using NestedTupleLvl1 = StateMapTuple<double, NestedVector, NestedArraySz4, NestedVectorArray6>;
  using NestedTupleLvl2 = StateMapTuple<double, NestedTupleLvl1, NestedArray6Array6Vector, NestedVectorArray4>;

  NestedTupleLvl2 nState;
  EXPECT_EQ(4, nState.data().size());
  for (int i = 0; i < nState.data().size(); ++i)
  {
    nState.data()(i) = 1;
  }
  //     cout<<nState.data().transpose()<<endl;
  nState.sub<2>().subResize(2);
  EXPECT_EQ(4 + 2 * 4, nState.data().size());
  //     cout<<nState.data().transpose()<<endl;
  for (int i = 0; i < nState.sub<2>().sub(0).data().size(); ++i)
  {
    nState.sub<2>().sub(0).data()(i) = 2;
  }
  for (int i = 0; i < nState.sub<2>().sub(1).data().size(); ++i)
  {
    nState.sub<2>().sub(1).data()(i) = 3;
  }
  //     cout<<nState.data().transpose()<<endl;
  nState.sub<1>().sub(0).sub(0).subResize(1);
  for (int i = 0; i < nState.sub<1>().sub(0).sub(0).data().size(); ++i)
  {
    nState.sub<1>().sub(0).sub(0).data()(i) = 4;
  }
  EXPECT_EQ(4 + 2 * 4 + 1, nState.data().size());
  nState.sub<1>().sub(0).sub(1).subResize(2);
  for (int i = 0; i < nState.sub<1>().sub(0).sub(1).data().size(); ++i)
  {
    nState.sub<1>().sub(0).sub(1).data()(i) = 5;
  }
  EXPECT_EQ(4 + 2 * 4 + 1 + 2, nState.data().size());
  nState.sub<1>().sub(1).sub(0).subResize(3);
  for (int i = 0; i < nState.sub<1>().sub(1).sub(0).data().size(); ++i)
  {
    nState.sub<1>().sub(1).sub(0).data()(i) = 6;
  }
  EXPECT_EQ(4 + 2 * 4 + 1 + 2 + 3, nState.data().size());
  //     cout<<nState.data().transpose()<<endl;
  nState.sub<0>().sub<0>().subResize(3);
  for (int i = 0; i < nState.sub<0>().sub<0>().data().size(); ++i)
  {
    nState.sub<0>().sub<0>().data()(i) = 7;
  }
  EXPECT_EQ(4 + 2 * 4 + 1 + 2 + 3 + 3, nState.data().size());
  nState.sub<0>().sub<2>().subResize(2);
  for (int i = 0; i < nState.sub<0>().sub<2>().data().size(); ++i)
  {
    nState.sub<0>().sub<2>().data()(i) = 8;
  }
  EXPECT_EQ(4 + 2 * 4 + 1 + 2 + 3 + 3 + 2 * 6, nState.data().size());
  //     cout<<nState.data().transpose()<<endl;
  std::array<double, 4 + 2 * 4 + 1 + 2 + 3 + 3 + 2 * 6> numRes = { 7, 7, 7, 1, 1, 1, 1, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
                                                                   8, 8, 4, 5, 5, 6, 6, 6, 2, 2, 2, 2, 3, 3, 3, 3 };
  for (size_t i = 0; i < numRes.size(); ++i)
  {
    EXPECT_EQ(numRes[i], nState.data()(i));
  }

  nState.sub<0>().sub<0>().subResize(1);
  nState.sub<2>().subResize(1);
  std::array<double, 4 + 1 * 4 + 1 + 2 + 3 + 1 + 2 * 6> numRes2 = { 7, 1, 1, 1, 1, 8, 8, 8, 8, 8, 8, 8, 8, 8,
                                                                    8, 8, 8, 4, 5, 5, 6, 6, 6, 2, 2, 2, 2 };
  for (size_t i = 0; i < numRes2.size(); ++i)
  {
    EXPECT_EQ(numRes2[i], nState.data()(i));
  }

  nState.sub<0>().sub<0>().subResize(0);
  nState.sub<2>().subResize(0);
  std::array<double, 4 + 0 * 4 + 1 + 2 + 3 + 0 + 2 * 6> numRes3 = { 1, 1, 1, 1, 8, 8, 8, 8, 8, 8, 8,
                                                                    8, 8, 8, 8, 8, 4, 5, 5, 6, 6, 6 };
  for (size_t i = 0; i < numRes3.size(); ++i)
  {
    EXPECT_EQ(numRes3[i], nState.data()(i));
  }
}

TEST(TestStateMap, ResizeWithEmptyVectors)
{
  constexpr const size_t sizeLvl0 = 4;
  using NestedVectorBase = StateMapVector<double, double>;
  using NestedArray = StateMapArray<double, NestedVectorBase, sizeLvl0>;
  using NestedVector = StateMapVector<double, NestedArray>;

  NestedVector nState;
  NestedArray nArr;
  nArr.sub(0).subResize(2);
  nState.subResize(2);
  //     cout<<nState.data().transpose()<<endl;
  nState.sub(0).sub(0).subResize(1);
  //     cout<<nState.data().transpose()<<endl;
  for (int i = 0; i < nState.data().size(); ++i)
  {
    nState.data()(i) = 1;
  }
  //     cout<<nState.data().transpose()<<endl;
  nState.sub(0).sub(0).subResize(3);
  std::array<double, 3> numRes0 = { 1, 0, 0 };
  for (int i = 0; i < nState.data().size(); ++i)
  {
    EXPECT_EQ(numRes0[i], nState.data()(i));
  }

  //     cout<<nState.data().transpose()<<endl;
  nState.subResize(2);

  for (int i = 0; i < nState.data().size(); ++i)
  {
    EXPECT_EQ(numRes0[i], nState.data()(i));
  }
}

TEST(TestStateMap, NestedArraysDataBuffer)
{
  constexpr const size_t sizeLvl0 = 4;
  constexpr const size_t sizeLvl1 = 3;
  using NestedLvl0 = StateMapArray<double, double, sizeLvl0>;
  using NestedLvl1 = StateMapArray<double, NestedLvl0, sizeLvl1>;

  Eigen::Vector4d inc4(1., 2., 3., 4.);
  Eigen::Vector3d dec3(9., 8., 7.);
  NestedLvl0 nArrLvl0;
  NestedLvl1 nArrLvl1;
  nArrLvl1.data().block(0, 0, 3, 1) = dec3;
  nArrLvl1.data().block(3, 0, 3, 1) = dec3;
  nArrLvl1.data().block(6, 0, 3, 1) = dec3;
  nArrLvl1.data().block(9, 0, 3, 1) = dec3;
  for (int i = 0; i < nArrLvl1.data().size(); ++i)
  {
    EXPECT_EQ(9 - i % 3, nArrLvl1.data()(i));
  }
  nArrLvl1.data().block(7, 0, 4, 1) = inc4;
  for (int i = 7; i < 7 + 4; ++i)
  {
    EXPECT_EQ(i - 7 + 1, nArrLvl1.data()(i));
  }
  nArrLvl1.sub(0).data() = inc4;
  nArrLvl1.sub(2).data() = inc4;
  for (int i = 0; i < nArrLvl1.sub(0).data().rows(); ++i)
  {
    EXPECT_EQ(i + 1, nArrLvl1.data()(i));
  }
  for (int i = 2 * (int)sizeLvl0; i < 2 * (int)sizeLvl0 + nArrLvl1.sub(3).data().rows(); ++i)
  {
    EXPECT_EQ(i + 1 - 2 * sizeLvl0, nArrLvl1.data()(i));
  }
}

TEST(TestStateMap, VirtualBase)
{
  using NestedArray4 = StateMapArray<double, double, 6>;
  using NestedVectorArray6 = StateMapVector<double, NestedArray4>;
  using NestedArray4VectorArray6 = StateMapArray<double, NestedVectorArray6, 4>;

  std::shared_ptr<StateMapBaseVirt<double>> nStateBaseVirt = std::make_shared<NestedArray4VectorArray6>();

  nStateBaseVirt->sub(0).subResize(2);
  for (int i = 0; i < nStateBaseVirt->data().size(); ++i)
  {
    nStateBaseVirt->data()(i) = i;
  }
  //     cout<<nStateBaseVirt->data().transpose()<<endl;
  for (int i = 0; i < nStateBaseVirt->data().size(); ++i)
  {
    EXPECT_EQ(i, nStateBaseVirt->data()(i));
  }

  try
  {
    nStateBaseVirt->subResize(2);
    FAIL() << "Expected std::runtime_error";
  }
  catch (std::runtime_error const &err)
  {
    EXPECT_EQ(std::string("Cannot resize an array"), err.what());
  }
  catch (...)
  {
    FAIL() << "Expected std::runtime_error";
  }

  try
  {
    nStateBaseVirt->sub(0).sub(0).sub(0);
    FAIL() << "Expected std::runtime_error";
  }
  catch (std::runtime_error const &err)
  {
    EXPECT_EQ(std::string("Access of numeric Leaf using virtual \"sub\" function not allowed"), err.what());
  }
  catch (...)
  {
    FAIL() << "Expected std::runtime_error";
  }

  using NestedTupleArray4VectorArray6 = StateMapTuple<double, NestedArray4, NestedVectorArray6>;

  std::shared_ptr<StateMapBaseVirt<double>> nStateTupBaseVirt = std::make_shared<NestedTupleArray4VectorArray6>();

  nStateTupBaseVirt->sub(1).subResize(2);
  for (int i = 0; i < nStateTupBaseVirt->data().size(); ++i)
  {
    nStateTupBaseVirt->data()(i) = i;
  }
  try
  {
    nStateTupBaseVirt->subResize(2);
    FAIL() << "Expected std::runtime_error";
  }
  catch (std::runtime_error const &err)
  {
    EXPECT_EQ(std::string("Cannot resize a tuple"), err.what());
  }
  catch (...)
  {
    FAIL() << "Expected std::runtime_error";
  }

  std::shared_ptr<NestedTupleArray4VectorArray6> nStateTupExt =
      dynamic_pointer_cast<NestedTupleArray4VectorArray6>(nStateTupBaseVirt);
  std::shared_ptr<StateMapBaseCRTP<NestedTupleArray4VectorArray6>> nStateTupBaseCRTP = nStateTupExt;

  for (int i = 0; i < nStateTupBaseVirt->sub(1).data().size(); ++i)
  {
    EXPECT_EQ(i + 6, nStateTupBaseVirt->sub(1).data()(i));
  }
  for (int i = 0; i < nStateTupExt->sub<1>().data().size(); ++i)
  {
    EXPECT_EQ(i + 6, nStateTupExt->sub<1>().data()(i));
  }
  for (int i = 0; i < nStateTupBaseCRTP->sub<1>().data().size(); ++i)
  {
    EXPECT_EQ(i + 6, nStateTupBaseCRTP->sub<1>().data()(i));
  }
}

}  // namespace <anonymous>

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  srand((int)time(0));
  return RUN_ALL_TESTS();
}
