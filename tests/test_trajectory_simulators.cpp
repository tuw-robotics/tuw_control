
#include "gtest/gtest.h"
#include <array>
#include <boost/concept_check.hpp>
#include <tuw_control/trajectory_simulator/trajectory_simulator_precalc.h>
#include <tuw_control/trajectory_simulator/trajectory_simulator_online.h>

#include <tuw_control/state/state_sim_template.hpp>
#include <tuw_control/param_func/param_func_spline/param_func_spline0_dist.h>
using namespace tuw;
using namespace std;

namespace
{
class StateSimTest : public StateSimTemplate<7, 3>
{
public:
  enum class StateVars
  {
    X,
    Y,
    THETA,
    V,
    W,
    T,
    S
  };

public:
  enum class StateNmVars
  {
    X,
    Y,
    THETA
  };

public:
  enum class StateCfVars
  {
    V,
    W,
    T,
    S
  };

public:
  enum class ParamFuncVars
  {
    V,
    W
  };

public:
  enum class ParamFuncLattices
  {
    T
  };

public:
  StateSimTest()
  {
  }

public:
  StateSimUPtr cloneStateSim() const override
  {
    return StateSimUPtr(new StateSimTest(*this)); /*::make_unique< StateSimTest >(*this);*/
  }

public:
  double stateArc() const override
  {
    return stateCf_.value(asInt(StateCfVars::T));
  }

public:
  double stateDist() const override
  {
    return stateCf_.value(asInt(StateCfVars::S));
  }

public:
  ParamFuncs* paramFuncs() override
  {
    return &paramFuncs_;
  }

public:
  ParamFuncsDist* paramFuncsDist() override
  {
    return &paramFuncs_;
  }

private:
  void setStateCf(const double& _arc, const ParamFuncs::EvalArcGuarantee& _evalArcGuarantee =
                                          ParamFuncs::EvalArcGuarantee::AFTER_LAST) override
  {
    paramFuncs_.setEvalArc(_arc, _evalArcGuarantee);
    stateCf_.value(asInt(StateCfVars::T)) = _arc;
    stateCf_.value(asInt(StateCfVars::V)) = paramFuncs_.computeFuncVal(asInt(ParamFuncVars::V));
    stateCf_.value(asInt(StateCfVars::W)) = paramFuncs_.computeFuncVal(asInt(ParamFuncVars::W));
    stateCf_.value(asInt(StateCfVars::S)) = paramFuncs_.computeS();
  }

private:
  State& stateNmDot() override
  {
    stateNmDotCache_.value(asInt(StateNmVars::X)) =
        stateCf_.value(asInt(StateCfVars::V)) * cos(stateNm_.value(asInt(StateNmVars::THETA)));
    stateNmDotCache_.value(asInt(StateNmVars::Y)) =
        stateCf_.value(asInt(StateCfVars::V)) * sin(stateNm_.value(asInt(StateNmVars::THETA)));
    stateNmDotCache_.value(asInt(StateNmVars::THETA)) = stateCf_.value(asInt(StateCfVars::W));
    return stateNmDotCache_;
  }

private:
  State& stateNmDelta(const double& _dArc) override
  {
    const double& v = stateCf_.value(asInt(StateCfVars::V));
    const double& w = stateCf_.value(asInt(StateCfVars::W));
    const double& theta = stateNm_.value(asInt(StateNmVars::THETA));
    double& dx = stateNmDotCache_.value(asInt(StateNmVars::X));
    double& dy = stateNmDotCache_.value(asInt(StateNmVars::Y));
    double& da = stateNmDotCache_.value(asInt(StateNmVars::THETA));

    if (fabs(w) > FLT_MIN)
    {
      const double r = v / w;
      dx = (-r * sin(theta) + r * sin(theta + w * _dArc));
      dy = (+r * cos(theta) - r * cos(theta + w * _dArc));
      da = w * _dArc;
    }
    else
    {
      dx = v * _dArc * cos(theta);
      dy = v * _dArc * sin(theta);
      da = 0.0;
    }
    return stateNmDotCache_;
  }

private:
  ParamFuncsSpline0Dist paramFuncs_;
};

using SNV = StateSimTest::StateNmVars;
using SCF = StateSimTest::StateCfVars;
using PFV = StateSimTest::ParamFuncVars;

using PFS = ParamFuncs::ParamFuncsStructure;
using PfCpD = ParamFuncs::CtrlPtDim;
using PFFeM = ParamFuncs::FuncEvalMode;
using EaG = ParamFuncs::EvalArcGuarantee;

///@todo move it from here !!!!
// namespace cost_functions {
//
// template<typename MapData>
// class CFLatMap1Weight<TrajectorySimulator::LatticeVec, MapData> : public
// CFLatMap1WeightBase<TrajectorySimulator::LatticeVec, MapData> {
//     public   : CFLatMap1Weight() { this->arcAcc = [this](const size_t& _i){ return this->latticePtr_->at(_i).arc; };
//     }
// };
//
// }
//
// class DiffDriveCostV : public cost_functions::CFLatMap1Weight<TrajectorySimulator::LatticeVec, double> {
//     public  : DiffDriveCostV() {
// 	stAcc   = [this](const size_t& _i){ return latticePtr_->at(_i).statePtr->state(
// asInt(StateSimDiffDrive::StateVars::V) ); };
// 	cost_functions::funcPredef_FL1::lin       (this);
// 	cost_functions::funcPredef_FL2::intTrap   (this);
// 	cost_functions::funcPredef_FL3::weightNorm(this);
// 	weight_ = 1;
// 	cost0_  = 0;
//     }
//     public   : void reset () override {
// 	setBoundIdxBegin( 0 );
// 	double boundArc = FLT_MAX; if (latticePtr_->size() > 0 ){ boundArc = latticePtr_->back().arc; }
// 	setBoundArcEnd(boundArc);
// 	CFLatMap1Weight<TrajectorySimulator::LatticeVec, double>::reset();
//     }
//     public   : size_t latFuncLayerIdx() override { return 2; }
// };
//
//
// class DiffDriveCostConstrV : public cost_functions::CfSum<TrajectorySimulator::LatticeVec, double> {
//     public  : DiffDriveCostConstrV() {
// 	stAcc = [this](const size_t& _i){ return latticePtr_->at(_i).statePtr->state( asInt(StateSimDiffDrive::StateVars::S)
// ); };
// 	cost_functions::funcPredef_FL2::intTrap(this);
// 	cost_functions::funcPredef_FL3::weight (this);
// 	weight_ = 1;
// 	cost0_  = 0;
//     }
//     public  : size_t latFuncLayerIdx() override { return 2; }
// };
// class DiffDriveConstVOnKnots      : public cost_functions::CostsArrayOnLatKnotVec
// <TrajectorySimulator::LatticeVec, double, DiffDriveCostConstrV   > {
//     public   : DiffDriveConstVOnKnots()  { cost0_  = 0; }
//     public   : size_t latKnotLayerIdx() override { return 2; }
// };
//
// class DiffDriveCostConstrVMin : public cost_functions::CfSum<TrajectorySimulator::LatticeVec, double> {
//     public  : DiffDriveCostConstrVMin() {
// 	stAcc = [this](const size_t& _i){ return latticePtr_->at(_i).statePtr->state( asInt(StateSimDiffDrive::StateVars::S)
// ); };
// 	cost_functions::funcPredef_FL2::min       (this);
// 	cost_functions::funcPredef_FL3::weightNorm(this);
// 	weight_ = 1;
// 	cost0_  = 0;
//     }
//     public  : size_t latFuncLayerIdx() override { return 2; }
// };
// class DiffDriveConstVBetweenKnots : public
// cost_functions::CostsArrayBetweenLatKnotVec<TrajectorySimulator::LatticeVec, double, DiffDriveCostConstrVMin> {
//     public   : DiffDriveConstVBetweenKnots()  { cost0_  = FLT_MAX; }
//     public   : size_t latKnotLayerIdx() override { return 2; }
// };

// class DiffDriveCostsEvaluator : public CostsEvaluator<TrajectorySimulator::LatticeVec, double> {
//     public   : DiffDriveCostsEvaluator(std::shared_ptr<double>& _mapDataPtr) : CostsEvaluator(_mapDataPtr) {}
//     public   : void setCosts() override {
// 	partialCosts_     [asInt(CostEvaluatorCostType::F)].resize(1);
// 	partialCosts_     [asInt(CostEvaluatorCostType::F)][0] = make_unique<DiffDriveCostV>();
//
// 	partialCostsArray_[asInt(CostEvaluatorCostType::F)].resize(1);
// 	partialCostsArray_[asInt(CostEvaluatorCostType::F)][0] = make_unique<DiffDriveConstVOnKnots>();
//
//
// 	partialCosts_     [asInt(CostEvaluatorCostType::H)].resize(2);
// 	partialCosts_     [asInt(CostEvaluatorCostType::H)][0] = make_unique<DiffDriveCostV>();
// 	partialCosts_     [asInt(CostEvaluatorCostType::H)][1] = make_unique<DiffDriveCostV>();
//
// 	partialCostsArray_[asInt(CostEvaluatorCostType::H)].resize(2);
// 	partialCostsArray_[asInt(CostEvaluatorCostType::H)][0] = make_unique<DiffDriveConstVBetweenKnots>();
// 	partialCostsArray_[asInt(CostEvaluatorCostType::H)][1] = make_unique<DiffDriveConstVOnKnots     >();
//     }
// };
//
// class DiffDriveTrajectoryOptimizer : public TrajectoryOptimizer {
//
//     using SVR = StateSimDiffDrive::StateVars;
//     using PFV = StateSimDiffDrive::ParamFuncVars;
//     using PFL = StateSimDiffDrive::ParamFuncLattices;
//
//     public   : DiffDriveTrajectoryOptimizer ( StateSimPtr& _stateSim, std::unique_ptr<CostsEvaluatorClass>
//     _costsEvaluator ) : TrajectoryOptimizer(_stateSim, std::move(_costsEvaluator) ) {}
//     public   : void setVars() override {
// 	varsIdx_[asInt(AccessType::STATE_0      )] = {asInt(SVR::X), asInt(SVR::Y), asInt(SVR::THETA)};
// 	varsIdx_[asInt(AccessType::PARAM_CP     )] = {asInt(PFV::V), asInt(PFV::W)};
// 	varsIdx_[asInt(AccessType::PARAM_ARC    )] = {asInt(PFL::T)};
// 	varsIdx_[asInt(AccessType::PARAM_CP_END )] = {asInt(PFV::V), asInt(PFV::W)};
// 	varsIdx_[asInt(AccessType::PARAM_ARC_END)] = {asInt(PFL::T)};
//     }
// };
//

// The fixture for testing class Foo.
class TrajectorySimulatorsTest : public ::testing::Test
{
protected:
  // You can remove any or all of the following functions if its body
  // is empty.

  TrajectorySimulatorsTest()
  {
    // construct stateSim object and initalize the parametric functions object
    sp = std::make_shared<StateSimTest>();
    sp->setDiscrType(RungeKutta::DiscretizationType::EULER);

    vector<PFS> pf(2, PFS());
    static constexpr const int paramFuncArcRefIdxAll = 0;
    pf[asInt(PFV::V)].ctrlPtsSize = 3;
    pf[asInt(PFV::V)].ctrlPtsArcRefIdx = paramFuncArcRefIdxAll;
    pf[asInt(PFV::W)].ctrlPtsSize = 3;
    pf[asInt(PFV::W)].ctrlPtsArcRefIdx = paramFuncArcRefIdxAll;

    pf[asInt(PFV::V)].evalReq[asInt(PFFeM::INT1)] = false;
    pf[asInt(PFV::V)].evalReq[asInt(PFFeM::INT2)] = false;
    pf[asInt(PFV::W)].evalReq[asInt(PFFeM::INT1)] = false;
    pf[asInt(PFV::W)].evalReq[asInt(PFFeM::INT2)] = false;
    vector<size_t> idxClosedFormV(1, asInt(PFV::V));

    sp->paramFuncs()->init(pf);
    sp->paramFuncsDist()->setDistCfMode(ParamFuncsDist::TraveledDistCfMode::V, idxClosedFormV);

    // construct trajectory simulator object
    trajSim = std::make_shared<TrajectorySimulatorPrecalc>(sp);
    trajSimOnline = std::make_shared<TrajectorySimulatorOnline>(sp);
  }

  virtual ~TrajectorySimulatorsTest()
  {
  }

  // If the constructor and destructor are not enough for setting up
  // and cleaning up each test, you can define the following methods:

  virtual void SetUp()
  {
    Test::SetUp();
    // Code here will be called immediately after the constructor (right
    // before each test).
  }

  virtual void TearDown()
  {
    Test::TearDown();
    // Code here will be called immediately after each test (right
    // before the destructor).
  }
  StateSimPtr sp;
  TrajectorySimulatorSPtr trajSim;
  TrajectorySimulatorSPtr trajSimOnline;

public:
  bool checkSimLatticeConsistency(vector<TrajectorySimulator::LatticePointType>& _lattice)
  {
    bool consistent = true;
    //         for ( size_t i = 0; i < _lattice.size(); ++i ) {
    //             for ( size_t j = 0; j < _lattice[i].statePtr->stateSize(); ++j ) {
    //             cout<<_lattice[i].statePtr->value(j)<<", "; }cout<<endl;
    //         }cout<<endl;
    for (size_t i = 1; i < trajSim->simulationLattice_.size(); ++i)
    {
      if (!(trajSim->simulationLattice_[i - 1].arc <= trajSim->simulationLattice_[i].arc))
      {
        consistent = false;
        break;
      }
    }
    return consistent;
  }
  size_t sumAllPartStates(TrajectorySimulatorSPtr _tsp)
  {
    size_t nrPartStates = 0;
    for (size_t i = 0; i < _tsp->partLattices_.size(); ++i)
    {
      nrPartStates += _tsp->partLattices_[i]->size();
    }
    return nrPartStates;
  }
  void checkSimLatticeStatesEqual(vector<TrajectorySimulator::LatticePointType>& _latticeRef,
                                  vector<TrajectorySimulator::LatticePointType>& _latticeTest)
  {
    //         for ( size_t i = 0; i < _latticeTest.size(); ++i ) {
    //             for ( size_t j = 0; j < _latticeTest[i].statePtr->stateSize(); ++j ) {
    //             cout<<_latticeRef[i].statePtr->value(j)<<":"<<_latticeTest[i].statePtr->value(j)<<", "; }cout<<endl;
    //         }cout<<endl;
    ASSERT_EQ(_latticeRef.size(), _latticeTest.size());
    for (size_t i = 0; i < _latticeTest.size(); ++i)
    {
      for (size_t j = 0; j < _latticeTest[i].statePtr->valueSize(); ++j)
      {
        EXPECT_DOUBLE_EQ(_latticeRef[i].statePtr->value(j), _latticeTest[i].statePtr->value(j));
      }
    }
  }
  void checkPartLatticeStatesEqual(TrajectorySimulator::LatticeVecSPtrVec& _partLatRef,
                                   TrajectorySimulator::LatticeVecSPtrVec& _partLatTest)
  {
    ASSERT_EQ(_partLatRef.size(), _partLatTest.size());
    for (size_t i = 0; i < _partLatRef.size(); ++i)
    {
      ASSERT_EQ(_partLatRef[i]->size(), _partLatTest[i]->size());
      for (size_t j = 0; j < _partLatRef[i]->size(); ++j)
      {
        EXPECT_DOUBLE_EQ(_partLatRef[i]->at(j).arc, _partLatTest[i]->at(j).arc);
        for (size_t k = 0; k < _partLatRef[i]->at(j).statePtr->valueSize(); ++k)
        {
          EXPECT_DOUBLE_EQ(_partLatRef[i]->at(j).statePtr->value(k), _partLatTest[i]->at(j).statePtr->value(k));
        }
      }
    }
  }
  size_t expectedDtStates(const double _DeltaT, const double _dt)
  {
    return (ceil(_DeltaT / _dt) + 1);
  }
};

////////////////////////////////////////////--------------------------------------------------////////////////////////////////////////////

TEST_F(TrajectorySimulatorsTest, SimPrecalcFrom0Dt)
{
  ParamFuncs* funcs = trajSim->stateSim()->paramFuncs();
  // set the parametric functions control points values
  double initT = 0;
  size_t funcIdx = 0;
  funcs->ctrlPtVal(funcIdx, 0, PfCpD::ARC) = 0 + initT;
  funcs->ctrlPtVal(funcIdx, 1, PfCpD::ARC) = 0.5 + initT;
  funcs->ctrlPtVal(funcIdx, 2, PfCpD::ARC) = 1 + initT;

  funcIdx = 0;
  funcs->ctrlPtVal(funcIdx, 0, PfCpD::VAL) = 3.0 * M_PI;
  funcs->ctrlPtVal(funcIdx, 1, PfCpD::VAL) = -0.5 * M_PI;
  funcs->ctrlPtVal(funcIdx, 2, PfCpD::VAL) = 2.0 * M_PI;

  funcIdx = 1;
  funcs->ctrlPtVal(funcIdx, 0, PfCpD::VAL) = 3.5 * M_PI;
  funcs->ctrlPtVal(funcIdx, 1, PfCpD::VAL) = 1.5 * M_PI;
  funcs->ctrlPtVal(funcIdx, 2, PfCpD::VAL) = -4.5 * M_PI;

  funcs->precompute();

  // set lattice types
  trajSim->dsBase() = -1;  // only dt lattice types

  size_t statesBeginEnd = 2, statesDt, statesDs, statesLattice0;
  double sumStates = 0;
  size_t statesSimExp;

  trajSim->dtBase() = 0.1;
  trajSim->simulateTrajectory(0);
  statesDt = expectedDtStates(funcs->funcsArcEnd(), trajSim->dt());
  statesDs = 0;
  statesLattice0 = 0;
  sumStates = statesBeginEnd + statesDt + statesDs + statesLattice0;
  EXPECT_EQ(sumStates, sumAllPartStates(trajSim));
  statesSimExp = statesDt;
  EXPECT_EQ(statesSimExp, trajSim->simLattice().size());
  checkSimLatticeConsistency(trajSim->simulationLattice_);

  trajSim->dtBase() = 0.12;
  trajSim->simulateTrajectory(0);
  statesDt = expectedDtStates(funcs->funcsArcEnd(), trajSim->dt());
  statesDs = 0;
  statesLattice0 = 0;
  sumStates = statesBeginEnd + statesDt + statesDs + statesLattice0;
  EXPECT_EQ(sumStates, sumAllPartStates(trajSim));
  statesSimExp = statesDt;
  EXPECT_EQ(statesSimExp, trajSim->simLattice().size());
  checkSimLatticeConsistency(trajSim->simulationLattice_);

  trajSim->dtBase() = 0.25;
  trajSim->simulateTrajectory(0);
  statesDt = expectedDtStates(funcs->funcsArcEnd(), trajSim->dt());
  statesDs = 0;
  statesLattice0 = 0;
  sumStates = statesBeginEnd + statesDt + statesDs + statesLattice0;
  EXPECT_EQ(sumStates, sumAllPartStates(trajSim));
  statesSimExp = statesDt;
  EXPECT_EQ(statesSimExp, trajSim->simLattice().size());
  checkSimLatticeConsistency(trajSim->simulationLattice_);
}

TEST_F(TrajectorySimulatorsTest, SimPrecalcFrom0DtDsCtrlPt)
{
  ParamFuncs* funcs = trajSim->stateSim()->paramFuncs();
  // set the parametric functions control points values
  double initT = 0;
  size_t funcIdx = 0;
  funcs->ctrlPtVal(funcIdx, 0, PfCpD::ARC) = 0 + initT;
  funcs->ctrlPtVal(funcIdx, 1, PfCpD::ARC) = 0.557 + initT;
  funcs->ctrlPtVal(funcIdx, 2, PfCpD::ARC) = 1 + initT;

  funcIdx = 0;
  funcs->ctrlPtVal(funcIdx, 0, PfCpD::VAL) = 1.0;
  funcs->ctrlPtVal(funcIdx, 1, PfCpD::VAL) = 1.0;
  funcs->ctrlPtVal(funcIdx, 2, PfCpD::VAL) = 1.0;

  funcIdx = 1;
  funcs->ctrlPtVal(funcIdx, 0, PfCpD::VAL) = 3.5 * M_PI;
  funcs->ctrlPtVal(funcIdx, 1, PfCpD::VAL) = 1.5 * M_PI;
  funcs->ctrlPtVal(funcIdx, 2, PfCpD::VAL) = -4.5 * M_PI;

  funcs->precompute();

  // equal dt, equal arc dist and control ponints arcs lattices
  vector<vector<double*> > funcKnotsLattice(1, vector<double*>(funcs->funcsArcSize(0), 0));
  for (size_t i = 0; i < funcKnotsLattice[0].size(); ++i)
  {
    funcKnotsLattice[0][i] = &funcs->funcsArc(0, i);
  }
  trajSim->setUserDefLattice(funcKnotsLattice);
  size_t statesBeginEnd = 2, statesDt, statesDs, statesLattice0;
  size_t statesSimExp;
  double sumStates = 0;

  trajSim->dtBase() = 0.1;
  trajSim->dsBase() = 0.1;
  trajSim->simulateTrajectory(0);
  statesDt = expectedDtStates(funcs->funcsArcEnd(), trajSim->dt());
  statesDs = 10 + 1;
  statesLattice0 = funcs->funcCtrlPtSize(0);
  sumStates = statesBeginEnd + statesDt + statesDs + statesLattice0;
  EXPECT_EQ(sumStates, sumAllPartStates(trajSim));
  //     for(size_t i = 0; i < trajSim->simLattice().size(); i++){ cout<<trajSim->simLattice()[i].arc<<", ";
  //     }cout<<endl;
  statesSimExp = statesDt + /*statesDs - 2 +*/ statesLattice0 - 2;
  EXPECT_EQ(statesSimExp, trajSim->simLattice().size());  // ds in same locations as dt
  checkSimLatticeConsistency(trajSim->simulationLattice_);

  trajSim->dtBase() = 0.12;
  trajSim->dsBase() = 0.12;
  trajSim->simulateTrajectory(0);
  statesDt = expectedDtStates(funcs->funcsArcEnd(), trajSim->dt());
  statesDs = 9 + 1;
  statesLattice0 = funcs->funcCtrlPtSize(0);
  sumStates = statesBeginEnd + statesDt + statesDs + statesLattice0;
  EXPECT_EQ(sumStates, sumAllPartStates(trajSim));
  statesSimExp = statesDt + /*statesDs - 2 +*/ statesLattice0 - 2;
  EXPECT_EQ(statesSimExp, trajSim->simLattice().size());  // ds in same locations as dt
  checkSimLatticeConsistency(trajSim->simulationLattice_);

  trajSim->dtBase() = 0.25;
  trajSim->dsBase() = 0.01;
  trajSim->simulateTrajectory(0);
  statesDt = expectedDtStates(funcs->funcsArcEnd(), trajSim->dt());
  statesDs = 100 + 1;
  statesLattice0 = funcs->funcCtrlPtSize(0);
  sumStates = statesBeginEnd + statesDt + statesDs + statesLattice0;
  EXPECT_EQ(sumStates, sumAllPartStates(trajSim));
  statesSimExp = statesDt + statesDs - 2 - 3 + statesLattice0 - 2;
  EXPECT_EQ(statesSimExp, trajSim->simLattice().size());  // 3 ds overlay in same locations with dt (.25, .5, .75)
  checkSimLatticeConsistency(trajSim->simulationLattice_);
}

TEST_F(TrajectorySimulatorsTest, SimPrecalcFromNon0DtDsCtrlPt)
{
  ParamFuncs* funcs = trajSim->stateSim()->paramFuncs();
  // set the parametric functions control points values
  double initT = 0;
  size_t funcIdx = 0;
  funcs->ctrlPtVal(funcIdx, 0, PfCpD::ARC) = 0 + initT;
  funcs->ctrlPtVal(funcIdx, 1, PfCpD::ARC) = 0.557 + initT;
  funcs->ctrlPtVal(funcIdx, 2, PfCpD::ARC) = 1 + initT;

  funcIdx = 0;
  funcs->ctrlPtVal(funcIdx, 0, PfCpD::VAL) = 1.0;
  funcs->ctrlPtVal(funcIdx, 1, PfCpD::VAL) = 1.0;
  funcs->ctrlPtVal(funcIdx, 2, PfCpD::VAL) = 1.0;

  funcIdx = 1;
  funcs->ctrlPtVal(funcIdx, 0, PfCpD::VAL) = 3.5 * M_PI;
  funcs->ctrlPtVal(funcIdx, 1, PfCpD::VAL) = 1.5 * M_PI;
  funcs->ctrlPtVal(funcIdx, 2, PfCpD::VAL) = -4.5 * M_PI;

  funcs->precompute();

  // equal dt, equal arc dist and control ponints arcs lattices
  vector<vector<double*> > funcKnotsLattice(1, vector<double*>(funcs->funcsArcSize(0), 0));
  for (size_t i = 0; i < funcKnotsLattice[0].size(); ++i)
  {
    funcKnotsLattice[0][i] = &funcs->funcsArc(0, i);
  }
  trajSim->setUserDefLattice(funcKnotsLattice);

  trajSim->dtBase() = 0.13;
  trajSim->dsBase() = 0.0795;
  trajSim->simulateTrajectory(0);

  vector<TrajectorySimulator::LatticePointType> trajStatesRef;
  for (size_t i = 0; i < trajSim->simulationLattice_.size(); ++i)
  {
    StateSPtr stateNow = trajSim->simulationLattice_[i].statePtr->cloneState();
    trajStatesRef.emplace_back(TrajectorySimulator::LatticePointType(
        trajSim->simulationLattice_[i].arc, trajSim->simulationLattice_[i].latticeType, stateNow));
  }
  TrajectorySimulator::LatticeVecSPtrVec partLatRef;
  partLatRef.resize(trajSim->partLattices_.size());
  for (auto& latRefI : partLatRef)
  {
    latRefI = make_shared<TrajectorySimulator::LatticeVec>();
  }
  for (size_t i = 0; i < trajSim->partLattices_.size(); ++i)
  {
    partLatRef[i]->resize(trajSim->partLattices_[i]->size());
    for (size_t j = 0; j < trajSim->partLattices_[i]->size(); ++j)
    {
      partLatRef[i]->at(j).arc = trajSim->partLattices_[i]->at(j).arc;
      partLatRef[i]->at(j).statePtr = trajSim->partLattices_[i]->at(j).statePtr->cloneState();
    }
  }

  trajSim->simulateTrajectory(0.10);
  checkSimLatticeStatesEqual(trajStatesRef, trajSim->simulationLattice_);
  checkPartLatticeStatesEqual(partLatRef, trajSim->partLattices_);
  trajSim->simulateTrajectory(0.34);
  checkSimLatticeStatesEqual(trajStatesRef, trajSim->simulationLattice_);
  checkPartLatticeStatesEqual(partLatRef, trajSim->partLattices_);
  trajSim->simulateTrajectory(0.79);
  checkSimLatticeStatesEqual(trajStatesRef, trajSim->simulationLattice_);
  checkPartLatticeStatesEqual(partLatRef, trajSim->partLattices_);
  trajSim->simulateTrajectory(1.00);
  checkSimLatticeStatesEqual(trajStatesRef, trajSim->simulationLattice_);
  checkPartLatticeStatesEqual(partLatRef, trajSim->partLattices_);
  trajSim->simulateTrajectory(0.54);
  checkSimLatticeStatesEqual(trajStatesRef, trajSim->simulationLattice_);
  checkPartLatticeStatesEqual(partLatRef, trajSim->partLattices_);
  trajSim->simulateTrajectory(1.50);
  checkSimLatticeStatesEqual(trajStatesRef, trajSim->simulationLattice_);
  checkPartLatticeStatesEqual(partLatRef, trajSim->partLattices_);
}

////////////////////////////////////////////--------------------------------------------------////////////////////////////////////////////

TEST_F(TrajectorySimulatorsTest, SimOnlineFrom0Dt)
{
  ParamFuncs* funcs = trajSim->stateSim()->paramFuncs();
  // set the parametric functions control points values
  double initT = 0;
  size_t funcIdx = 0;
  funcs->ctrlPtVal(funcIdx, 0, PfCpD::ARC) = 0 + initT;
  funcs->ctrlPtVal(funcIdx, 1, PfCpD::ARC) = 0.5 + initT;
  funcs->ctrlPtVal(funcIdx, 2, PfCpD::ARC) = 1 + initT;

  funcIdx = 0;
  funcs->ctrlPtVal(funcIdx, 0, PfCpD::VAL) = 3.0 * M_PI;
  funcs->ctrlPtVal(funcIdx, 1, PfCpD::VAL) = -0.5 * M_PI;
  funcs->ctrlPtVal(funcIdx, 2, PfCpD::VAL) = 2.0 * M_PI;

  funcIdx = 1;
  funcs->ctrlPtVal(funcIdx, 0, PfCpD::VAL) = 3.5 * M_PI;
  funcs->ctrlPtVal(funcIdx, 1, PfCpD::VAL) = 1.5 * M_PI;
  funcs->ctrlPtVal(funcIdx, 2, PfCpD::VAL) = -4.5 * M_PI;

  funcs->precompute();

  // set lattice types
  trajSim->dsBase() = -1;  // only dt lattice types
  trajSimOnline->dsBase() = -1;

  trajSim->dtBase() = 0.10;
  trajSim->simulateTrajectory(0);
  trajSimOnline->dtBase() = 0.10;
  trajSimOnline->simulateTrajectory(0);
  checkSimLatticeStatesEqual(trajSim->simulationLattice_, trajSimOnline->simulationLattice_);
  checkPartLatticeStatesEqual(trajSim->partLattices_, trajSimOnline->partLattices_);

  trajSim->dtBase() = 0.12;
  trajSim->simulateTrajectory(0);
  trajSimOnline->dtBase() = 0.12;
  trajSimOnline->simulateTrajectory(0);
  checkSimLatticeStatesEqual(trajSim->simulationLattice_, trajSimOnline->simulationLattice_);
  checkPartLatticeStatesEqual(trajSim->partLattices_, trajSimOnline->partLattices_);

  trajSim->dtBase() = 0.25;
  trajSim->simulateTrajectory(0);
  trajSimOnline->dtBase() = 0.25;
  trajSimOnline->simulateTrajectory(0);
  checkSimLatticeStatesEqual(trajSim->simulationLattice_, trajSimOnline->simulationLattice_);
  checkPartLatticeStatesEqual(trajSim->partLattices_, trajSimOnline->partLattices_);
}

TEST_F(TrajectorySimulatorsTest, SimOnlineFrom0DtDsCtrlPt)
{
  ParamFuncs* funcs = trajSim->stateSim()->paramFuncs();
  // set the parametric functions control points values
  double initT = 0;
  size_t funcIdx = 0;
  funcs->ctrlPtVal(funcIdx, 0, PfCpD::ARC) = 0 + initT;
  funcs->ctrlPtVal(funcIdx, 1, PfCpD::ARC) = 0.557 + initT;
  funcs->ctrlPtVal(funcIdx, 2, PfCpD::ARC) = 1 + initT;

  funcIdx = 0;
  funcs->ctrlPtVal(funcIdx, 0, PfCpD::VAL) = 1.0;
  funcs->ctrlPtVal(funcIdx, 1, PfCpD::VAL) = 1.0;
  funcs->ctrlPtVal(funcIdx, 2, PfCpD::VAL) = 1.0;

  funcIdx = 1;
  funcs->ctrlPtVal(funcIdx, 0, PfCpD::VAL) = 3.5 * M_PI;
  funcs->ctrlPtVal(funcIdx, 1, PfCpD::VAL) = 1.5 * M_PI;
  funcs->ctrlPtVal(funcIdx, 2, PfCpD::VAL) = -4.5 * M_PI;

  funcs->precompute();

  vector<vector<double*> > funcKnotsLattice(1, vector<double*>(funcs->funcsArcSize(0), 0));
  for (size_t i = 0; i < funcKnotsLattice[0].size(); ++i)
  {
    funcKnotsLattice[0][i] = &funcs->funcsArc(0, i);
  }
  trajSim->setUserDefLattice(funcKnotsLattice);
  trajSimOnline->setUserDefLattice(funcKnotsLattice);

  trajSim->dtBase() = 0.10;
  trajSim->dsBase() = 0.10;
  trajSim->simulateTrajectory(0);
  trajSimOnline->dtBase() = 0.10;
  trajSimOnline->dsBase() = 0.10;
  trajSimOnline->simulateTrajectory(0);
  checkSimLatticeStatesEqual(trajSim->simulationLattice_, trajSimOnline->simulationLattice_);
  checkPartLatticeStatesEqual(trajSim->partLattices_, trajSimOnline->partLattices_);

  trajSim->dtBase() = 0.12;
  trajSim->dsBase() = 0.12;
  trajSim->simulateTrajectory(0);
  trajSimOnline->dtBase() = 0.12;
  trajSimOnline->dsBase() = 0.12;
  trajSimOnline->simulateTrajectory(0);
  checkSimLatticeStatesEqual(trajSim->simulationLattice_, trajSimOnline->simulationLattice_);
  checkPartLatticeStatesEqual(trajSim->partLattices_, trajSimOnline->partLattices_);

  trajSim->dtBase() = 0.25;
  trajSim->dsBase() = 0.01;
  trajSim->simulateTrajectory(0);
  trajSimOnline->dtBase() = 0.25;
  trajSimOnline->dsBase() = 0.01;
  trajSimOnline->simulateTrajectory(0);
  checkSimLatticeStatesEqual(trajSim->simulationLattice_, trajSimOnline->simulationLattice_);
  checkPartLatticeStatesEqual(trajSim->partLattices_, trajSimOnline->partLattices_);
}

TEST_F(TrajectorySimulatorsTest, SimOnlineFromNon0DtDsCtrlPt)
{
  ParamFuncs* funcs = trajSim->stateSim()->paramFuncs();
  // set the parametric functions control points values
  double initT = 0;
  size_t funcIdx = 0;
  funcs->ctrlPtVal(funcIdx, 0, PfCpD::ARC) = 0 + initT;
  funcs->ctrlPtVal(funcIdx, 1, PfCpD::ARC) = 0.557 + initT;
  funcs->ctrlPtVal(funcIdx, 2, PfCpD::ARC) = 1 + initT;

  funcIdx = 0;
  funcs->ctrlPtVal(funcIdx, 0, PfCpD::VAL) = 0.45;
  funcs->ctrlPtVal(funcIdx, 1, PfCpD::VAL) = -1.0;
  funcs->ctrlPtVal(funcIdx, 2, PfCpD::VAL) = 2.4;

  funcIdx = 1;
  funcs->ctrlPtVal(funcIdx, 0, PfCpD::VAL) = 3.5 * M_PI;
  funcs->ctrlPtVal(funcIdx, 1, PfCpD::VAL) = 1.5 * M_PI;
  funcs->ctrlPtVal(funcIdx, 2, PfCpD::VAL) = -4.5 * M_PI;

  funcs->precompute();

  vector<vector<double*> > funcKnotsLattice(1, vector<double*>(funcs->funcsArcSize(0), 0));
  for (size_t i = 0; i < funcKnotsLattice[0].size(); ++i)
  {
    funcKnotsLattice[0][i] = &funcs->funcsArc(0, i);
  }
  trajSim->setUserDefLattice(funcKnotsLattice);
  trajSimOnline->setUserDefLattice(funcKnotsLattice);

  trajSim->dtBase() = 0.13;
  trajSim->dsBase() = 0.0795;
  trajSim->simulateTrajectory(0);
  trajSimOnline->dtBase() = 0.13;
  trajSimOnline->dsBase() = 0.0795;
  trajSimOnline->simulateTrajectory(0);

  trajSim->simulateTrajectory(0.10);
  trajSimOnline->simulateTrajectory(0.10);
  checkSimLatticeStatesEqual(trajSim->simulationLattice_, trajSimOnline->simulationLattice_);
  checkPartLatticeStatesEqual(trajSim->partLattices_, trajSimOnline->partLattices_);
  trajSim->simulateTrajectory(0.34);
  trajSimOnline->simulateTrajectory(0.34);
  checkSimLatticeStatesEqual(trajSim->simulationLattice_, trajSimOnline->simulationLattice_);
  checkPartLatticeStatesEqual(trajSim->partLattices_, trajSimOnline->partLattices_);
  trajSim->simulateTrajectory(0.79);
  trajSimOnline->simulateTrajectory(0.79);
  checkSimLatticeStatesEqual(trajSim->simulationLattice_, trajSimOnline->simulationLattice_);
  checkPartLatticeStatesEqual(trajSim->partLattices_, trajSimOnline->partLattices_);
  trajSim->simulateTrajectory(1.00);
  trajSimOnline->simulateTrajectory(1.00);
  checkSimLatticeStatesEqual(trajSim->simulationLattice_, trajSimOnline->simulationLattice_);
  checkPartLatticeStatesEqual(trajSim->partLattices_, trajSimOnline->partLattices_);
  trajSim->simulateTrajectory(0.52);
  trajSimOnline->simulateTrajectory(0.52);
  checkSimLatticeStatesEqual(trajSim->simulationLattice_, trajSimOnline->simulationLattice_);
  checkPartLatticeStatesEqual(trajSim->partLattices_, trajSimOnline->partLattices_);
  trajSim->simulateTrajectory(1.50);
  trajSimOnline->simulateTrajectory(1.50);
  checkSimLatticeStatesEqual(trajSim->simulationLattice_, trajSimOnline->simulationLattice_);
  checkPartLatticeStatesEqual(trajSim->partLattices_, trajSimOnline->partLattices_);
}

////////////////////////////////////////////--------------------------------------------------////////////////////////////////////////////

}  // namespace

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
