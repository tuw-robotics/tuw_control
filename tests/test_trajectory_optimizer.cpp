
#include "gtest/gtest.h"
#include <array>
#include <boost/concept_check.hpp>
#include <tuw_control/trajectory_simulator/trajectory_simulator_precalc.h>
#include <tuw_control/trajectory_simulator/trajectory_simulator_online.h>
#include <tuw_control/trajectory_evaluator/trajectory_optimizer.h>
#include <tuw_control/costs_evaluator/cost_functions.hpp>

#include <tuw_control/state/state_sim_template.hpp>
#include <tuw_control/param_func/param_func_spline/param_func_spline0_dist.h>

using namespace tuw;
using namespace std;

namespace tuw { namespace cost_functions {

template<typename MapData>
class CFLatMap1Weight<TrajectorySimulator::LatticeVec, MapData> : public CFLatMap1WeightBase<TrajectorySimulator::LatticeVec, MapData> {
    public   : CFLatMap1Weight() { this->arcAcc = [this](const size_t& _i){ return this->latticePtr_->at(_i).arc; }; }
};

} }

namespace {

class StateSimTest : public StateSimTemplate<7,3> {

    public   : enum class StateVars    { X, Y, THETA, V, W, T, S }; 
    public   : enum class StateNmVars  { X, Y, THETA }; 
    public   : enum class StateCfVars  { V, W, T, S   }; 
    public   : enum class ParamFuncVars{ V, W }; 
    public   : enum class ParamFuncLattices{ T };

    public   : StateSimTest                    () {}
    public   : StateSimUPtr     cloneStateSim  () const override { return StateSimUPtr(new StateSimTest(*this) );/*::make_unique< StateSimTest >(*this);*/ }
    public   : double           stateArc       () const override { return stateCf_.value(asInt(StateCfVars::T)); }
    public   : double           stateDist      () const override { return stateCf_.value(asInt(StateCfVars::S)); }
    public   : ParamFuncs*      paramFuncs     () override       { return &paramFuncs_; }
    public   : ParamFuncsDist*  paramFuncsDist () override       { return &paramFuncs_; }

    private  : void             setStateCf     ( const double& _arc, const ParamFuncs::EvalArcGuarantee& _evalArcGuarantee = ParamFuncs::EvalArcGuarantee::AFTER_LAST  ) override {
	paramFuncs_.setEvalArc (_arc, _evalArcGuarantee);
	stateCf_.value(asInt(StateCfVars::T)) = _arc;
	stateCf_.value(asInt(StateCfVars::V)) = paramFuncs_.computeFuncVal(asInt(ParamFuncVars::V));
	stateCf_.value(asInt(StateCfVars::W)) = paramFuncs_.computeFuncVal(asInt(ParamFuncVars::W)); 
	stateCf_.value(asInt(StateCfVars::S)) = paramFuncs_.computeS();
    }
    private  : State&           stateNmDot     ()                      override { 
	stateNmDotCache_.value(asInt(StateNmVars::X))     = stateCf_.value(asInt(StateCfVars::V)) * cos(stateNm_.value(asInt(StateNmVars::THETA)));
	stateNmDotCache_.value(asInt(StateNmVars::Y))     = stateCf_.value(asInt(StateCfVars::V)) * sin(stateNm_.value(asInt(StateNmVars::THETA)));
	stateNmDotCache_.value(asInt(StateNmVars::THETA)) = stateCf_.value(asInt(StateCfVars::W));
	return stateNmDotCache_; 
    }
    private  : State&           stateNmDelta   ( const double& _dArc ) override {
	const double& v     = stateCf_        .value(asInt(StateCfVars::V));
	const double& w     = stateCf_        .value(asInt(StateCfVars::W));
	const double& theta = stateNm_        .value(asInt(StateNmVars::THETA));
	double& dx          = stateNmDotCache_.value(asInt(StateNmVars::X));
	double& dy          = stateNmDotCache_.value(asInt(StateNmVars::Y));
	double& da          = stateNmDotCache_.value(asInt(StateNmVars::THETA));
	
	if ( fabs ( w ) > FLT_MIN ) {
	    const double r = v / w;
	    dx = ( - r * sin ( theta ) + r * sin ( theta + w * _dArc ) );
	    dy = ( + r * cos ( theta ) - r * cos ( theta + w * _dArc ) );
	    da = w * _dArc;
	} else {
	    dx = v * _dArc * cos ( theta );
	    dy = v * _dArc * sin ( theta );
	    da = 0.0;
	}
	return stateNmDotCache_;
    }
    private  : ParamFuncsSpline0Dist paramFuncs_;
};

using SNF = StateSimTest::StateVars;
using SNV = StateSimTest::StateNmVars;
using SCF = StateSimTest::StateCfVars;
using PFV = StateSimTest::ParamFuncVars;


// void coutCostEvalCosts(TrajectorySimGradeSPtr trajSimGrade) {
//     cout<<"Cost Functions:"<<endl;
//     cout<<"f = "<<trajSimGrade->trajSim()->costsEvaluator_->f<<endl<<"h = "; 
//     for(size_t i = 0; i < trajSimGrade->trajSim()->costsEvaluator_->h.size(); ++i) { cout<<trajSimGrade->trajSim()->costsEvaluator_->h[i]<<", "; } cout<<endl;
//     cout<<"gradF = ";
//     for(size_t i = 0; i < trajSimGrade->trajSim()->costsEvaluator_->gradF.size(); ++i) { cout<<trajSimGrade->trajSim()->costsEvaluator_->gradF[i]<<", "; } cout<<endl;
//     cout<<"gradH = "<<endl<<trajSimGrade->trajSim()->costsEvaluator_->gradH<<endl;
// }
    
// The fixture for testing class Foo.
class TrajOptTest : public ::testing::Test {
protected:
    // You can remove any or all of the following functions if its body
    // is empty.

    TrajOptTest() {
	
	dummyMapData = make_shared<double>(0);
	
	//initialize parametric functions structure
	using PFS = ParamFuncs::ParamFuncsStructure; using PfCpD = ParamFuncs::CtrlPtDim; using FeM = ParamFuncs::FuncEvalMode; //using EaG = ParamFuncs::EvalArcGuarantee;
	size_t funcIdx = 0; vector<PFS> pf( 2 , PFS() );
	static constexpr const int paramFuncArcRefIdxAll = 0;
	pf[0].ctrlPtsSize = 5; pf[0].ctrlPtsArcRefIdx = paramFuncArcRefIdxAll; 
	pf[1].ctrlPtsSize = 5; pf[1].ctrlPtsArcRefIdx = paramFuncArcRefIdxAll;
	pf[0].evalReq[asInt(FeM::INT1)] = false; 
	pf[0].evalReq[asInt(FeM::INT2)] = false;    
	
	//construct stateSim object and initalize the parametric functions object
	stateSimPtr = std::make_shared<StateSimTest>(); 
	stateSimPtr->paramFuncs    ()->init(pf); 
	stateSimPtr->paramFuncsDist()->setDistCfMode(ParamFuncsDist::TraveledDistCfMode::V, vector<size_t> (1,0) );
	
	//set the parametric functions control points
	ParamFuncs* funcs  = stateSimPtr->paramFuncs();
	double initT = 0;
	funcIdx = 0;
	funcs->ctrlPtVal(funcIdx, 0, PfCpD::ARC) = 0    +initT; 
	funcs->ctrlPtVal(funcIdx, 1, PfCpD::ARC) = 0.25 +initT;
	funcs->ctrlPtVal(funcIdx, 2, PfCpD::ARC) = 0.51 +initT;
	funcs->ctrlPtVal(funcIdx, 3, PfCpD::ARC) = 0.75 +initT;
	funcs->ctrlPtVal(funcIdx, 4, PfCpD::ARC) = 1    +initT;
	
	funcIdx = 0;
	funcs->ctrlPtVal(funcIdx, 0, PfCpD::VAL) = 1.;
	funcs->ctrlPtVal(funcIdx, 1, PfCpD::VAL) = 2.;
	funcs->ctrlPtVal(funcIdx, 2, PfCpD::VAL) = 3.;
	funcs->ctrlPtVal(funcIdx, 3, PfCpD::VAL) = 4.;
	funcs->ctrlPtVal(funcIdx, 4, PfCpD::VAL) = 5.;
	
	funcIdx = 1;
	funcs->ctrlPtVal(funcIdx, 0, PfCpD::VAL) =  3.5*M_PI; 
	funcs->ctrlPtVal(funcIdx, 1, PfCpD::VAL) =  1.5*M_PI; 
	funcs->ctrlPtVal(funcIdx, 2, PfCpD::VAL) = -4.5*M_PI; 
	funcs->ctrlPtVal(funcIdx, 3, PfCpD::VAL) = -3.5*M_PI; 
	funcs->ctrlPtVal(funcIdx, 4, PfCpD::VAL) = -2.0*M_PI; 
	funcs->precompute();
    }
    virtual ~TrajOptTest() { }
    // If the constructor and destructor are not enough for setting up
    // and cleaning up each test, you can define the following methods:
    virtual void SetUp() {
	Test::SetUp();
        // Code here will be called immediately after the constructor (right
        // before each test).
    }
    virtual void TearDown() {
	Test::TearDown();
        // Code here will be called immediately after each test (right
        // before the destructor).
    }
    StateSimPtr stateSimPtr;
    TrajectoryOptimizerSPtr trajOpt;
    shared_ptr<double> dummyMapData;
};

////////////////////////////////////////////--------------------------------------------------////////////////////////////////////////////


using       TS = TrajectorySimulator;
using   TSBSLT = TS::BaseSimLatticeType;
using TSLatVec = TS::LatticeVec;

using MapData       = double;
using namespace tuw::cost_functions;

////////********************************************************************************************************************************
class TestCost_LinSumW_V : public CFLatMap1Weight<TSLatVec, MapData> {
    public  : TestCost_LinSumW_V() { 
	stAcc   = [this](const size_t& _i){ return latticePtr_->at(_i).statePtr->value( asInt(SNF::V) ); }; 
	funcPredef_FL1::lin   (this); 
	funcPredef_FL2::sum   (this); 
	funcPredef_FL3::weight(this);
	weight_ = 1; cost0_  = 0;
    }
};

////////********************************************************************************************************************************
class TestCost_LinIntWNorm_W : public CFLatMap1Weight<TSLatVec, MapData> {
    public  : TestCost_LinIntWNorm_W() { 
	stAcc   = [this](const size_t& _i){ return latticePtr_->at(_i).statePtr->value( asInt(SNF::W) ); }; 
	funcPredef_FL1::lin       (this); 
	funcPredef_FL2::intTrap   (this); 
	funcPredef_FL3::weightNorm(this);
	weight_ = 1; cost0_  = 0;
    }
};

////////********************************************************************************************************************************
class TestCost_Lin_TPos : public CFLatMap1Weight<TSLatVec, MapData> {
    public  : TestCost_Lin_TPos() { 
	stAcc   = [this](const size_t& _i){ return latticePtr_->at(_i).statePtr->value( asInt(SNF::T) )/*latticePtr_->at(_i).arc - latticePtr_->at(_i-1).arc*/; }; 
	funcPredef_FL1::lin   (this); 
	funcPredef_FL2::sum   (this); 
	funcPredef_FL3::weight(this);
	weight_ = 1; cost0_  = 0;
    }
};
////////********************************************************************************************************************************

class Test1CostArray_LinSumW_V    : public CostsArrayLat<TSLatVec,MapData,TestCost_LinSumW_V    >{ 
    size_t latFuncLayerIdx() override { return TS::lattTypeIdx( asInt(TSBSLT::LATTICE_ARC_EQ_DT) ); }
    size_t latKnotLayerIdx() override { return TS::lattTypeIdx( asInt(TSBSLT::ARC_BG_BK) ); }
    
};
class Test1CostArray_LinSumW_W    : public CostsArrayLat<TSLatVec,MapData,TestCost_LinIntWNorm_W    >{ 
    size_t latFuncLayerIdx() override { return TS::lattTypeIdx( asInt(TSBSLT::LATTICE_ARC_EQ_DT) ); }
    size_t latKnotLayerIdx() override { return TS::lattTypeIdx( asInt(TSBSLT::ARC_BG_BK) ); }
    
};
class Test1CostArray_Lin_TPos: public CostsArrayLat<TSLatVec,MapData,TestCost_Lin_TPos>{ 
    size_t latFuncLayerIdx() override { return TS::lattTypeIdx( 0 ); }
    size_t latKnotLayerIdx() override { return TS::lattTypeIdx( 0 ); }
};


class TestCostsEvaluatorT1 : public CostsEvaluator<TSLatVec, MapData> {
    public   : TestCostsEvaluatorT1(std::shared_ptr<MapData>& _mapDataPtr) : CostsEvaluator(_mapDataPtr) {}
    public   : void setCosts() override {
	partialCostsArray_[asInt(CostEvaluatorCostType::F)].resize(2);
	partialCostsArray_[asInt(CostEvaluatorCostType::F)][0] = std::make_unique< Test1CostArray_LinSumW_V >();
	partialCostsArray_[asInt(CostEvaluatorCostType::F)][1] = std::make_unique< Test1CostArray_LinSumW_W >();
	partialCostsArray_[asInt(CostEvaluatorCostType::H)].resize(1);
	partialCostsArray_[asInt(CostEvaluatorCostType::H)][0] = std::make_unique< Test1CostArray_Lin_TPos >();
    }
};

class OptimizationStateDiffDrive : public OptimizationState {
    public   : StateSPtr cloneState () const override { return std::make_shared<OptimizationStateDiffDrive>(*this); }
    public   : void bindVariables(TrajectorySimulator& _trajSim) override {
	variables.clear();
	//values.emplace_back( OptStateData( *_trajOpt.stateSim_->state0().value(0), valueZero ) );
	auto* paramFuncs = _trajSim.stateSim()->paramFuncs();
	for ( size_t i = 0; i < paramFuncs->funcsSize(); ++i ) {
	    for ( size_t j = 1; j < paramFuncs->funcCtrlPtSize(i); ++j ) {
		variables.emplace_back(  &paramFuncs->ctrlPtVal ( i, j, ParamFuncs::CtrlPtDim::VAL ), &paramFuncs->ctrlPtVal( i, max(0, (int)j-2), ParamFuncs::CtrlPtDim::ARC ) );
	    }
	}
	
	for ( size_t j = 1; j < paramFuncs->funcsArcSize(0); ++j ) {
	    variables.emplace_back(  &paramFuncs->funcsArc ( 0, j ), &paramFuncs->funcsArc( 0,  max(0, (int)j-2) ) );
	}
    }
};

TEST_F ( TrajOptTest, GenericTest ) {
    
    trajOpt = make_shared<TrajectoryOptimizer>( stateSimPtr, std::make_unique<TestCostsEvaluatorT1>(dummyMapData), std::make_shared<OptimizationStateDiffDrive>() ); 
    stateSimPtr->setDiscrType( RungeKutta::DiscretizationType::HEUN );
    vector< vector<double*> > funcKnotsLattice(1, vector<double*>(stateSimPtr->paramFuncs()->funcsArcSize(0),0) ); 
    for(size_t i = 0; i < funcKnotsLattice[0].size();++i){ funcKnotsLattice[0][i] = &stateSimPtr->paramFuncs()->funcsArc(0,i); }
    trajOpt->trajSim()->setUserDefLattice(funcKnotsLattice);    
    trajOpt->setSimMode(TrajectorySimulator::SimMode::PRECALC);

    double dt = 0.01;
    double ds = 0.1;
    
    trajOpt->trajSim()->dt() = dt; 
    trajOpt->trajSim()->ds() = ds;
    
    trajOpt->stepSize() = 1e-5;
    
    trajOpt->computeJacobian(false);
//     coutCostEvalCosts(trajOpt);
    
    auto& costsEvaluator = trajOpt->trajSim()->costsEvaluator_;
    auto f     = costsEvaluator->f;
    auto h     = costsEvaluator->h;
    auto g     = costsEvaluator->g;
    auto gradF = costsEvaluator->gradF;
    auto gradH = costsEvaluator->gradH;
    auto gradG = costsEvaluator->gradG;
    
    trajOpt->computeJacobian();
//     coutCostEvalCosts(trajOpt);
    
    EXPECT_DOUBLE_EQ( f, costsEvaluator->f );
    
    ASSERT_EQ( gradF.size(), costsEvaluator->gradF.size() ); for ( size_t i = 0; i < gradF.size(); ++i ) { EXPECT_DOUBLE_EQ( gradF[i], costsEvaluator->gradF[i] ); }
    ASSERT_EQ( h    .size(), costsEvaluator->h    .size() ); for ( size_t i = 0; i < h    .size(); ++i ) { EXPECT_DOUBLE_EQ( h    [i], costsEvaluator->h    [i] ); }
    ASSERT_EQ( g    .size(), costsEvaluator->g    .size() ); for ( size_t i = 0; i < g    .size(); ++i ) { EXPECT_DOUBLE_EQ( g    [i], costsEvaluator->g    [i] ); }
    
    ASSERT_EQ( gradH.rows(), costsEvaluator->gradH.rows() );
    ASSERT_EQ( gradH.cols(), costsEvaluator->gradH.cols() );
    for ( int i = 0; i < gradH.rows(); ++i ) { for ( int j = 0; j < gradH.cols(); ++j ) { EXPECT_DOUBLE_EQ( gradH(i,j), costsEvaluator->gradH(i,j) ); } }
    
    ASSERT_EQ( gradG.rows(), costsEvaluator->gradG.rows() );
    ASSERT_EQ( gradG.cols(), costsEvaluator->gradG.cols() );
    for ( int i = 0; i < gradG.rows(); ++i ) { for ( int j = 0; j < gradG.cols(); ++j ) { EXPECT_DOUBLE_EQ( gradG(i,j), costsEvaluator->gradG(i,j) ); } }
}

////////////////////////////////////////////--------------------------------------------------////////////////////////////////////////////

}  // namespace

int main ( int argc, char **argv ) {
    ::testing::InitGoogleTest ( &argc, argv );
    return RUN_ALL_TESTS();
}
