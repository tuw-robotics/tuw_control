
#include "gtest/gtest.h"
#include <array>
#include <boost/concept_check.hpp>
#include <tuw_control/trajectory_simulator/trajectory_simulator_precalc.h>
#include <tuw_control/trajectory_simulator/trajectory_simulator_online.h>
#include <tuw_control/trajectory_evaluator/trajectory_sim_grade.h>
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


// void coutCostEvalCosts(TrajectorySimGradeSPtr& trajSimGrade) {
//     cout<<"Cost Functions:"<<endl;
//     cout<<"f = "<<trajSimGrade->trajSim()->costsEvaluator_->f<<endl<<"h = "; 
//     for(size_t i = 0; i < trajSimGrade->trajSim()->costsEvaluator_->h.size(); ++i) { cout<<trajSimGrade->trajSim()->costsEvaluator_->h[i]<<", "; } cout<<endl;cout<<endl;
// }
    
// The fixture for testing class Foo.
class TrajSimGradeTest : public ::testing::Test {
protected:
    // You can remove any or all of the following functions if its body
    // is empty.

    TrajSimGradeTest() {
	
	dummyMapData = make_shared<double>(0);
	
	//initialize parametric functions structure
	using PFS = ParamFuncs::ParamFuncsStructure; using PfCpD = ParamFuncs::CtrlPtDim; using FeM = ParamFuncs::FuncEvalMode; //using EaG = ParamFuncs::EvalArcGuarantee;
	size_t funcIdx = 0; vector<PFS> pf( 2 , PFS() );
	static constexpr const int paramFuncArcRefIdxAll = 0;
	pf[0].ctrlPtsSize = 3; pf[0].ctrlPtsArcRefIdx = paramFuncArcRefIdxAll; 
	pf[1].ctrlPtsSize = 3; pf[1].ctrlPtsArcRefIdx = paramFuncArcRefIdxAll;
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
	funcs->ctrlPtVal(funcIdx, 1, PfCpD::ARC) = 0.5  +initT;
	funcs->ctrlPtVal(funcIdx, 2, PfCpD::ARC) = 1    +initT;
	
	funcIdx = 0;
	funcs->ctrlPtVal(funcIdx, 0, PfCpD::VAL) = 1.;
	funcs->ctrlPtVal(funcIdx, 1, PfCpD::VAL) = 1.;
	funcs->ctrlPtVal(funcIdx, 2, PfCpD::VAL) = 1.;
	
	funcIdx = 1;
	funcs->ctrlPtVal(funcIdx, 0, PfCpD::VAL) =  3.5*M_PI; 
	funcs->ctrlPtVal(funcIdx, 1, PfCpD::VAL) =  1.5*M_PI; 
	funcs->ctrlPtVal(funcIdx, 2, PfCpD::VAL) = -4.5*M_PI; 
	funcs->precompute();
    }
    virtual ~TrajSimGradeTest() { }
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
    TrajectorySimGradeSPtr trajSimGrade;
    shared_ptr<double> dummyMapData;
    
public:
    template<typename CostEvalType>
    void multiCheckCostFunc(const double& _dt, const double& _ds, std::function<double(double,double)> costExpFunc){
	
	trajSimGrade = make_shared<TrajectorySimGrade>( stateSimPtr, std::make_unique<CostEvalType>(dummyMapData) ); 
	trajSimGrade->trajSim()->stateSim()->setDiscrType( RungeKutta::DiscretizationType::HEUN );
	trajSimGrade->setSimMode(TrajectorySimulator::SimMode::PRECALC);

	trajSimGrade->trajSim()->dt() = _dt; 
	trajSimGrade->evaluateTrajectory();
	
	double sumEndAns, sumEndExp;
	sumEndAns = trajSimGrade->trajSim()->costsEvaluator_->f;
	sumEndExp = costExpFunc(_dt,_ds);
	EXPECT_DOUBLE_EQ(sumEndExp, sumEndAns);
	
	trajSimGrade->setSimMode(TrajectorySimulator::SimMode::ONLINE);
	trajSimGrade->trajSim()->dt() = _dt; 
	trajSimGrade->evaluateTrajectory();
	
	sumEndAns = trajSimGrade->trajSim()->costsEvaluator_->f;
	sumEndExp = costExpFunc(_dt,_ds);
	EXPECT_DOUBLE_EQ(sumEndExp, sumEndAns);
	
	
	trajSimGrade->trajSim()->dt() = _dt; 
	trajSimGrade->trajSim()->ds() = _ds;
	trajSimGrade->evaluateTrajectory();
	
	sumEndAns = trajSimGrade->trajSim()->costsEvaluator_->f;
	sumEndExp = costExpFunc(_dt,_ds);
	EXPECT_DOUBLE_EQ(sumEndExp, sumEndAns);
	
	
	trajSimGrade->setSimMode(TrajectorySimulator::SimMode::PRECALC);
	trajSimGrade->trajSim()->dt() = _dt; 
	trajSimGrade->trajSim()->ds() = _ds;
	trajSimGrade->evaluateTrajectory();
	
	sumEndAns = trajSimGrade->trajSim()->costsEvaluator_->f;
	sumEndExp = costExpFunc(_dt,_ds);
	EXPECT_DOUBLE_EQ(sumEndExp, sumEndAns);
    }
    
    template<typename CostEvalType>
    void multiCheckCostFuncArray(const double& _dt, const double& _ds, function< shared_ptr<vector<double>>(double,double)> costExpFunc){
	
	trajSimGrade = make_shared<TrajectorySimGrade>( stateSimPtr, std::make_unique<CostEvalType>(dummyMapData) ); 
	trajSimGrade->trajSim()->stateSim()->setDiscrType( RungeKutta::DiscretizationType::HEUN );
	trajSimGrade->setSimMode(TrajectorySimulator::SimMode::PRECALC);

	trajSimGrade->trajSim()->dt() = _dt; 
	trajSimGrade->evaluateTrajectory();
	
	shared_ptr< vector<double> > ans = make_shared< vector<double> >();
	shared_ptr< vector<double> > exp;
	ans->resize(trajSimGrade->trajSim()->costsEvaluator_->h.size()); for(size_t i = 0; i < ans->size(); ++i) { ans->at(i) = trajSimGrade->trajSim()->costsEvaluator_->h[i]; }
	exp = costExpFunc(_dt,_ds);
	ASSERT_EQ( exp->size(), ans->size() );
	for(size_t i = 0; i < ans->size(); ++i) { EXPECT_DOUBLE_EQ( exp->at(i), ans->at(i) ); }
	
	trajSimGrade->setSimMode(TrajectorySimulator::SimMode::ONLINE);
	trajSimGrade->trajSim()->dt() = _dt; 
	trajSimGrade->evaluateTrajectory();
	
	ans->resize(trajSimGrade->trajSim()->costsEvaluator_->h.size()); for(size_t i = 0; i < ans->size(); ++i) { ans->at(i) = trajSimGrade->trajSim()->costsEvaluator_->h[i]; }
	exp = costExpFunc(_dt,_ds);
	ASSERT_EQ( exp->size(), ans->size() );
	for(size_t i = 0; i < ans->size(); ++i) { EXPECT_DOUBLE_EQ( exp->at(i), ans->at(i) ); }
	
	
	trajSimGrade->trajSim()->dt() = _dt; 
	trajSimGrade->trajSim()->ds() = _ds;
	trajSimGrade->evaluateTrajectory();
	
	ans->resize(trajSimGrade->trajSim()->costsEvaluator_->h.size()); for(size_t i = 0; i < ans->size(); ++i) { ans->at(i) = trajSimGrade->trajSim()->costsEvaluator_->h[i]; }
	exp = costExpFunc(_dt,_ds);
	ASSERT_EQ( exp->size(), ans->size() );
	for(size_t i = 0; i < ans->size(); ++i) { EXPECT_DOUBLE_EQ( exp->at(i), ans->at(i) ); }
	
	
	trajSimGrade->setSimMode(TrajectorySimulator::SimMode::PRECALC);
	trajSimGrade->trajSim()->dt() = _dt; 
	trajSimGrade->trajSim()->ds() = _ds;
	trajSimGrade->evaluateTrajectory();
	
	ans->resize(trajSimGrade->trajSim()->costsEvaluator_->h.size()); for(size_t i = 0; i < ans->size(); ++i) { ans->at(i) = trajSimGrade->trajSim()->costsEvaluator_->h[i]; }
	exp = costExpFunc(_dt,_ds);
	ASSERT_EQ( exp->size(), ans->size() );
	for(size_t i = 0; i < ans->size(); ++i) { EXPECT_DOUBLE_EQ( exp->at(i), ans->at(i) ); }
    }
    
    
    template<typename CostEvalType>
    void multiCheckCostFuncArrayKnot(const double& _dt, const double& _ds, bool checkEqCostsNr ){
	
	trajSimGrade = make_shared<TrajectorySimGrade>( stateSimPtr, std::make_unique<CostEvalType>(dummyMapData) ); 
	trajSimGrade->trajSim()->stateSim()->setDiscrType( RungeKutta::DiscretizationType::HEUN );
	
	ParamFuncs* funcs  = trajSimGrade->trajSim()->stateSim()->paramFuncs();
	vector< vector<double> > funcKnotsLattice(1, vector<double>(funcs->funcsArcSize(0),0) ); 
	for(size_t i = 0; i < funcKnotsLattice[0].size();++i){ funcKnotsLattice[0][i] = funcs->funcsArc(0,i); }
	trajSimGrade->trajSim()->setUserDefLattice(funcKnotsLattice);    
	
	trajSimGrade->setSimMode(TrajectorySimulator::SimMode::PRECALC);

	trajSimGrade->trajSim()->dt() = _dt; 
	trajSimGrade->evaluateTrajectory();
	trajSimGrade->trajSim()->ds() = -1;
	shared_ptr< vector<double> > expPreEq = make_shared< vector<double> >();
	expPreEq->resize(trajSimGrade->trajSim()->costsEvaluator_->h.size()); for(size_t i=0;i<expPreEq->size();++i) { expPreEq->at(i) = trajSimGrade->trajSim()->costsEvaluator_->h[i]; }
// 	coutCostEvalCosts(trajSimGrade);
	
	trajSimGrade->trajSim()->dt() = _dt / 10.; 
	trajSimGrade->trajSim()->ds() = _ds;
	trajSimGrade->evaluateTrajectory();
	shared_ptr< vector<double> > expPreSm = make_shared< vector<double> >();
	expPreSm->resize(trajSimGrade->trajSim()->costsEvaluator_->h.size()); for(size_t i=0;i<expPreSm->size();++i) { expPreSm->at(i) = trajSimGrade->trajSim()->costsEvaluator_->h[i]; }
// 	coutCostEvalCosts(trajSimGrade);
	
	trajSimGrade->trajSim()->dt() = _dt * 10.; 
	trajSimGrade->trajSim()->ds() = -1;
	trajSimGrade->evaluateTrajectory();
	shared_ptr< vector<double> > expPreGr = make_shared< vector<double> >();
	expPreGr->resize(trajSimGrade->trajSim()->costsEvaluator_->h.size()); for(size_t i=0;i<expPreGr->size();++i) { expPreGr->at(i) = trajSimGrade->trajSim()->costsEvaluator_->h[i]; }
// 	coutCostEvalCosts(trajSimGrade);
	
	///////////////////////////////////////////////////////////////////////////////
	trajSimGrade->setSimMode(TrajectorySimulator::SimMode::ONLINE);
	shared_ptr< vector<double> > ansOnline  = make_shared< vector<double> >();
	
	for(size_t i = 0; i < funcKnotsLattice[0].size();++i){ funcKnotsLattice[0][i] = funcs->funcsArc(0,i); } trajSimGrade->trajSim()->setUserDefLattice(funcKnotsLattice);  
	trajSimGrade->trajSim()->dt() = _dt; 
	trajSimGrade->trajSim()->ds() = -1;
	trajSimGrade->evaluateTrajectory();
// 	coutCostEvalCosts(trajSimGrade);
	ansOnline->resize(trajSimGrade->trajSim()->costsEvaluator_->h.size()); for(size_t i=0;i<ansOnline->size();++i) { ansOnline->at(i) = trajSimGrade->trajSim()->costsEvaluator_->h[i]; }
	if(checkEqCostsNr){ASSERT_EQ( expPreEq->size(), ansOnline->size() );}
	for(size_t i = 0; i < ansOnline->size(); ++i) { EXPECT_TRUE( ansOnline->at(i) >= 0 ); EXPECT_DOUBLE_EQ( expPreEq->at(i), ansOnline->at(i) ); }
	
	
	for(size_t i = 0; i < funcKnotsLattice[0].size();++i){ funcKnotsLattice[0][i] = funcs->funcsArc(0,i); } trajSimGrade->trajSim()->setUserDefLattice(funcKnotsLattice);  
	trajSimGrade->trajSim()->dt() = _dt / 10.; 
	trajSimGrade->trajSim()->ds() = _ds;
	trajSimGrade->evaluateTrajectory();
// 	coutCostEvalCosts(trajSimGrade);
	ansOnline->resize(trajSimGrade->trajSim()->costsEvaluator_->h.size()); for(size_t i=0;i<ansOnline->size();++i) { ansOnline->at(i) = trajSimGrade->trajSim()->costsEvaluator_->h[i]; }
	if(checkEqCostsNr){ASSERT_EQ( expPreSm->size(), ansOnline->size() );}
	for(size_t i = 0; i < ansOnline->size(); ++i) { EXPECT_TRUE( ansOnline->at(i) >= 0 ); EXPECT_DOUBLE_EQ( expPreSm->at(i), ansOnline->at(i) ); }
	
	
	for(size_t i = 0; i < funcKnotsLattice[0].size();++i){ funcKnotsLattice[0][i] = funcs->funcsArc(0,i); } trajSimGrade->trajSim()->setUserDefLattice(funcKnotsLattice);  
	trajSimGrade->trajSim()->dt() = _dt * 10; 
	trajSimGrade->trajSim()->ds() = -1;
	trajSimGrade->evaluateTrajectory();
// 	coutCostEvalCosts(trajSimGrade);
	ansOnline->resize(trajSimGrade->trajSim()->costsEvaluator_->h.size()); for(size_t i=0;i<ansOnline->size();++i) { ansOnline->at(i) = trajSimGrade->trajSim()->costsEvaluator_->h[i]; }
	if(checkEqCostsNr){ASSERT_EQ( expPreGr->size(), ansOnline->size() );}
	for(size_t i = 0; i < ansOnline->size(); ++i) { EXPECT_TRUE( ansOnline->at(i) >= 0 ); EXPECT_DOUBLE_EQ( expPreGr->at(i), ansOnline->at(i) ); }
	
	
	for(size_t i = 0; i < funcKnotsLattice[0].size();++i){ funcKnotsLattice[0][i] = funcs->funcsArc(0,i); } trajSimGrade->trajSim()->setUserDefLattice(funcKnotsLattice);  
	trajSimGrade->trajSim()->dt() = _dt / 10.; 
	trajSimGrade->trajSim()->ds() = _ds;
	trajSimGrade->evaluateTrajectory();
// 	coutCostEvalCosts(trajSimGrade);
	ansOnline->resize(trajSimGrade->trajSim()->costsEvaluator_->h.size()); for(size_t i=0;i<ansOnline->size();++i) { ansOnline->at(i) = trajSimGrade->trajSim()->costsEvaluator_->h[i]; }
	if(checkEqCostsNr){ASSERT_EQ( expPreEq->size(), ansOnline->size() );}
	for(size_t i = 0; i < ansOnline->size(); ++i) { EXPECT_TRUE( ansOnline->at(i) >= 0 ); EXPECT_DOUBLE_EQ( expPreSm->at(i), ansOnline->at(i) ); }
    }
    
    template<typename CostEvalType>
    void multiCheckCostFuncArrayKnotOnlyDt(const double& _dt ){
	trajSimGrade = make_shared<TrajectorySimGrade>( stateSimPtr, std::make_unique<CostEvalType>(dummyMapData) ); 
	trajSimGrade->trajSim()->stateSim()->setDiscrType( RungeKutta::DiscretizationType::HEUN );
	trajSimGrade->setSimMode(TrajectorySimulator::SimMode::PRECALC);

	trajSimGrade->trajSim()->dt() = _dt; 
	trajSimGrade->evaluateTrajectory();
	shared_ptr< vector<double> > expPreEq = make_shared< vector<double> >();
	expPreEq->resize(trajSimGrade->trajSim()->costsEvaluator_->h.size()); for(size_t i=0;i<expPreEq->size();++i) { expPreEq->at(i) = trajSimGrade->trajSim()->costsEvaluator_->h[i]; }
// 	coutCostEvalCosts(trajSimGrade);
	
	trajSimGrade->trajSim()->dt() = _dt / 10.; 
	trajSimGrade->evaluateTrajectory();
	shared_ptr< vector<double> > expPreSm = make_shared< vector<double> >();
	expPreSm->resize(trajSimGrade->trajSim()->costsEvaluator_->h.size()); for(size_t i=0;i<expPreSm->size();++i) { expPreSm->at(i) = trajSimGrade->trajSim()->costsEvaluator_->h[i]; }
// 	coutCostEvalCosts(trajSimGrade);
	
	trajSimGrade->trajSim()->dt() = _dt * 10.;
	trajSimGrade->evaluateTrajectory();
	shared_ptr< vector<double> > expPreGr = make_shared< vector<double> >();
	expPreGr->resize(trajSimGrade->trajSim()->costsEvaluator_->h.size()); for(size_t i=0;i<expPreGr->size();++i) { expPreGr->at(i) = trajSimGrade->trajSim()->costsEvaluator_->h[i]; }
// 	coutCostEvalCosts(trajSimGrade);
	
	///////////////////////////////////////////////////////////////////////////////
	trajSimGrade->setSimMode(TrajectorySimulator::SimMode::ONLINE);
	shared_ptr< vector<double> > ansOnline  = make_shared< vector<double> >();
	
	trajSimGrade->trajSim()->dt() = _dt; 
	trajSimGrade->evaluateTrajectory();
// 	coutCostEvalCosts(trajSimGrade);
	ansOnline->resize(trajSimGrade->trajSim()->costsEvaluator_->h.size()); for(size_t i=0;i<ansOnline->size();++i) { ansOnline->at(i) = trajSimGrade->trajSim()->costsEvaluator_->h[i]; }
	for(size_t i = 0; i < ansOnline->size(); ++i) { EXPECT_TRUE( ansOnline->at(i) >= 0 ); EXPECT_DOUBLE_EQ( expPreEq->at(i), ansOnline->at(i) ); }
	
	
	trajSimGrade->trajSim()->dt() = _dt / 10.; 
	trajSimGrade->evaluateTrajectory();
// 	coutCostEvalCosts(trajSimGrade);
	ansOnline->resize(trajSimGrade->trajSim()->costsEvaluator_->h.size()); for(size_t i=0;i<ansOnline->size();++i) { ansOnline->at(i) = trajSimGrade->trajSim()->costsEvaluator_->h[i]; }
	for(size_t i = 0; i < ansOnline->size(); ++i) { EXPECT_TRUE( ansOnline->at(i) >= 0 ); EXPECT_DOUBLE_EQ( expPreSm->at(i), ansOnline->at(i) ); }
	
	trajSimGrade->trajSim()->dt() = _dt * 10; 
	trajSimGrade->evaluateTrajectory();
// 	coutCostEvalCosts(trajSimGrade);
	ansOnline->resize(trajSimGrade->trajSim()->costsEvaluator_->h.size()); for(size_t i=0;i<ansOnline->size();++i) { ansOnline->at(i) = trajSimGrade->trajSim()->costsEvaluator_->h[i]; }
	for(size_t i = 0; i < ansOnline->size(); ++i) { EXPECT_TRUE( ansOnline->at(i) >= 0 ); EXPECT_DOUBLE_EQ( expPreGr->at(i), ansOnline->at(i) ); }
	
	
	trajSimGrade->trajSim()->dt() = _dt / 10.; 
	trajSimGrade->evaluateTrajectory();
// 	coutCostEvalCosts(trajSimGrade);
	ansOnline->resize(trajSimGrade->trajSim()->costsEvaluator_->h.size()); for(size_t i=0;i<ansOnline->size();++i) { ansOnline->at(i) = trajSimGrade->trajSim()->costsEvaluator_->h[i]; }
	for(size_t i = 0; i < ansOnline->size(); ++i) { EXPECT_TRUE( ansOnline->at(i) >= 0 ); EXPECT_DOUBLE_EQ( expPreSm->at(i), ansOnline->at(i) ); }
    }

};

////////////////////////////////////////////--------------------------------------------------////////////////////////////////////////////


using       TS = TrajectorySimulator;
using   TSBSLT = TS::BaseSimLatticeType;
using TSLatVec = TS::LatticeVec;

using MapData       = double;
using namespace tuw::cost_functions;

////////********************************************************************************************************************************
class TestCost_LinSumW : public CFLatMap1Weight<TSLatVec, MapData> {
    public  : TestCost_LinSumW() { 
	stAcc   = [this](const size_t& _i){ return latticePtr_->at(_i).statePtr->value( asInt(SNF::T) ); }; 
	funcPredef_FL1::lin   (this); 
	funcPredef_FL2::sum   (this); 
	funcPredef_FL3::weight(this);
	weight_ = 1; cost0_  = 0;
    }
};


////////********************************************************************************************************************************
class TestCost_LinSumWNorm : public CFLatMap1Weight<TSLatVec, MapData> {
    public  : TestCost_LinSumWNorm() { 
	stAcc   = [this](const size_t& _i){ return latticePtr_->at(_i).statePtr->value( asInt(SNF::T) ); }; 
	funcPredef_FL1::lin       (this); 
	funcPredef_FL2::sum       (this); 
	funcPredef_FL3::weightNorm(this);
	weight_ = 1; cost0_  = 0;
    }
};
////////********************************************************************************************************************************
class TestCost_LinIntWNorm : public CFLatMap1Weight<TSLatVec, MapData> {
    public  : TestCost_LinIntWNorm() { 
	stAcc   = [this](const size_t& _i){ return latticePtr_->at(_i).statePtr->value( asInt(SNF::T) ); }; 
	funcPredef_FL1::lin       (this); 
	funcPredef_FL2::intTrap   (this); 
	funcPredef_FL3::weightNorm(this);
	weight_ = 1; cost0_  = 0;
    }
};
////////********************************************************************************************************************************

class Test1CostArray_LinSumW    : public CostsArrayLat<TSLatVec,MapData,TestCost_LinSumW    >{ 
    size_t latFuncLayerIdx() override { return TS::lattTypeIdx( asInt(TSBSLT::LATTICE_ARC_EQ_DT) ); }
    size_t latKnotLayerIdx() override { return TS::lattTypeIdx( asInt(TSBSLT::ARC_BG_BK) ); }
    
};
class Test1CostArray_LinSumWNorm: public CostsArrayLat<TSLatVec,MapData,TestCost_LinSumWNorm>{ 
    size_t latFuncLayerIdx() override { return TS::lattTypeIdx( asInt(TSBSLT::LATTICE_ARC_EQ_DT) ); }
    size_t latKnotLayerIdx() override { return TS::lattTypeIdx( asInt(TSBSLT::ARC_BG_BK) ); }
};
class Test1CostArray_LinIntWNorm: public CostsArrayLat<TSLatVec,MapData,TestCost_LinIntWNorm>{ 
    size_t latFuncLayerIdx() override { return TS::lattTypeIdx(asInt(TSBSLT::LATTICE_ARC_EQ_DT) ); }
    size_t latKnotLayerIdx() override { return TS::lattTypeIdx(asInt(TSBSLT::ARC_BG_BK) ); }
    
};


class TestCostsEvaluatorT1C1 : public CostsEvaluator<TSLatVec, MapData> {
    public   : TestCostsEvaluatorT1C1(std::shared_ptr<MapData>& _mapDataPtr) : CostsEvaluator(_mapDataPtr) {}
    public   : void setCosts() override {
	partialCostsArray_[asInt(CostEvaluatorCostType::F)].resize(1);
	partialCostsArray_[asInt(CostEvaluatorCostType::F)][0] = std::make_unique< Test1CostArray_LinSumW    >();
    }
};
class TestCostsEvaluatorT1C2 : public CostsEvaluator<TSLatVec, MapData> {
    public   : TestCostsEvaluatorT1C2(std::shared_ptr<MapData>& _mapDataPtr) : CostsEvaluator(_mapDataPtr) {}
    public   : void setCosts() override {
	partialCostsArray_[asInt(CostEvaluatorCostType::F)].resize(1);
	partialCostsArray_[asInt(CostEvaluatorCostType::F)][0] = std::make_unique< Test1CostArray_LinSumWNorm    >();
    }
};
class TestCostsEvaluatorT1C3 : public CostsEvaluator<TSLatVec, MapData> {
    public   : TestCostsEvaluatorT1C3(std::shared_ptr<MapData>& _mapDataPtr) : CostsEvaluator(_mapDataPtr) {}
    public   : void setCosts() override {
	partialCostsArray_[asInt(CostEvaluatorCostType::F)].resize(1);
	partialCostsArray_[asInt(CostEvaluatorCostType::F)][0] = std::make_unique< Test1CostArray_LinIntWNorm    >();
    }
};

TEST_F ( TrajSimGradeTest, CostFuncsOnTOnT ) {
    double dt, ds;
    
    {
    auto sumeEndExpFunc = [this](double dt, double ds){ 
	double sumEndExp = 0; size_t i = 1;
	for(; i <= 1. / trajSimGrade->trajSim()->dt(); ++i) { sumEndExp += (double)i * dt; } /*sumEndExp+=1- --i*dt;*/  return sumEndExp;  };
    
    dt = 0.01; ds = 0.02; multiCheckCostFunc<TestCostsEvaluatorT1C1>(dt, ds, sumeEndExpFunc);
    dt =    1; ds = 0.05; multiCheckCostFunc<TestCostsEvaluatorT1C1>(dt, ds, sumeEndExpFunc);
    dt = 1e-4; ds = 0.02; multiCheckCostFunc<TestCostsEvaluatorT1C1>(dt, ds, sumeEndExpFunc);
    dt = 0.1 ; ds = -1  ; multiCheckCostFunc<TestCostsEvaluatorT1C1>(dt, ds, sumeEndExpFunc);
    }
    
    {
    auto sumeEndExpFunc = [this](double dt, double ds){ 
	double sumEndExp = 0; size_t i = 1;
	for(; i <= 1. / trajSimGrade->trajSim()->dt(); ++i) { sumEndExp += (double)i * dt; }  return sumEndExp / ( (double)--i * dt );};
    
    dt = 0.01; ds = 0.02; multiCheckCostFunc<TestCostsEvaluatorT1C2>(dt, ds, sumeEndExpFunc);
    dt =    1; ds = 0.05; multiCheckCostFunc<TestCostsEvaluatorT1C2>(dt, ds, sumeEndExpFunc);
    dt = 1e-4; ds = 0.02; multiCheckCostFunc<TestCostsEvaluatorT1C2>(dt, ds, sumeEndExpFunc);
    dt = 0.1 ; ds = -1  ; multiCheckCostFunc<TestCostsEvaluatorT1C2>(dt, ds, sumeEndExpFunc);
    }
    
    {
    auto sumeEndExpFunc = [this](double dt, double ds){ 
	double sumEndExp = 0; size_t i = 1;
	for(; i <= 1. / trajSimGrade->trajSim()->dt(); ++i) { sumEndExp += 0.5 * dt * ( (i+i-1)*dt ); } return sumEndExp / ((double)--i * dt);  };
    
    dt = 0.01; ds = 0.02; multiCheckCostFunc<TestCostsEvaluatorT1C3>(dt, ds, sumeEndExpFunc);
    dt =    1; ds = 0.05; multiCheckCostFunc<TestCostsEvaluatorT1C3>(dt, ds, sumeEndExpFunc);
    dt = 1e-4; ds = 0.02; multiCheckCostFunc<TestCostsEvaluatorT1C3>(dt, ds, sumeEndExpFunc);
    dt = 0.1 ; ds = -1  ; multiCheckCostFunc<TestCostsEvaluatorT1C3>(dt, ds, sumeEndExpFunc);
    }
}

class Test2CostArray_LinSumW    : public CostsArrayLat<TSLatVec,MapData,TestCost_LinSumW    >{ 
    size_t latFuncLayerIdx() override { return TS::lattTypeIdx( asInt(TSBSLT::LATTICE_ARC_EQ_DT) ); }
    size_t latKnotLayerIdx() override { return TS::lattTypeIdx( asInt(TSBSLT::LATTICE_ARC_EQ_DT) ); }
    
};
class Test2CostArray_LinSumWNorm: public CostsArrayLat<TSLatVec,MapData,TestCost_LinSumWNorm>{ 
    size_t latFuncLayerIdx() override { return TS::lattTypeIdx( asInt(TSBSLT::LATTICE_ARC_EQ_DT) ); }
    size_t latKnotLayerIdx() override { return TS::lattTypeIdx( asInt(TSBSLT::LATTICE_ARC_EQ_DT) ); }
};
class Test2CostArray_LinIntWNorm: public CostsArrayLat<TSLatVec,MapData,TestCost_LinIntWNorm>{ 
    size_t latFuncLayerIdx() override { return TS::lattTypeIdx(asInt(TSBSLT::LATTICE_ARC_EQ_DT) ); }
    size_t latKnotLayerIdx() override { return TS::lattTypeIdx(asInt(TSBSLT::LATTICE_ARC_EQ_DT) ); }
    
};


class TestCostsEvaluatorT2C1 : public CostsEvaluator<TSLatVec, MapData> {
    public   : TestCostsEvaluatorT2C1(std::shared_ptr<MapData>& _mapDataPtr) : CostsEvaluator(_mapDataPtr) {}
    public   : void setCosts() override {
	partialCostsArray_[asInt(CostEvaluatorCostType::H)].resize(1);
	partialCostsArray_[asInt(CostEvaluatorCostType::H)][0] = std::make_unique< Test2CostArray_LinSumW    >();
    }
};
class TestCostsEvaluatorT2C2 : public CostsEvaluator<TSLatVec, MapData> {
    public   : TestCostsEvaluatorT2C2(std::shared_ptr<MapData>& _mapDataPtr) : CostsEvaluator(_mapDataPtr) {}
    public   : void setCosts() override {
	partialCostsArray_[asInt(CostEvaluatorCostType::H)].resize(1);
	partialCostsArray_[asInt(CostEvaluatorCostType::H)][0] = std::make_unique< Test2CostArray_LinSumWNorm    >();
    }
};
class TestCostsEvaluatorT2C3 : public CostsEvaluator<TSLatVec, MapData> {
    public   : TestCostsEvaluatorT2C3(std::shared_ptr<MapData>& _mapDataPtr) : CostsEvaluator(_mapDataPtr) {}
    public   : void setCosts() override {
	partialCostsArray_[asInt(CostEvaluatorCostType::H)].resize(1);
	partialCostsArray_[asInt(CostEvaluatorCostType::H)][0] = std::make_unique< Test2CostArray_LinIntWNorm    >();
    }
};

TEST_F ( TrajSimGradeTest, CostFuncsArrayOnTOnTLat ) {
    double dt, ds;
    
    {
    auto sumeEndExpFunc = [this](double dt, double ds){ 
	shared_ptr<vector<double>> costsExp = make_shared< vector<double> >( trajSimGrade->trajSim()->stateSim()->paramFuncs()->funcsArcEnd() / (double)dt ); 
	for(size_t i = 1; i <= 1. / trajSimGrade->trajSim()->dt(); ++i) { costsExp->at(i-1) = (double)i * dt; } return costsExp;  };
    
    dt = 0.01; ds = 0.02; multiCheckCostFuncArray<TestCostsEvaluatorT2C1>(dt, ds, sumeEndExpFunc);
    dt =    1; ds = 0.05; multiCheckCostFuncArray<TestCostsEvaluatorT2C1>(dt, ds, sumeEndExpFunc);
    dt = 1e-3; ds = 0.02; multiCheckCostFuncArray<TestCostsEvaluatorT2C1>(dt, ds, sumeEndExpFunc);
    dt = 0.1 ; ds = -1  ; multiCheckCostFuncArray<TestCostsEvaluatorT2C1>(dt, ds, sumeEndExpFunc);
    }
    
    {
    auto sumeEndExpFunc = [this](double dt, double ds){ 
	shared_ptr<vector<double>> costsExp = make_shared< vector<double> >( trajSimGrade->trajSim()->stateSim()->paramFuncs()->funcsArcEnd() / (double)dt ); 
	for(size_t i = 1; i <= 1. / trajSimGrade->trajSim()->dt(); ++i) { costsExp->at(i-1) = (double)i * dt / (i*dt - (i-1)*dt); } return costsExp;  };
    
    dt = 0.01; ds = 0.02; multiCheckCostFuncArray<TestCostsEvaluatorT2C2>(dt, ds, sumeEndExpFunc);
    dt =    1; ds = 0.05; multiCheckCostFuncArray<TestCostsEvaluatorT2C2>(dt, ds, sumeEndExpFunc);
    dt = 1e-3; ds = 0.02; multiCheckCostFuncArray<TestCostsEvaluatorT2C2>(dt, ds, sumeEndExpFunc);
    dt = 0.1 ; ds = -1  ; multiCheckCostFuncArray<TestCostsEvaluatorT2C2>(dt, ds, sumeEndExpFunc);
    }
    
    {
    auto sumeEndExpFunc = [this](double dt, double ds){ 
	shared_ptr<vector<double>> costsExp = make_shared< vector<double> >( trajSimGrade->trajSim()->stateSim()->paramFuncs()->funcsArcEnd() / (double)dt ); 
	for(size_t i = 1; i <= 1. / trajSimGrade->trajSim()->dt(); ++i) { double ii = i; costsExp->at(i-1) = ( ii*dt - (ii-1)*dt) * ( ii*dt+(ii-1)*dt ) / (ii*dt - (ii-1)*dt) / 2. ; } return costsExp;  };
    
    dt = 0.01; ds = 0.02; multiCheckCostFuncArray<TestCostsEvaluatorT2C3>(dt, ds, sumeEndExpFunc);
    dt =    1; ds = 0.05; multiCheckCostFuncArray<TestCostsEvaluatorT2C3>(dt, ds, sumeEndExpFunc);
    dt = 1e-3; ds = 0.02; multiCheckCostFuncArray<TestCostsEvaluatorT2C3>(dt, ds, sumeEndExpFunc);
    dt = 0.1 ; ds = -1  ; multiCheckCostFuncArray<TestCostsEvaluatorT2C3>(dt, ds, sumeEndExpFunc);
    }
}


class Test3CostArray_LinSumW    : public Test2CostArray_LinSumW    { size_t latKnotLayerIdx()override{return TS::lattTypeIdx(0);}};
class Test3CostArray_LinSumWNorm: public Test2CostArray_LinSumWNorm{ size_t latKnotLayerIdx()override{return TS::lattTypeIdx(0);}};
class Test3CostArray_LinIntWNorm: public Test2CostArray_LinIntWNorm{ size_t latKnotLayerIdx()override{return TS::lattTypeIdx(0);}};


class TestCostsEvaluatorT3C1 : public CostsEvaluator<TSLatVec, MapData> {
    public   : TestCostsEvaluatorT3C1(std::shared_ptr<MapData>& _mapDataPtr) : CostsEvaluator(_mapDataPtr) {}
    public   : void setCosts() override {
	partialCostsArray_[asInt(CostEvaluatorCostType::H)].resize(1);
	partialCostsArray_[asInt(CostEvaluatorCostType::H)][0] = std::make_unique< Test3CostArray_LinSumW    >();
    }
};
class TestCostsEvaluatorT3C2 : public CostsEvaluator<TSLatVec, MapData> {
    public   : TestCostsEvaluatorT3C2(std::shared_ptr<MapData>& _mapDataPtr) : CostsEvaluator(_mapDataPtr) {}
    public   : void setCosts() override {
	partialCostsArray_[asInt(CostEvaluatorCostType::H)].resize(1);
	partialCostsArray_[asInt(CostEvaluatorCostType::H)][0] = std::make_unique< Test3CostArray_LinSumWNorm    >();
    }
};
class TestCostsEvaluatorT3C3 : public CostsEvaluator<TSLatVec, MapData> {
    public   : TestCostsEvaluatorT3C3(std::shared_ptr<MapData>& _mapDataPtr) : CostsEvaluator(_mapDataPtr) {}
    public   : void setCosts() override {
	partialCostsArray_[asInt(CostEvaluatorCostType::H)].resize(1);
	partialCostsArray_[asInt(CostEvaluatorCostType::H)][0] = std::make_unique< Test3CostArray_LinIntWNorm    >();
    }
};



TEST_F ( TrajSimGradeTest, CostFuncsArrayOnTOnTLatFuncKnots ) {
    double dt, ds;
    
    dt = 0.01 ; ds = 0.02; multiCheckCostFuncArrayKnot<TestCostsEvaluatorT3C1>(dt, ds, true);
    dt = 0.1  ; ds = 0.05; multiCheckCostFuncArrayKnot<TestCostsEvaluatorT3C1>(dt, ds, true);
    dt = 0.001; ds = -1  ; multiCheckCostFuncArrayKnot<TestCostsEvaluatorT3C1>(dt, ds, true);
    
    dt = 0.01 ; ds = 0.02; multiCheckCostFuncArrayKnot<TestCostsEvaluatorT3C2>(dt, ds, true);
    dt = 0.1  ; ds = 0.05; multiCheckCostFuncArrayKnot<TestCostsEvaluatorT3C2>(dt, ds, true);
    dt = 0.001; ds = -1  ; multiCheckCostFuncArrayKnot<TestCostsEvaluatorT3C2>(dt, ds, true);
    
    dt = 0.01 ; ds = 0.02; multiCheckCostFuncArrayKnot<TestCostsEvaluatorT3C3>(dt, ds, true);
    dt = 0.1  ; ds = 0.05; multiCheckCostFuncArrayKnot<TestCostsEvaluatorT3C3>(dt, ds, true);
    dt = 0.001; ds = -1  ; multiCheckCostFuncArrayKnot<TestCostsEvaluatorT3C3>(dt, ds, true);
}

class TestCost_LinSumW_onX : public TestCost_LinSumW {
    public  : TestCost_LinSumW_onX    () : TestCost_LinSumW    () {  stAcc = [this](const size_t& _i){ return latticePtr_->at(_i).statePtr->value( asInt(SNF::X) ); };  }
};
class TestCost_LinSumWNorm_onX : public TestCost_LinSumWNorm {
    public  : TestCost_LinSumWNorm_onX() : TestCost_LinSumWNorm() {  stAcc = [this](const size_t& _i){ return latticePtr_->at(_i).statePtr->value( asInt(SNF::X) ); };  }
};
class TestCost_LinIntWNorm_onX : public TestCost_LinIntWNorm {
    public  : TestCost_LinIntWNorm_onX() : TestCost_LinIntWNorm() {  stAcc = [this](const size_t& _i){ return latticePtr_->at(_i).statePtr->value( asInt(SNF::X) ); };  }
};

class Test4CostArray_LinSumW    : public CostsArrayLat<TSLatVec,MapData,TestCost_LinSumW_onX    > { 
    size_t latFuncLayerIdx() override { return TS::lattTypeIdx( asInt(TSBSLT::LATTICE_ARC_EQ_DT) ); }
    size_t latKnotLayerIdx() override { return TS::lattTypeIdx( asInt(TSBSLT::LATTICE_ARC_EQ_DT) ); }  
};
class Test4CostArray_LinSumWNorm: public CostsArrayLat<TSLatVec,MapData,TestCost_LinSumWNorm_onX> { 
    size_t latFuncLayerIdx() override { return TS::lattTypeIdx( asInt(TSBSLT::LATTICE_ARC_EQ_DT) ); }
    size_t latKnotLayerIdx() override { return TS::lattTypeIdx( asInt(TSBSLT::LATTICE_ARC_EQ_DT) ); }  
};
class Test4CostArray_LinIntWNorm: public CostsArrayLat<TSLatVec,MapData,TestCost_LinIntWNorm_onX> { 
    size_t latFuncLayerIdx() override { return TS::lattTypeIdx( asInt(TSBSLT::LATTICE_ARC_EQ_DT) ); }
    size_t latKnotLayerIdx() override { return TS::lattTypeIdx( asInt(TSBSLT::LATTICE_ARC_EQ_DT) ); }  
};

class TestCostsEvaluatorT4C1 : public CostsEvaluator<TSLatVec, MapData> {
    public   : TestCostsEvaluatorT4C1(std::shared_ptr<MapData>& _mapDataPtr) : CostsEvaluator(_mapDataPtr) {}
    public   : void setCosts() override {
	partialCostsArray_[asInt(CostEvaluatorCostType::H)].resize(1);
	partialCostsArray_[asInt(CostEvaluatorCostType::H)][0] = std::make_unique< Test4CostArray_LinSumW    >();
    }
};
class TestCostsEvaluatorT4C2 : public CostsEvaluator<TSLatVec, MapData> {
    public   : TestCostsEvaluatorT4C2(std::shared_ptr<MapData>& _mapDataPtr) : CostsEvaluator(_mapDataPtr) {}
    public   : void setCosts() override {
	partialCostsArray_[asInt(CostEvaluatorCostType::H)].resize(1);
	partialCostsArray_[asInt(CostEvaluatorCostType::H)][0] = std::make_unique< Test4CostArray_LinSumWNorm    >();
    }
};
class TestCostsEvaluatorT4C3 : public CostsEvaluator<TSLatVec, MapData> {
    public   : TestCostsEvaluatorT4C3(std::shared_ptr<MapData>& _mapDataPtr) : CostsEvaluator(_mapDataPtr) {}
    public   : void setCosts() override {
	partialCostsArray_[asInt(CostEvaluatorCostType::H)].resize(1);
	partialCostsArray_[asInt(CostEvaluatorCostType::H)][0] = std::make_unique< Test4CostArray_LinIntWNorm    >();
    }
};

TEST_F ( TrajSimGradeTest, CostFuncsArrayOnTOnTLatFuncKnotsOnlineInterrupt ) {
    double dt, ds;
    
    dt = 0.01 ; ds = 0.02; multiCheckCostFuncArrayKnot<TestCostsEvaluatorT4C1>(dt, ds, false);
    dt = 0.1  ; ds = 0.05; multiCheckCostFuncArrayKnot<TestCostsEvaluatorT4C1>(dt, ds, false);
    dt = 0.001; ds = -1  ; multiCheckCostFuncArrayKnot<TestCostsEvaluatorT4C1>(dt, ds, false);
    
    dt = 0.01 ; ds = 0.02; multiCheckCostFuncArrayKnot<TestCostsEvaluatorT4C2>(dt, ds, false);
    dt = 0.1  ; ds = 0.05; multiCheckCostFuncArrayKnot<TestCostsEvaluatorT4C2>(dt, ds, false);
    dt = 0.001; ds = -1  ; multiCheckCostFuncArrayKnot<TestCostsEvaluatorT4C2>(dt, ds, false);
    
    dt = 0.01 ; ds = 0.02; multiCheckCostFuncArrayKnot<TestCostsEvaluatorT4C3>(dt, ds, false);
    dt = 0.1  ; ds = 0.05; multiCheckCostFuncArrayKnot<TestCostsEvaluatorT4C3>(dt, ds, false);
    dt = 0.001; ds = -1  ; multiCheckCostFuncArrayKnot<TestCostsEvaluatorT4C3>(dt, ds, false);
}

TEST_F ( TrajSimGradeTest, CostFuncsArrayOnTOnTLatFuncKnotsOnlineInterruptOnlyT ) {
    double dt;
    
    dt = 0.01 ;multiCheckCostFuncArrayKnotOnlyDt<TestCostsEvaluatorT4C1>(dt);
    dt = 0.1  ;multiCheckCostFuncArrayKnotOnlyDt<TestCostsEvaluatorT4C1>(dt);
    
    dt = 0.01 ;multiCheckCostFuncArrayKnotOnlyDt<TestCostsEvaluatorT4C2>(dt);
    dt = 0.1  ;multiCheckCostFuncArrayKnotOnlyDt<TestCostsEvaluatorT4C2>(dt);
    
    dt = 0.01 ;multiCheckCostFuncArrayKnotOnlyDt<TestCostsEvaluatorT4C3>(dt);
    dt = 0.1  ;multiCheckCostFuncArrayKnotOnlyDt<TestCostsEvaluatorT4C3>(dt);
    
}




class Test6CostArray_LinSumW    : public CostsArrayLat<TSLatVec,MapData,TestCost_LinSumW_onX    > { 
    size_t latFuncLayerIdx() override { return TS::lattTypeIdx( asInt(TSBSLT::LATTICE_ARC_EQ_DT) ); }
    size_t latKnotLayerIdx() override { return TS::lattTypeIdx( asInt(TSBSLT::ARC_BG_BK) ); }  
};
class Test6CostArray_LinSumWNorm: public CostsArrayLat<TSLatVec,MapData,TestCost_LinSumWNorm_onX> { 
    size_t latFuncLayerIdx() override { return TS::lattTypeIdx( asInt(TSBSLT::LATTICE_ARC_EQ_DT) ); }
    size_t latKnotLayerIdx() override { return TS::lattTypeIdx( asInt(TSBSLT::ARC_BG_BK) ); }  
};
class Test6CostArray_LinIntWNorm: public CostsArrayLat<TSLatVec,MapData,TestCost_LinIntWNorm_onX> { 
    size_t latFuncLayerIdx() override { return TS::lattTypeIdx( asInt(TSBSLT::LATTICE_ARC_EQ_DT) ); }
    size_t latKnotLayerIdx() override { return TS::lattTypeIdx( asInt(TSBSLT::ARC_BG_BK) ); }  
};

class TestCostsEvaluatorT6C1 : public CostsEvaluator<TSLatVec, MapData> {
    public   : TestCostsEvaluatorT6C1(std::shared_ptr<MapData>& _mapDataPtr) : CostsEvaluator(_mapDataPtr) {}
    public   : void setCosts() override {
	partialCostsArray_[asInt(CostEvaluatorCostType::H)].resize(1);
	partialCostsArray_[asInt(CostEvaluatorCostType::H)][0] = std::make_unique< Test6CostArray_LinSumW    >();
    }
};
class TestCostsEvaluatorT6C2 : public CostsEvaluator<TSLatVec, MapData> {
    public   : TestCostsEvaluatorT6C2(std::shared_ptr<MapData>& _mapDataPtr) : CostsEvaluator(_mapDataPtr) {}
    public   : void setCosts() override {
	partialCostsArray_[asInt(CostEvaluatorCostType::H)].resize(1);
	partialCostsArray_[asInt(CostEvaluatorCostType::H)][0] = std::make_unique< Test6CostArray_LinSumWNorm    >();
    }
};
class TestCostsEvaluatorT6C3 : public CostsEvaluator<TSLatVec, MapData> {
    public   : TestCostsEvaluatorT6C3(std::shared_ptr<MapData>& _mapDataPtr) : CostsEvaluator(_mapDataPtr) {}
    public   : void setCosts() override {
	partialCostsArray_[asInt(CostEvaluatorCostType::H)].resize(1);
	partialCostsArray_[asInt(CostEvaluatorCostType::H)][0] = std::make_unique< Test6CostArray_LinIntWNorm    >();
    }
};

TEST_F ( TrajSimGradeTest, CostFuncsArrayOnTOnTLatFuncKnotsOnlineInterruptOnlyTSmallerKnots ) {
    double dt;
    
    dt = 0.1  ;multiCheckCostFuncArrayKnotOnlyDt<TestCostsEvaluatorT6C1>(dt);
    dt = 0.01 ;multiCheckCostFuncArrayKnotOnlyDt<TestCostsEvaluatorT6C1>(dt);
    
    
    dt = 0.01 ;multiCheckCostFuncArrayKnotOnlyDt<TestCostsEvaluatorT6C2>(dt);
    dt = 0.1  ;multiCheckCostFuncArrayKnotOnlyDt<TestCostsEvaluatorT6C2>(dt);
    
    dt = 0.01 ;multiCheckCostFuncArrayKnotOnlyDt<TestCostsEvaluatorT6C3>(dt);
    dt = 0.1  ;multiCheckCostFuncArrayKnotOnlyDt<TestCostsEvaluatorT6C3>(dt);
    
}

class Test7CostArray_LinSumW    : public CostsArrayLat<TSLatVec,MapData,TestCost_LinSumW_onX    > { 
    size_t latFuncLayerIdx() override { return TS::lattTypeIdx( asInt(TSBSLT::ARC_BG_BK) ); }
    size_t latKnotLayerIdx() override { return TS::lattTypeIdx( asInt(TSBSLT::LATTICE_ARC_EQ_DT) ); }  
};
class Test7CostArray_LinSumWNorm: public CostsArrayLat<TSLatVec,MapData,TestCost_LinSumWNorm_onX> { 
    size_t latFuncLayerIdx() override { return TS::lattTypeIdx( asInt(TSBSLT::ARC_BG_BK) ); }
    size_t latKnotLayerIdx() override { return TS::lattTypeIdx( asInt(TSBSLT::LATTICE_ARC_EQ_DT) ); }  
};
class Test7CostArray_LinIntWNorm: public CostsArrayLat<TSLatVec,MapData,TestCost_LinIntWNorm_onX> { 
    size_t latFuncLayerIdx() override { return TS::lattTypeIdx( asInt(TSBSLT::ARC_BG_BK) ); }
    size_t latKnotLayerIdx() override { return TS::lattTypeIdx( asInt(TSBSLT::LATTICE_ARC_EQ_DT) ); }  
};

class TestCostsEvaluatorT7C1 : public CostsEvaluator<TSLatVec, MapData> {
    public   : TestCostsEvaluatorT7C1(std::shared_ptr<MapData>& _mapDataPtr) : CostsEvaluator(_mapDataPtr) {}
    public   : void setCosts() override {
	partialCostsArray_[asInt(CostEvaluatorCostType::H)].resize(1);
	partialCostsArray_[asInt(CostEvaluatorCostType::H)][0] = std::make_unique< Test7CostArray_LinSumW    >();
    }
};
class TestCostsEvaluatorT7C2 : public CostsEvaluator<TSLatVec, MapData> {
    public   : TestCostsEvaluatorT7C2(std::shared_ptr<MapData>& _mapDataPtr) : CostsEvaluator(_mapDataPtr) {}
    public   : void setCosts() override {
	partialCostsArray_[asInt(CostEvaluatorCostType::H)].resize(1);
	partialCostsArray_[asInt(CostEvaluatorCostType::H)][0] = std::make_unique< Test7CostArray_LinSumWNorm    >();
    }
};
class TestCostsEvaluatorT7C3 : public CostsEvaluator<TSLatVec, MapData> {
    public   : TestCostsEvaluatorT7C3(std::shared_ptr<MapData>& _mapDataPtr) : CostsEvaluator(_mapDataPtr) {}
    public   : void setCosts() override {
	partialCostsArray_[asInt(CostEvaluatorCostType::H)].resize(1);
	partialCostsArray_[asInt(CostEvaluatorCostType::H)][0] = std::make_unique< Test7CostArray_LinIntWNorm    >();
    }
};

TEST_F ( TrajSimGradeTest, CostFuncsArrayOnTOnBELatPrecalc ) {
    
    trajSimGrade = make_shared<TrajectorySimGrade>( stateSimPtr, std::make_unique<TestCostsEvaluatorT7C1>(dummyMapData) ); 
    trajSimGrade->trajSim()->stateSim()->setDiscrType( RungeKutta::DiscretizationType::HEUN );
    trajSimGrade->setSimMode(TrajectorySimulator::SimMode::PRECALC);

    trajSimGrade->trajSim()->dt() = 0.1; 
    trajSimGrade->evaluateTrajectory();
    shared_ptr< vector<double> > expPreEq = make_shared< vector<double> >();
    expPreEq->resize(trajSimGrade->trajSim()->costsEvaluator_->h.size()); for(size_t i=0;i<expPreEq->size();++i) { expPreEq->at(i) = trajSimGrade->trajSim()->costsEvaluator_->h[i]; }
//     coutCostEvalCosts(trajSimGrade);
}

////////////////////////////////////////////--------------------------------------------------////////////////////////////////////////////

}  // namespace

int main ( int argc, char **argv ) {
    ::testing::InitGoogleTest ( &argc, argv );
    return RUN_ALL_TESTS();
}
