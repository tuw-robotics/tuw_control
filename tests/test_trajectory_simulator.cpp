
#include "gtest/gtest.h"
#include <array>
#include <boost/concept_check.hpp>
#include <tuw_control/platform_models/agv_diff_drive_v_w.hpp>
#include <tuw_control/trajectory_simulator_new/trajectory_simulator.hpp>

using namespace tuw;
using namespace std;

namespace
{
using PFS = ParamFuncsStructure;
using PfCpD = CtrlPtDim;
using PFFeM = FuncEvalMode;
using EaG = EvalArcGuarantee;

// // using StateDiffDriveType = StateDiffDriveVW;
// // template<template<class> class TDiscretizationType> using StateSimDiffDriveType =
// StateSimDiffDriveVW<TDiscretizationType>;
//
// using StateDiffDriveType    = StateWithGradDiffDriveVW;
// template<template<class> class TDiscretizationType> using StateSimDiffDriveType =
// StateWithGradSimDiffDriveVW<TDiscretizationType>;
//
// using StateSimDiffDriveVWHeunType = StateSimDiffDriveType<heun_abc>;
// using StateSimPtr                 = std::shared_ptr<StateSimDiffDriveVWHeunType>;
// using TrajectorySimulatorType     = TrajectorySimulator<double, StateSimDiffDriveVWHeunType>;
// using TrajectorySimulatorSPtr     = std::shared_ptr<TrajectorySimulatorType>;
//
// // using SNV = StateSimTest::StateNmVars;
// // using SCF = StateSimTest::StateCfVars;
// using PFV = StateSimDiffDriveVWHeunType::ParamFuncVars;
//
// // The fixture for testing class Foo.
// class TrajectorySimulatorTest : public ::testing::Test {
// protected:
//     // You can remove any or all of the following functions if its body
//     // is empty.
//
//     TrajectorySimulatorTest() {
//
// 	//construct stateSim object and initalize the parametric functions object
// 	sp = std::make_shared<StateSimDiffDriveVWHeunType>();
// // 	sp->setDiscrType( RungeKutta::DiscretizationType::EULER );
//
// 	vector<PFS> pf( 2 , PFS() );
// 	static constexpr const int paramFuncArcRefIdxAll = 0;
// 	pf[asInt(PFV::V)].ctrlPtsSize = 3; pf[asInt(PFV::V)].ctrlPtsArcRefIdx = paramFuncArcRefIdxAll;
// 	pf[asInt(PFV::W)].ctrlPtsSize = 3; pf[asInt(PFV::W)].ctrlPtsArcRefIdx = paramFuncArcRefIdxAll;
//
// 	pf[asInt(PFV::V)].evalReq[asInt(PFFeM::INT1)] = false; pf[asInt(PFV::V)].evalReq[asInt(PFFeM::INT2)] = false;
// 	pf[asInt(PFV::W)].evalReq[asInt(PFFeM::INT1)] = true;  pf[asInt(PFV::W)].evalReq[asInt(PFFeM::INT2)] = false;
// 	vector<size_t> idxClosedFormV(1,asInt(PFV::V));
//
// 	sp->paramFuncs()->init(pf);
// 	sp->paramFuncsDist()->setDistCfMode(TraveledDistCfMode::V, idxClosedFormV);
//
//
// 	//construct trajectory simulator object
// 	trajSim       = std::make_shared<TrajectorySimulatorType>(sp);
//     }
//
//     virtual ~TrajectorySimulatorTest() {
//     }
//
//     // If the constructor and destructor are not enough for setting up
//     // and cleaning up each test, you can define the following methods:
//
//     virtual void SetUp() {
// 	Test::SetUp();
//         // Code here will be called immediately after the constructor (right
//         // before each test).
//     }
//
//     virtual void TearDown() {
// 	Test::TearDown();
//         // Code here will be called immediately after each test (right
//         // before the destructor).
//     }
//     StateSimPtr sp;
//     TrajectorySimulatorSPtr trajSim;
//
// public:
//     bool checkSimLatticeConsistency () {
//         bool consistent = true;
// // 	for ( size_t i = 0; i < trajSim->simLatticeSize(); ++i ) {
// // 	    cout<<trajSim->simLatticeI(i).arc<<", ";
// // //             for ( size_t j = 0; j < trajSim->simLatticeI(i).statePtr->stateSize(); ++j ) {
// cout<<trajSim->simLatticeI(i).statePtr->data()(j)<<", "; }cout<<endl;
// //         } cout<<endl;
//         for ( size_t i = 1; i < trajSim->simLatticeSize(); ++i ) { if( !(trajSim->simLatticeI(i-1).arc <=
//         trajSim->simLatticeI(i).arc) ) { consistent = false; break; } }
//         return consistent;
//     }
//     size_t sumAllPartStates (TrajectorySimulatorSPtr _tsp) {
//         size_t nrPartStates = 0;
//         for( size_t i = 0; i < _tsp->partLattices_.size(); ++i){ nrPartStates += _tsp->partLattices_[i]->size(); }
//         return nrPartStates;
//     }
//     void checkSimLatticeStatesEqual ( vector <TrajectorySimulatorType::LatticePointType >& _latticeRef/*, vector
//     <TrajectorySimulatorType::LatticePointType >& _latticeTest*/ ) {
// //         for ( size_t i = 0; i < _latticeTest.size(); ++i ) {
// //             for ( size_t j = 0; j < _latticeTest[i].statePtr->stateSize(); ++j ) {
// cout<<_latticeRef[i].statePtr->value(j)<<":"<<_latticeTest[i].statePtr->value(j)<<", "; }cout<<endl;
// //         }cout<<endl;
//         ASSERT_EQ( _latticeRef.size(), trajSim->simLatticeSize() );
//         for ( size_t i = 0; i < trajSim->simLatticeSize(); ++i ) {
//             for ( int j = 0; j < trajSim->simLatticeI(i).statePtr->data().size(); ++j ) { EXPECT_DOUBLE_EQ(
//             _latticeRef[i].statePtr->data()(j), trajSim->simLatticeI(i).statePtr->data()(j) ); }
//         }
//     }
//     void checkPartLatticeStatesEqual ( TrajectorySimulatorType::LatticeVecSPtrVec& _partLatRef,
//     TrajectorySimulatorType::LatticeVecSPtrVec& _partLatTest ) {
//         ASSERT_EQ( _partLatRef.size(), _partLatTest.size() );
//         for ( size_t i = 0; i < _partLatRef.size(); ++i ) {
// 	    ASSERT_EQ( _partLatRef[i]->size(), _partLatTest[i]->size() );
//             for ( size_t j = 0; j < _partLatRef[i]->size(); ++j ) {
// 		EXPECT_DOUBLE_EQ( _partLatRef[i]->at(j).arc, _partLatTest[i]->at(j).arc );
// 		for ( int k = 0; k < _partLatRef[i]->at(j).statePtr->data().size(); ++k ) {
// 		    EXPECT_DOUBLE_EQ( _partLatRef[i]->at(j).statePtr->data()(k), _partLatTest[i]->at(j).statePtr->data()(k) );
// 		}
// 	    }
//         }
//     }
//     size_t expectedDtStates( const double _DeltaT, const double _dt ) {
//         return ( ceil( _DeltaT / _dt ) + 1 );
//     }
// };
//
// ////////////////////////////////////////////--------------------------------------------------////////////////////////////////////////////
//
//
// TEST_F ( TrajectorySimulatorTest, SimPrecalcFrom0Dt ) {
//
//     auto* funcs = trajSim->stateSim()->paramFuncs();
//     //set the parametric functions control points values
//     double initT = 0; size_t funcIdx = 0;
//     funcs->ctrlPtVal(funcIdx, 0, PfCpD::ARC) = 0    +initT;
//     funcs->ctrlPtVal(funcIdx, 1, PfCpD::ARC) = 0.5  +initT;
//     funcs->ctrlPtVal(funcIdx, 2, PfCpD::ARC) = 1    +initT;
//
//     funcIdx = 0;
//     funcs->ctrlPtVal(funcIdx, 0, PfCpD::VAL) =   3.0 * M_PI;
//     funcs->ctrlPtVal(funcIdx, 1, PfCpD::VAL) = - 0.5 * M_PI;
//     funcs->ctrlPtVal(funcIdx, 2, PfCpD::VAL) =   2.0 * M_PI;
//
//     funcIdx = 1;
//     funcs->ctrlPtVal(funcIdx, 0, PfCpD::VAL) =  3.5 * M_PI;
//     funcs->ctrlPtVal(funcIdx, 1, PfCpD::VAL) =  1.5 * M_PI;
//     funcs->ctrlPtVal(funcIdx, 2, PfCpD::VAL) = -4.5 * M_PI;
//
//     funcs->precompute();
//
//     //set lattice types
//     trajSim->dsBase() = -1;//only dt lattice types
//
//     size_t statesBeginEnd  = 2, statesDt, statesDs, statesLattice0; double sumStates = 0; size_t statesSimExp;
//
//     trajSim->dtBase() = 0.1; trajSim->simulateTrajectory(0);
//     statesDt = expectedDtStates( funcs->funcsArcEnd(), trajSim->dt() ); statesDs = 0; statesLattice0 = 0; sumStates =
//     statesBeginEnd + statesDt + statesDs + statesLattice0;
//     EXPECT_EQ( sumStates, sumAllPartStates (trajSim) );
//     statesSimExp = statesDt; EXPECT_EQ( statesSimExp, trajSim->simLatticeSize() );
//     checkSimLatticeConsistency();
//
//     trajSim->dtBase() = 0.12; trajSim->simulateTrajectory(0);
//     statesDt = expectedDtStates( funcs->funcsArcEnd(), trajSim->dt() ); statesDs = 0; statesLattice0 = 0; sumStates =
//     statesBeginEnd + statesDt + statesDs + statesLattice0;
//     EXPECT_EQ( sumStates, sumAllPartStates (trajSim) );
//     statesSimExp = statesDt; EXPECT_EQ( statesSimExp, trajSim->simLatticeSize() );
//     checkSimLatticeConsistency();
//
//     trajSim->dtBase() = 0.25; trajSim->simulateTrajectory(0);
//     statesDt = expectedDtStates( funcs->funcsArcEnd(), trajSim->dt() ); statesDs = 0; statesLattice0 = 0; sumStates =
//     statesBeginEnd + statesDt + statesDs + statesLattice0;
//     EXPECT_EQ( sumStates, sumAllPartStates (trajSim) );
//     statesSimExp = statesDt; EXPECT_EQ( statesSimExp, trajSim->simLatticeSize() );
//     checkSimLatticeConsistency();
// }
//
// TEST_F ( TrajectorySimulatorTest, SimPrecalcFrom0DtDsCtrlPt ) {
//
//     auto* funcs = trajSim->stateSim()->paramFuncs();
//     //set the parametric functions control points values
//     double initT = 0; size_t funcIdx = 0;
//     funcs->ctrlPtVal(funcIdx, 0, PfCpD::ARC) = 0      +initT;
//     funcs->ctrlPtVal(funcIdx, 1, PfCpD::ARC) = 0.557  +initT;
//     funcs->ctrlPtVal(funcIdx, 2, PfCpD::ARC) = 1      +initT;
//
//     funcIdx = 0;
//     funcs->ctrlPtVal(funcIdx, 0, PfCpD::VAL) =   1.0;
//     funcs->ctrlPtVal(funcIdx, 1, PfCpD::VAL) =   1.0;
//     funcs->ctrlPtVal(funcIdx, 2, PfCpD::VAL) =   1.0;
//
//     funcIdx = 1;
//     funcs->ctrlPtVal(funcIdx, 0, PfCpD::VAL) =  3.5 * M_PI;
//     funcs->ctrlPtVal(funcIdx, 1, PfCpD::VAL) =  1.5 * M_PI;
//     funcs->ctrlPtVal(funcIdx, 2, PfCpD::VAL) = -4.5 * M_PI;
//
//     funcs->precompute();
//
//     //equal dt, equal arc dist and control ponints arcs lattices
//     vector< vector<double*> > funcKnotsLattice(1, vector<double*>(funcs->funcsArcSize(0),0) );
//     for(size_t i = 0; i < funcKnotsLattice[0].size();++i){ funcKnotsLattice[0][i] = &funcs->funcsArc(0,i); }
//     trajSim->setUserDefLattice(funcKnotsLattice);
//     size_t statesBeginEnd  = 2, statesDt, statesDs, statesLattice0;  size_t statesSimExp;
//     double sumStates = 0;
//
//     trajSim->dtBase() = 0.1; trajSim->dsBase() = 0.1; trajSim->simulateTrajectory(0);
//     statesDt = expectedDtStates( funcs->funcsArcEnd(), trajSim->dt() ); statesDs = 10+1; statesLattice0 =
//     funcs->funcCtrlPtSize(0); sumStates = statesBeginEnd + statesDt + statesDs + statesLattice0;
//     EXPECT_EQ( sumStates, sumAllPartStates (trajSim) );
// //     for(size_t i = 0; i < trajSim->simLattice().size(); i++){ cout<<trajSim->simLattice()[i].arc<<", ";
// }cout<<endl;
//     statesSimExp = statesDt + /*statesDs - 2 +*/ statesLattice0 - 2; EXPECT_EQ( statesSimExp,
//     trajSim->simLatticeSize() );//ds in same locations as dt
//     checkSimLatticeConsistency();
//
//     trajSim->dtBase() = 0.12; trajSim->dsBase() = 0.12; trajSim->simulateTrajectory(0);
//     statesDt = expectedDtStates( funcs->funcsArcEnd(), trajSim->dt() ); statesDs =  9+1; statesLattice0 =
//     funcs->funcCtrlPtSize(0); sumStates = statesBeginEnd + statesDt + statesDs + statesLattice0;
//     EXPECT_EQ( sumStates, sumAllPartStates (trajSim) );
//     statesSimExp = statesDt + /*statesDs - 2 +*/ statesLattice0 - 2; EXPECT_EQ( statesSimExp,
//     trajSim->simLatticeSize() );//ds in same locations as dt
//     checkSimLatticeConsistency();
//
//     trajSim->dtBase() = 0.25; trajSim->dsBase() = 0.01; trajSim->simulateTrajectory(0);
//     statesDt = expectedDtStates( funcs->funcsArcEnd(), trajSim->dt() ); statesDs =100+1; statesLattice0 =
//     funcs->funcCtrlPtSize(0); sumStates = statesBeginEnd + statesDt + statesDs + statesLattice0;
//     EXPECT_EQ( sumStates, sumAllPartStates (trajSim) );
//     statesSimExp = statesDt + statesDs - 2 - 3 + statesLattice0 - 2; EXPECT_EQ( statesSimExp,
//     trajSim->simLatticeSize() );//3 ds overlay in same locations with dt (.25, .5, .75)
//     checkSimLatticeConsistency();
// }
//
//
// TEST_F ( TrajectorySimulatorTest, SimPrecalcFromNon0DtDsCtrlPt ) {
//
//     auto* funcs = trajSim->stateSim()->paramFuncs();
//     //set the parametric functions control points values
//     double initT = 0; size_t funcIdx = 0;
//     funcs->ctrlPtVal(funcIdx, 0, PfCpD::ARC) = 0      +initT;
//     funcs->ctrlPtVal(funcIdx, 1, PfCpD::ARC) = 0.557  +initT;
//     funcs->ctrlPtVal(funcIdx, 2, PfCpD::ARC) = 1      +initT;
//
//     funcIdx = 0;
//     funcs->ctrlPtVal(funcIdx, 0, PfCpD::VAL) =   1.0;
//     funcs->ctrlPtVal(funcIdx, 1, PfCpD::VAL) =   1.0;
//     funcs->ctrlPtVal(funcIdx, 2, PfCpD::VAL) =   1.0;
//
//     funcIdx = 1;
//     funcs->ctrlPtVal(funcIdx, 0, PfCpD::VAL) =  3.5 * M_PI;
//     funcs->ctrlPtVal(funcIdx, 1, PfCpD::VAL) =  1.5 * M_PI;
//     funcs->ctrlPtVal(funcIdx, 2, PfCpD::VAL) = -4.5 * M_PI;
//
//     funcs->precompute();
//
//     //equal dt, equal arc dist and control ponints arcs lattices
//     vector< vector<double*> > funcKnotsLattice(1, vector<double*>(funcs->funcsArcSize(0),0) );
//     for(size_t i = 0; i < funcKnotsLattice[0].size();++i){ funcKnotsLattice[0][i] = &funcs->funcsArc(0,i); }
//     trajSim->setUserDefLattice(funcKnotsLattice);
//
//     trajSim->dtBase() = 0.13; trajSim->dsBase() = 0.0795; trajSim->simulateTrajectory(0);
//
//     vector<TrajectorySimulatorType::LatticePointType > trajStatesRef(trajSim->simLatticeSize());
//     for(size_t i = 0; i < trajSim->simLatticeSize(); ++i) {
// 	trajStatesRef[i].arc = trajSim->simLatticeI(i).arc;
// 	trajStatesRef[i].latticeType = trajSim->simLatticeI(i).latticeType;
// 	trajStatesRef[i].statePtr->data() = trajSim->simLatticeI(i).statePtr->data();
//     }
//     TrajectorySimulatorType::LatticeVecSPtrVec partLatRef; partLatRef.resize(trajSim->partLattices_.size() );
//     for(auto& latRefI : partLatRef) { latRefI = make_shared<TrajectorySimulatorType::LatticeVec>();}
//     for(size_t i = 0; i < trajSim->partLattices_.size(); ++i){
// 	partLatRef[i]->resize(trajSim->partLattices_[i]->size() );
// 	for(size_t j = 0; j < trajSim->partLattices_[i]->size(); ++j){
// 	    partLatRef[i]->at(j).statePtr = std::make_shared<TrajectorySimulatorType::StateType>();
// 	    partLatRef[i]->at(j).arc = trajSim->partLattices_[i]->at(j).arc;
// 	    partLatRef[i]->at(j).statePtr->data() = trajSim->partLattices_[i]->at(j).statePtr->data();
// 	}
//     }
//     checkSimLatticeConsistency();
//
//     trajSim->simulateTrajectory(0.10); checkSimLatticeConsistency();checkSimLatticeStatesEqual( trajStatesRef );
//     checkPartLatticeStatesEqual( partLatRef, trajSim->partLattices_ );
//     trajSim->simulateTrajectory(0.34); checkSimLatticeConsistency();checkSimLatticeStatesEqual( trajStatesRef );
//     checkPartLatticeStatesEqual( partLatRef, trajSim->partLattices_ );
//     trajSim->simulateTrajectory(0.79); checkSimLatticeConsistency();checkSimLatticeStatesEqual( trajStatesRef );
//     checkPartLatticeStatesEqual( partLatRef, trajSim->partLattices_ );
//     trajSim->simulateTrajectory(1.00); checkSimLatticeConsistency();checkSimLatticeStatesEqual( trajStatesRef );
//     checkPartLatticeStatesEqual( partLatRef, trajSim->partLattices_ );
//     trajSim->simulateTrajectory(0.54); checkSimLatticeConsistency();checkSimLatticeStatesEqual( trajStatesRef );
//     checkPartLatticeStatesEqual( partLatRef, trajSim->partLattices_ );
//     trajSim->simulateTrajectory(1.50); checkSimLatticeConsistency();checkSimLatticeStatesEqual( trajStatesRef );
//     checkPartLatticeStatesEqual( partLatRef, trajSim->partLattices_ );
// }
//
//
// ////////////////////////////////////////////--------------------------------------------------////////////////////////////////////////////
//
//
}  // namespace
//
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
