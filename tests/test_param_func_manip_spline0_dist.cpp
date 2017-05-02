
#include "gtest/gtest.h"
#include <array>
#include <boost/concept_check.hpp>
#include <tuw_control/param_func_new/param_func_spline/param_func_spline0_dist.hpp>

using namespace tuw;
using namespace std;

namespace {


using ParamFuncsExtType     = ParamFuncsSpline0Dist<double,-1,-1>;
using ParamFuncsType        = ParamFuncsBaseVirt<double>;
using ParamFuncsSPtr        = std::shared_ptr<ParamFuncsBaseVirt<double>>;
using ParamFuncsExtTypePtr  = std::shared_ptr<ParamFuncsExtType>;


// The fixture for testing class Foo.
class ParamFuncManipSplineDistTest : public ::testing::Test {
protected:
    // You can remove any or all of the following functions if its body
    // is empty.

    ParamFuncManipSplineDistTest() {
        funcs = std::make_shared<ParamFuncsExtType>();
    }

    virtual ~ParamFuncManipSplineDistTest() {
    }

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
    ParamFuncsSPtr funcs;
};

////////////////////////////////////////////--------------------------------------------------////////////////////////////////////////////

TEST_F ( ParamFuncManipSplineDistTest, Initialization0 ) {
    using PFS = ParamFuncsStructure; using PfCpD = CtrlPtDim; using FeM = FuncEvalMode; size_t funcIdx;
    vector<PFS> pf(3,PFS());
    pf[0].ctrlPtsSize = 5; pf[0].ctrlPtsArcRefIdx = 0; 
    pf[1].ctrlPtsSize = 5; pf[1].ctrlPtsArcRefIdx = 0; 
    pf[2].ctrlPtsSize = 8; pf[2].ctrlPtsArcRefIdx = 1; 
    
    pf[0].evalReq[(size_t)FeM::DIFF1] = false; pf[0].evalReq[(size_t)FeM::DIFF2] = false; pf[0].evalReq[(size_t)FeM::INT1] = false; pf[0].evalReq[(size_t)FeM::INT2] = false;
    pf[1].evalReq[(size_t)FeM::DIFF1] = false; pf[1].evalReq[(size_t)FeM::DIFF2] = false; pf[1].evalReq[(size_t)FeM::INT1] = false; pf[1].evalReq[(size_t)FeM::INT2] = false;
    pf[2].evalReq[(size_t)FeM::DIFF1] = false; pf[2].evalReq[(size_t)FeM::DIFF2] = false; pf[2].evalReq[(size_t)FeM::INT1] = false; pf[2].evalReq[(size_t)FeM::INT2] = false;
    funcs->init( pf );
    
    funcIdx = 0; for ( size_t j = 0; j < funcs->funcCtrlPtSize(funcIdx); j++ ) { funcs->ctrlPtVal(funcIdx, j, PfCpD::VAL) = 10-2*j; funcs->ctrlPtVal(funcIdx, j, PfCpD::ARC) =      j; }
    funcIdx = 1; for ( size_t j = 0; j < funcs->funcCtrlPtSize(funcIdx); j++ ) { funcs->ctrlPtVal(funcIdx, j, PfCpD::VAL) =      j; funcs->ctrlPtVal(funcIdx, j, PfCpD::ARC) =    2*j; }
    funcIdx = 2; for ( size_t j = 1; j < funcs->funcCtrlPtSize(funcIdx); j++ ) { funcs->ctrlPtVal(funcIdx, j, PfCpD::VAL) = -2+7*j; funcs->ctrlPtVal(funcIdx, j, PfCpD::ARC) = 33+3*j; }
    
    funcIdx = 2; funcs->ctrlPtVal(funcIdx, 0, PfCpD::ARC) = 666; funcs->ctrlPtVal(funcIdx, funcs->funcCtrlPtSize(funcIdx)-1, PfCpD::ARC) = 555;
    funcIdx = 1; funcs->ctrlPtVal(funcIdx, 0, PfCpD::ARC) = 123; 
    
    funcs->precompute();
    
    size_t jj;
    funcIdx = 1; jj = 0; EXPECT_EQ(123, funcs->ctrlPtVal(funcIdx,jj,PfCpD::ARC) ); jj = funcs->funcCtrlPtSize(funcIdx)-1; EXPECT_EQ(555, funcs->ctrlPtVal(funcIdx,jj,PfCpD::ARC) );
    funcIdx = 2; jj = 0; EXPECT_EQ(123, funcs->ctrlPtVal(funcIdx,jj,PfCpD::ARC) ); jj = funcs->funcCtrlPtSize(funcIdx)-1; EXPECT_EQ(555, funcs->ctrlPtVal(funcIdx,jj,PfCpD::ARC) );

//     funcIdx = 0; for(size_t j = 1; j < funcs->funcCtrlPtSize(funcIdx)-1; j++){ EXPECT_EQ(10-2*j, funcs->ctrlPtVal(funcIdx,j,PfCpD::VAL) ); EXPECT_EQ(   2*j, funcs->ctrlPtVal(funcIdx,j,PfCpD::ARC) ); }
//     funcIdx = 1; for(size_t j = 1; j < funcs->funcCtrlPtSize(funcIdx)-1; j++){ EXPECT_EQ(     j, funcs->ctrlPtVal(funcIdx,j,PfCpD::VAL) ); EXPECT_EQ(   2*j, funcs->ctrlPtVal(funcIdx,j,PfCpD::ARC) ); }
//     funcIdx = 2; for(size_t j = 1; j < funcs->funcCtrlPtSize(funcIdx)-1; j++){ EXPECT_EQ(-2+7*j, funcs->ctrlPtVal(funcIdx,j,PfCpD::VAL) ); EXPECT_EQ(33+3*j, funcs->ctrlPtVal(funcIdx,j,PfCpD::ARC) ); }
}

TEST_F ( ParamFuncManipSplineDistTest, Initialization0FuncStatic ) {
    
    funcs = std::make_shared< ParamFuncsSpline0Dist<double,3,-1> >();
    
    using PFS = ParamFuncsStructure; using PfCpD = CtrlPtDim; using FeM = FuncEvalMode; size_t funcIdx;
    vector<PFS> pf(3,PFS());
    pf[0].ctrlPtsSize = 5; pf[0].ctrlPtsArcRefIdx = 0; 
    pf[1].ctrlPtsSize = 5; pf[1].ctrlPtsArcRefIdx = 0; 
    pf[2].ctrlPtsSize = 8; pf[2].ctrlPtsArcRefIdx = 1; 
    
    pf[0].evalReq[(size_t)FeM::DIFF1] = false; pf[0].evalReq[(size_t)FeM::DIFF2] = false; pf[0].evalReq[(size_t)FeM::INT1] = false; pf[0].evalReq[(size_t)FeM::INT2] = false;
    pf[1].evalReq[(size_t)FeM::DIFF1] = false; pf[1].evalReq[(size_t)FeM::DIFF2] = false; pf[1].evalReq[(size_t)FeM::INT1] = false; pf[1].evalReq[(size_t)FeM::INT2] = false;
    pf[2].evalReq[(size_t)FeM::DIFF1] = false; pf[2].evalReq[(size_t)FeM::DIFF2] = false; pf[2].evalReq[(size_t)FeM::INT1] = false; pf[2].evalReq[(size_t)FeM::INT2] = false;
    funcs->init( pf );
    
    funcIdx = 0; for ( size_t j = 0; j < funcs->funcCtrlPtSize(funcIdx); j++ ) { funcs->ctrlPtVal(funcIdx, j, PfCpD::VAL) = 10-2*j; funcs->ctrlPtVal(funcIdx, j, PfCpD::ARC) =      j; }
    funcIdx = 1; for ( size_t j = 0; j < funcs->funcCtrlPtSize(funcIdx); j++ ) { funcs->ctrlPtVal(funcIdx, j, PfCpD::VAL) =      j; funcs->ctrlPtVal(funcIdx, j, PfCpD::ARC) =    2*j; }
    funcIdx = 2; for ( size_t j = 1; j < funcs->funcCtrlPtSize(funcIdx); j++ ) { funcs->ctrlPtVal(funcIdx, j, PfCpD::VAL) = -2+7*j; funcs->ctrlPtVal(funcIdx, j, PfCpD::ARC) = 33+3*j; }
    
    funcIdx = 2; funcs->ctrlPtVal(funcIdx, 0, PfCpD::ARC) = 666; funcs->ctrlPtVal(funcIdx, funcs->funcCtrlPtSize(funcIdx)-1, PfCpD::ARC) = 555;
    funcIdx = 1; funcs->ctrlPtVal(funcIdx, 0, PfCpD::ARC) = 123; 
    
    funcs->precompute();
    
    size_t jj;
    funcIdx = 1; jj = 0; EXPECT_EQ(123, funcs->ctrlPtVal(funcIdx,jj,PfCpD::ARC) ); jj = funcs->funcCtrlPtSize(funcIdx)-1; EXPECT_EQ(555, funcs->ctrlPtVal(funcIdx,jj,PfCpD::ARC) );
    funcIdx = 2; jj = 0; EXPECT_EQ(123, funcs->ctrlPtVal(funcIdx,jj,PfCpD::ARC) ); jj = funcs->funcCtrlPtSize(funcIdx)-1; EXPECT_EQ(555, funcs->ctrlPtVal(funcIdx,jj,PfCpD::ARC) );

//     funcIdx = 0; for(size_t j = 1; j < funcs->funcCtrlPtSize(funcIdx)-1; j++){ EXPECT_EQ(10-2*j, funcs->ctrlPtVal(funcIdx,j,PfCpD::VAL) ); EXPECT_EQ(   2*j, funcs->ctrlPtVal(funcIdx,j,PfCpD::ARC) ); }
//     funcIdx = 1; for(size_t j = 1; j < funcs->funcCtrlPtSize(funcIdx)-1; j++){ EXPECT_EQ(     j, funcs->ctrlPtVal(funcIdx,j,PfCpD::VAL) ); EXPECT_EQ(   2*j, funcs->ctrlPtVal(funcIdx,j,PfCpD::ARC) ); }
//     funcIdx = 2; for(size_t j = 1; j < funcs->funcCtrlPtSize(funcIdx)-1; j++){ EXPECT_EQ(-2+7*j, funcs->ctrlPtVal(funcIdx,j,PfCpD::VAL) ); EXPECT_EQ(33+3*j, funcs->ctrlPtVal(funcIdx,j,PfCpD::ARC) ); }
}

TEST_F ( ParamFuncManipSplineDistTest, Initialization0FuncAllStatic ) {
    
    funcs = std::make_shared< ParamFuncsSpline0Dist<double,3,2> >();
    
    using PFS = ParamFuncsStructure; using PfCpD = CtrlPtDim; using FeM = FuncEvalMode; size_t funcIdx;
    vector<PFS> pf(3,PFS());
    pf[0].ctrlPtsSize = 5; pf[0].ctrlPtsArcRefIdx = 0; 
    pf[1].ctrlPtsSize = 5; pf[1].ctrlPtsArcRefIdx = 0; 
    pf[2].ctrlPtsSize = 8; pf[2].ctrlPtsArcRefIdx = 1; 
    
    pf[0].evalReq[(size_t)FeM::DIFF1] = false; pf[0].evalReq[(size_t)FeM::DIFF2] = false; pf[0].evalReq[(size_t)FeM::INT1] = false; pf[0].evalReq[(size_t)FeM::INT2] = false;
    pf[1].evalReq[(size_t)FeM::DIFF1] = false; pf[1].evalReq[(size_t)FeM::DIFF2] = false; pf[1].evalReq[(size_t)FeM::INT1] = false; pf[1].evalReq[(size_t)FeM::INT2] = false;
    pf[2].evalReq[(size_t)FeM::DIFF1] = false; pf[2].evalReq[(size_t)FeM::DIFF2] = false; pf[2].evalReq[(size_t)FeM::INT1] = false; pf[2].evalReq[(size_t)FeM::INT2] = false;
    funcs->init( pf );
    
    funcIdx = 0; for ( size_t j = 0; j < funcs->funcCtrlPtSize(funcIdx); j++ ) { funcs->ctrlPtVal(funcIdx, j, PfCpD::VAL) = 10-2*j; funcs->ctrlPtVal(funcIdx, j, PfCpD::ARC) =      j; }
    funcIdx = 1; for ( size_t j = 0; j < funcs->funcCtrlPtSize(funcIdx); j++ ) { funcs->ctrlPtVal(funcIdx, j, PfCpD::VAL) =      j; funcs->ctrlPtVal(funcIdx, j, PfCpD::ARC) =    2*j; }
    funcIdx = 2; for ( size_t j = 1; j < funcs->funcCtrlPtSize(funcIdx); j++ ) { funcs->ctrlPtVal(funcIdx, j, PfCpD::VAL) = -2+7*j; funcs->ctrlPtVal(funcIdx, j, PfCpD::ARC) = 33+3*j; }
    
    funcIdx = 2; funcs->ctrlPtVal(funcIdx, 0, PfCpD::ARC) = 666; funcs->ctrlPtVal(funcIdx, funcs->funcCtrlPtSize(funcIdx)-1, PfCpD::ARC) = 555;
    funcIdx = 1; funcs->ctrlPtVal(funcIdx, 0, PfCpD::ARC) = 123; 
    
    funcs->precompute();
    
    size_t jj;
    funcIdx = 1; jj = 0; EXPECT_EQ(123, funcs->ctrlPtVal(funcIdx,jj,PfCpD::ARC) ); jj = funcs->funcCtrlPtSize(funcIdx)-1; EXPECT_EQ(555, funcs->ctrlPtVal(funcIdx,jj,PfCpD::ARC) );
    funcIdx = 2; jj = 0; EXPECT_EQ(123, funcs->ctrlPtVal(funcIdx,jj,PfCpD::ARC) ); jj = funcs->funcCtrlPtSize(funcIdx)-1; EXPECT_EQ(555, funcs->ctrlPtVal(funcIdx,jj,PfCpD::ARC) );

//     funcIdx = 0; for(size_t j = 1; j < funcs->funcCtrlPtSize(funcIdx)-1; j++){ EXPECT_EQ(10-2*j, funcs->ctrlPtVal(funcIdx,j,PfCpD::VAL) ); EXPECT_EQ(   2*j, funcs->ctrlPtVal(funcIdx,j,PfCpD::ARC) ); }
//     funcIdx = 1; for(size_t j = 1; j < funcs->funcCtrlPtSize(funcIdx)-1; j++){ EXPECT_EQ(     j, funcs->ctrlPtVal(funcIdx,j,PfCpD::VAL) ); EXPECT_EQ(   2*j, funcs->ctrlPtVal(funcIdx,j,PfCpD::ARC) ); }
//     funcIdx = 2; for(size_t j = 1; j < funcs->funcCtrlPtSize(funcIdx)-1; j++){ EXPECT_EQ(-2+7*j, funcs->ctrlPtVal(funcIdx,j,PfCpD::VAL) ); EXPECT_EQ(33+3*j, funcs->ctrlPtVal(funcIdx,j,PfCpD::ARC) ); }
}

namespace EvalArcFuncValFuncValDiff1 {
    using PFS = ParamFuncsStructure; using PfCpD = CtrlPtDim; using FeM = FuncEvalMode; using EaG = EvalArcGuarantee;
    
    void initValDiff( ParamFuncsSPtr funcs, double initT ) {
	size_t funcIdx = 0; vector<PFS> pf( 4 , PFS() );
	pf[0].ctrlPtsSize = 4; pf[0].ctrlPtsArcRefIdx = 0; 
	pf[1].ctrlPtsSize = 4; pf[1].ctrlPtsArcRefIdx = 0;
	pf[2].ctrlPtsSize = 7; pf[2].ctrlPtsArcRefIdx = 1;
	pf[3].ctrlPtsSize = 9; pf[3].ctrlPtsArcRefIdx = 2;

	pf[0].evalReq[(size_t)FeM::DIFF1] = false; pf[0].evalReq[(size_t)FeM::DIFF2] = false; pf[0].evalReq[(size_t)FeM::INT1] = false; pf[0].evalReq[(size_t)FeM::INT2] = false;
	funcs->init( pf );

	funcIdx = 0;
	funcs->ctrlPtVal(funcIdx, 0, PfCpD::VAL) = 0; funcs->ctrlPtVal(funcIdx, 0, PfCpD::ARC) = 0+initT;
	funcs->ctrlPtVal(funcIdx, 1, PfCpD::VAL) = 2; funcs->ctrlPtVal(funcIdx, 1, PfCpD::ARC) = 2+initT;
	funcs->ctrlPtVal(funcIdx, 2, PfCpD::VAL) = 1; funcs->ctrlPtVal(funcIdx, 2, PfCpD::ARC) = 3+initT;
	funcs->ctrlPtVal(funcIdx, 3, PfCpD::VAL) = 5; funcs->ctrlPtVal(funcIdx, 3, PfCpD::ARC) = 4+initT;

	funcIdx = 1;
	funcs->ctrlPtVal(funcIdx, 0, PfCpD::VAL) = 1; 
	funcs->ctrlPtVal(funcIdx, 1, PfCpD::VAL) = 3;
	funcs->ctrlPtVal(funcIdx, 2, PfCpD::VAL) = 2;
	funcs->ctrlPtVal(funcIdx, 3, PfCpD::VAL) = 6;
    
	funcs->precompute();
    }
    
    void testValDiff(ParamFuncsSPtr funcs, double initT, size_t funcIdx, double funcShift) {
	funcs->setEvalArc(       0+initT, EaG::NONE); EXPECT_DOUBLE_EQ(     0  +funcShift, funcs->computeFuncVal( funcIdx ) ); 
	funcs->setEvalArc(       1+initT, EaG::NONE); EXPECT_DOUBLE_EQ(     1  +funcShift, funcs->computeFuncVal( funcIdx ) );
	funcs->setEvalArc(     1.5+initT, EaG::NONE); EXPECT_DOUBLE_EQ(     1.5+funcShift, funcs->computeFuncVal( funcIdx ) ); 
	funcs->setEvalArc(2.0-1e-5+initT, EaG::NONE); EXPECT_DOUBLE_EQ(2.0-1e-5+funcShift, funcs->computeFuncVal( funcIdx ) ); 
	funcs->setEvalArc(     2.0+initT, EaG::NONE); EXPECT_DOUBLE_EQ(     2.0+funcShift, funcs->computeFuncVal( funcIdx ) ); 
	funcs->setEvalArc(2.0+1e-5+initT, EaG::NONE); EXPECT_DOUBLE_EQ(2.0-1e-5+funcShift, funcs->computeFuncVal( funcIdx ) ); 
	funcs->setEvalArc(     2.5+initT, EaG::NONE); EXPECT_DOUBLE_EQ(     1.5+funcShift, funcs->computeFuncVal( funcIdx ) );
	funcs->setEvalArc(     3.5+initT, EaG::NONE); EXPECT_DOUBLE_EQ(       3+funcShift, funcs->computeFuncVal( funcIdx ) );

	funcs->setEvalArc(       0+initT, EaG::NONE);
	funcs->setEvalArc(       0+initT, EaG::NEAR_LAST); EXPECT_DOUBLE_EQ(     0  +funcShift, funcs->computeFuncVal( funcIdx ) ); 
	funcs->setEvalArc(       1+initT, EaG::NEAR_LAST); EXPECT_DOUBLE_EQ(     1  +funcShift, funcs->computeFuncVal( funcIdx ) ); 
	funcs->setEvalArc(     1.5+initT, EaG::NEAR_LAST); EXPECT_DOUBLE_EQ(     1.5+funcShift, funcs->computeFuncVal( funcIdx ) ); 
	funcs->setEvalArc(2.0-1e-5+initT, EaG::NEAR_LAST); EXPECT_DOUBLE_EQ(2.0-1e-5+funcShift, funcs->computeFuncVal( funcIdx ) ); 
	funcs->setEvalArc(     2.0+initT, EaG::NEAR_LAST); EXPECT_DOUBLE_EQ(     2.0+funcShift, funcs->computeFuncVal( funcIdx ) ); 
	funcs->setEvalArc(2.0+1e-5+initT, EaG::NEAR_LAST); EXPECT_DOUBLE_EQ(2.0-1e-5+funcShift, funcs->computeFuncVal( funcIdx ) ); 
	funcs->setEvalArc(     2.5+initT, EaG::NEAR_LAST); EXPECT_DOUBLE_EQ(     1.5+funcShift, funcs->computeFuncVal( funcIdx ) );
	funcs->setEvalArc(     3.5+initT, EaG::NEAR_LAST); EXPECT_DOUBLE_EQ(       3+funcShift, funcs->computeFuncVal( funcIdx ) );

	funcs->setEvalArc(       4+initT, EaG::NONE);
	funcs->setEvalArc(     3.5+initT, EaG::NEAR_LAST); EXPECT_DOUBLE_EQ(       3+funcShift, funcs->computeFuncVal( funcIdx ) );
	funcs->setEvalArc(     2.5+initT, EaG::NEAR_LAST); EXPECT_DOUBLE_EQ(     1.5+funcShift, funcs->computeFuncVal( funcIdx ) );
	funcs->setEvalArc(2.0+1e-5+initT, EaG::NEAR_LAST); EXPECT_DOUBLE_EQ(2.0-1e-5+funcShift, funcs->computeFuncVal( funcIdx ) );
	funcs->setEvalArc(     2.0+initT, EaG::NEAR_LAST); EXPECT_DOUBLE_EQ(     2.0+funcShift, funcs->computeFuncVal( funcIdx ) ); 
	funcs->setEvalArc(2.0-1e-5+initT, EaG::NEAR_LAST); EXPECT_DOUBLE_EQ(2.0-1e-5+funcShift, funcs->computeFuncVal( funcIdx ) ); 
	funcs->setEvalArc(     1.5+initT, EaG::NEAR_LAST); EXPECT_DOUBLE_EQ(     1.5+funcShift, funcs->computeFuncVal( funcIdx ) ); 
	funcs->setEvalArc(       1+initT, EaG::NEAR_LAST); EXPECT_DOUBLE_EQ(     1  +funcShift, funcs->computeFuncVal( funcIdx ) ); 
	funcs->setEvalArc(       0+initT, EaG::NEAR_LAST); EXPECT_DOUBLE_EQ(     0  +funcShift, funcs->computeFuncVal( funcIdx ) ); 
    }
}


TEST_F ( ParamFuncManipSplineDistTest, evalArcFuncValFuncValDiff1 ) {
    size_t funcIdx = 0;
    double initT = 2;
    double funcShift;
    
    EvalArcFuncValFuncValDiff1::initValDiff(funcs, initT);
    
    funcIdx = 0; EvalArcFuncValFuncValDiff1::testValDiff(funcs, initT, funcIdx, funcShift = 0);
    funcIdx = 1; EvalArcFuncValFuncValDiff1::testValDiff(funcs, initT, funcIdx, funcShift = 1);
}

TEST_F ( ParamFuncManipSplineDistTest, funcValInt1funcValInt2 ) {
    using PFS = ParamFuncsStructure; using PfCpD = CtrlPtDim; using FeM = FuncEvalMode; using EaG = EvalArcGuarantee;
    size_t funcIdx = 0; vector<PFS> pf( 4 , PFS() );
    pf[0].ctrlPtsSize = 4; pf[0].ctrlPtsArcRefIdx = 0; 
    pf[1].ctrlPtsSize = 4; pf[1].ctrlPtsArcRefIdx = 0;
    pf[2].ctrlPtsSize = 7; pf[2].ctrlPtsArcRefIdx = 1;
    pf[3].ctrlPtsSize = 9; pf[3].ctrlPtsArcRefIdx = 2;
    
    pf[0].evalReq[(size_t)FeM::INT1] = true; pf[0].evalReq[(size_t)FeM::INT2] = true;
    funcs->init( pf );
    
    double initT = 2;
    funcs->ctrlPtVal(funcIdx, 0, PfCpD::VAL) = 0; funcs->ctrlPtVal(funcIdx, 0, PfCpD::ARC) = 0+initT;
    funcs->ctrlPtVal(funcIdx, 1, PfCpD::VAL) = 2; funcs->ctrlPtVal(funcIdx, 1, PfCpD::ARC) = 2+initT;
    funcs->ctrlPtVal(funcIdx, 2, PfCpD::VAL) = 1; funcs->ctrlPtVal(funcIdx, 2, PfCpD::ARC) = 3+initT;
    funcs->ctrlPtVal(funcIdx, 3, PfCpD::VAL) = 5; funcs->ctrlPtVal(funcIdx, 3, PfCpD::ARC) = 4+initT;
    
    
    
    funcs->precompute();
    
    funcs->setEvalArc(       0+initT, EaG::NONE); EXPECT_DOUBLE_EQ(     0  , funcs->computeFuncInt1( 0 ) ); 
    funcs->setEvalArc(       1+initT, EaG::NONE); EXPECT_DOUBLE_EQ(     0.5, funcs->computeFuncInt1( 0 ) ); 
    funcs->setEvalArc(     2.5+initT, EaG::NONE); EXPECT_DOUBLE_EQ(   2.875, funcs->computeFuncInt1( 0 ) );
    funcs->setEvalArc(     3.5+initT, EaG::NONE); EXPECT_DOUBLE_EQ(     4.5, funcs->computeFuncInt1( 0 ) );
    funcs->setEvalArc(     4.0+initT, EaG::NONE); EXPECT_DOUBLE_EQ(     6.5, funcs->computeFuncInt1( 0 ) );

    funcs->setEvalArc(       0+initT, EaG::NONE); EXPECT_DOUBLE_EQ(       0, funcs->computeFuncInt2( 0 ) ); 
    funcs->setEvalArc(       1+initT, EaG::NONE); EXPECT_DOUBLE_EQ(   1./6., funcs->computeFuncInt2( 0 ) ); 
    funcs->setEvalArc(     2.5+initT, EaG::NONE); EXPECT_DOUBLE_EQ(  1.5625, funcs->computeFuncInt2( 0 ) );
    funcs->setEvalArc(     3.5+initT, EaG::NONE); EXPECT_DOUBLE_EQ(   2.375, funcs->computeFuncInt2( 0 ) );
    funcs->setEvalArc(     4.0+initT, EaG::NONE); EXPECT_DOUBLE_EQ(  10./3., funcs->computeFuncInt2( 0 ) );
    
    funcs->setEvalArc(     2+initT, EaG::NONE      ); EXPECT_DOUBLE_EQ( 1, funcs->computeFuncDiff1( 0 ) );
    funcs->setEvalArc(     2+initT, EaG::NEAR_LAST); EXPECT_DOUBLE_EQ( 1, funcs->computeFuncDiff1( 0 ) );
    
    ParamFuncsExtType funcs2( dynamic_cast<ParamFuncsExtType&>(*funcs) );
    
    funcs2.precompute();
    
    funcs2.setEvalArc(       0+initT, EaG::NONE); EXPECT_DOUBLE_EQ(     0  , funcs2.computeFuncInt1( 0 ) ); 
    funcs2.setEvalArc(       1+initT, EaG::NONE); EXPECT_DOUBLE_EQ(     0.5, funcs2.computeFuncInt1( 0 ) ); 
    funcs2.setEvalArc(     2.5+initT, EaG::NONE); EXPECT_DOUBLE_EQ(   2.875, funcs2.computeFuncInt1( 0 ) );
    funcs2.setEvalArc(     3.5+initT, EaG::NONE); EXPECT_DOUBLE_EQ(     4.5, funcs2.computeFuncInt1( 0 ) );
    funcs2.setEvalArc(     4.0+initT, EaG::NONE); EXPECT_DOUBLE_EQ(     6.5, funcs2.computeFuncInt1( 0 ) );

    funcs2.setEvalArc(       0+initT, EaG::NONE); EXPECT_DOUBLE_EQ(       0, funcs2.computeFuncInt2( 0 ) ); 
    funcs2.setEvalArc(       1+initT, EaG::NONE); EXPECT_DOUBLE_EQ(   1./6., funcs2.computeFuncInt2( 0 ) ); 
    funcs2.setEvalArc(     2.5+initT, EaG::NONE); EXPECT_DOUBLE_EQ(  1.5625, funcs2.computeFuncInt2( 0 ) );
    funcs2.setEvalArc(     3.5+initT, EaG::NONE); EXPECT_DOUBLE_EQ(   2.375, funcs2.computeFuncInt2( 0 ) );
    funcs2.setEvalArc(     4.0+initT, EaG::NONE); EXPECT_DOUBLE_EQ(  10./3., funcs2.computeFuncInt2( 0 ) );
    
    funcs2.setEvalArc(     2+initT, EaG::NONE      ); EXPECT_DOUBLE_EQ( 1, funcs2.computeFuncDiff1( 0 ) );
    funcs2.setEvalArc(     2+initT, EaG::NEAR_LAST); EXPECT_DOUBLE_EQ( 1, funcs2.computeFuncDiff1( 0 ) );
    
    ParamFuncsExtType funcs3;
    funcs3 = dynamic_cast<ParamFuncsExtType&>(*funcs);
    
    funcs3.precompute();
    
    funcs3.setEvalArc(       0+initT, EaG::NONE); EXPECT_DOUBLE_EQ(     0  , funcs3.computeFuncInt1( 0 ) ); 
    funcs3.setEvalArc(       1+initT, EaG::NONE); EXPECT_DOUBLE_EQ(     0.5, funcs3.computeFuncInt1( 0 ) ); 
    funcs3.setEvalArc(     2.5+initT, EaG::NONE); EXPECT_DOUBLE_EQ(   2.875, funcs3.computeFuncInt1( 0 ) );
    funcs3.setEvalArc(     3.5+initT, EaG::NONE); EXPECT_DOUBLE_EQ(     4.5, funcs3.computeFuncInt1( 0 ) );
    funcs3.setEvalArc(     4.0+initT, EaG::NONE); EXPECT_DOUBLE_EQ(     6.5, funcs3.computeFuncInt1( 0 ) );

    funcs3.setEvalArc(       0+initT, EaG::NONE); EXPECT_DOUBLE_EQ(       0, funcs3.computeFuncInt2( 0 ) ); 
    funcs3.setEvalArc(       1+initT, EaG::NONE); EXPECT_DOUBLE_EQ(   1./6., funcs3.computeFuncInt2( 0 ) ); 
    funcs3.setEvalArc(     2.5+initT, EaG::NONE); EXPECT_DOUBLE_EQ(  1.5625, funcs3.computeFuncInt2( 0 ) );
    funcs3.setEvalArc(     3.5+initT, EaG::NONE); EXPECT_DOUBLE_EQ(   2.375, funcs3.computeFuncInt2( 0 ) );
    funcs3.setEvalArc(     4.0+initT, EaG::NONE); EXPECT_DOUBLE_EQ(  10./3., funcs3.computeFuncInt2( 0 ) );
    
    funcs3.setEvalArc(     2+initT, EaG::NONE      ); EXPECT_DOUBLE_EQ( 1, funcs3.computeFuncDiff1( 0 ) );
    funcs3.setEvalArc(     2+initT, EaG::NEAR_LAST); EXPECT_DOUBLE_EQ( 1, funcs3.computeFuncDiff1( 0 ) );
}



////////////////////////////////////////////--------------------------------------------------////////////////////////////////////////////

using pfsd = ParamFuncsExtType;
TEST ( Spline0computeDtDs_V_AV_Test, TPos_VPos_A0 ) {
    double dt = 1, v = 1, av = 0, dsAnsGiven = 1; double dsAns, dtAns;
    dsAns = pfsd::computeDeltaS_V_AV(   dt, v, av); EXPECT_DOUBLE_EQ ( dsAnsGiven, dsAns );
    dtAns = pfsd::computeDeltaT_V_AV(dsAns, v, av); EXPECT_DOUBLE_EQ (         dt, dtAns );
}
TEST ( Spline0computeDtDs_V_AV_Test, TPos_VNeg_A0 ) {
    double dt = 1, v = -1, av = 0, dsAnsGiven = 1; double dsAns, dtAns;
    dsAns = pfsd::computeDeltaS_V_AV(   dt, v, av); EXPECT_DOUBLE_EQ ( dsAnsGiven, dsAns );
    dtAns = pfsd::computeDeltaT_V_AV(dsAns, v, av); EXPECT_DOUBLE_EQ (         dt, dtAns );
}
TEST ( Spline0computeDtDs_V_AV_Test, TNeg_VPos_A0 ) {
    double dt = -1, v = +1, av = 0, dsAnsGiven = -1; double dsAns, dtAns;
    dsAns = pfsd::computeDeltaS_V_AV(   dt, v, av); EXPECT_DOUBLE_EQ ( dsAnsGiven, dsAns );
    dtAns = pfsd::computeDeltaT_V_AV(dsAns, v, av); EXPECT_DOUBLE_EQ (         dt, dtAns );
}
TEST ( Spline0computeDtDs_V_AV_Test, TNeg_VNeg_A0 ) {
    double dt = -1, v = -1, av = 0, dsAnsGiven = -1; double dsAns, dtAns;
    dsAns = pfsd::computeDeltaS_V_AV(   dt, v, av); EXPECT_DOUBLE_EQ ( dsAnsGiven, dsAns );
    dtAns = pfsd::computeDeltaT_V_AV(dsAns, v, av); EXPECT_DOUBLE_EQ (         dt, dtAns );
}
////////////////////////////////////////////
TEST ( Spline0computeDtDs_V_AV_Test, TPos_VPos_APos ) {
    double dt = 1, v = 1, av = 1, dsAnsGiven = 1.5; double dsAns, dtAns;
    dsAns = pfsd::computeDeltaS_V_AV(   dt, v, av); EXPECT_DOUBLE_EQ ( dsAnsGiven, dsAns );
    dtAns = pfsd::computeDeltaT_V_AV(dsAns, v, av); EXPECT_DOUBLE_EQ (         dt, dtAns );
}
TEST ( Spline0computeDtDs_V_AV_Test, TPos_VNeg_ANeg ) {
    double dt = 1, v = -1, av = -1, dsAnsGiven = 1.5; double dsAns, dtAns;
    dsAns = pfsd::computeDeltaS_V_AV(   dt, v, av); EXPECT_DOUBLE_EQ ( dsAnsGiven, dsAns );
    dtAns = pfsd::computeDeltaT_V_AV(dsAns, v, av); EXPECT_DOUBLE_EQ (         dt, dtAns );
}
TEST ( Spline0computeDtDs_V_AV_Test, TNeg_VPos_APos ) {
    double dt = -1, v = +1, av = +1, dsAnsGiven = -1.5; double dsAns, dtAns;
    dsAns = pfsd::computeDeltaS_V_AV(   dt, v, av); EXPECT_DOUBLE_EQ ( dsAnsGiven, dsAns );
    dtAns = pfsd::computeDeltaT_V_AV(dsAns, v, av); EXPECT_DOUBLE_EQ (         dt, dtAns );
}
TEST ( Spline0computeDtDs_V_AV_Test, TNeg_VNeg_ANeg ) {
    double dt = -1, v = -1, av = -1, dsAnsGiven = -1.5; double dsAns, dtAns;
    dsAns = pfsd::computeDeltaS_V_AV(   dt, v, av); EXPECT_DOUBLE_EQ ( dsAnsGiven, dsAns );
    dtAns = pfsd::computeDeltaT_V_AV(dsAns, v, av); EXPECT_DOUBLE_EQ (         dt, dtAns );
}
////////////////////////////////////////////
TEST ( Spline0computeDtDs_V_AV_Test, TPos_VPos_ANeg_Above0 ) {
    double dt = 1, v = 1, av = -0.5, dsAnsGiven = 0.75; double dsAns, dtAns;
    dsAns = pfsd::computeDeltaS_V_AV(   dt, v, av); EXPECT_DOUBLE_EQ ( dsAnsGiven, dsAns );
    dtAns = pfsd::computeDeltaT_V_AV(dsAns, v, av); EXPECT_DOUBLE_EQ (         dt, dtAns );
}
TEST ( Spline0computeDtDs_V_AV_Test, TPos_VNeg_APos_Below0 ) {
    double dt = 1, v = -1, av = 0.5, dsAnsGiven = 0.75; double dsAns, dtAns;
    dsAns = pfsd::computeDeltaS_V_AV(   dt, v, av); EXPECT_DOUBLE_EQ ( dsAnsGiven, dsAns );
    dtAns = pfsd::computeDeltaT_V_AV(dsAns, v, av); EXPECT_DOUBLE_EQ (         dt, dtAns );
}
TEST ( Spline0computeDtDs_V_AV_Test, TNeg_VPos_ANeg_Above0 ) {
    double dt = -1, v = 1, av = -0.5, dsAnsGiven = -0.75; double dsAns, dtAns;
    dsAns = pfsd::computeDeltaS_V_AV(   dt, v, av); EXPECT_DOUBLE_EQ ( dsAnsGiven, dsAns );
    dtAns = pfsd::computeDeltaT_V_AV(dsAns, v, av); EXPECT_DOUBLE_EQ (         dt, dtAns );
}
TEST ( Spline0computeDtDs_V_AV_Test, TNeg_VNeg_APos_Below0 ) {
    double dt = -1, v = -1, av = 0.5, dsAnsGiven = -0.75; double dsAns, dtAns;
    dsAns = pfsd::computeDeltaS_V_AV(   dt, v, av); EXPECT_DOUBLE_EQ ( dsAnsGiven, dsAns );
    dtAns = pfsd::computeDeltaT_V_AV(dsAns, v, av); EXPECT_DOUBLE_EQ (         dt, dtAns );
}
////////////////////////////////////////////
TEST ( Spline0computeDtDs_V_AV_Test, TPos_VPos_ANeg_Below0_1 ) {
    double dt = 1, v = 1, av = -1.5, dsAnsGiven = 5./12.; double dsAns, dtAns;
    dsAns = pfsd::computeDeltaS_V_AV(   dt, v, av); EXPECT_DOUBLE_EQ ( dsAnsGiven, dsAns );
    dtAns = pfsd::computeDeltaT_V_AV(dsAns, v, av); EXPECT_DOUBLE_EQ (         dt, dtAns );
}
TEST ( Spline0computeDtDs_V_AV_Test, TPos_VNeg_APos_Above0_1 ) {
    double dt = 1, v = -1, av = +1.5, dsAnsGiven = 5./12.; double dsAns, dtAns;
    dsAns = pfsd::computeDeltaS_V_AV(   dt, v, av); EXPECT_DOUBLE_EQ ( dsAnsGiven, dsAns );
    dtAns = pfsd::computeDeltaT_V_AV(dsAns, v, av); EXPECT_DOUBLE_EQ (         dt, dtAns );
}
TEST ( Spline0computeDtDs_V_AV_Test, TNeg_VPos_ANeg_Below0_1 ) {
    double dt = -1, v = 1, av = -1.5, dsAnsGiven = -5./12.; double dsAns, dtAns;
    dsAns = pfsd::computeDeltaS_V_AV(   dt, v, av); EXPECT_DOUBLE_EQ ( dsAnsGiven, dsAns );
    dtAns = pfsd::computeDeltaT_V_AV(dsAns, v, av); EXPECT_DOUBLE_EQ (         dt, dtAns );
}
TEST ( Spline0computeDtDs_V_AV_Test, TNeg_VNeg_APos_Above0_1 ) {
    double dt = -1, v = -1, av = +1.5, dsAnsGiven = -5./12.; double dsAns, dtAns;
    dsAns = pfsd::computeDeltaS_V_AV(   dt, v, av); EXPECT_DOUBLE_EQ ( dsAnsGiven, dsAns );
    dtAns = pfsd::computeDeltaT_V_AV(dsAns, v, av); EXPECT_DOUBLE_EQ (         dt, dtAns );
}
////////////////////////////////////////////
TEST ( Spline0computeDtDs_V_AV_Test, TPos_VPos_ANeg_Below0 ) {
    double dt = 6, v = 3, av = -3.5; double dsAns, dtAns;
    dsAns = pfsd::computeDeltaS_V_AV(   dt, v, av);
    dtAns = pfsd::computeDeltaT_V_AV(dsAns, v, av); EXPECT_DOUBLE_EQ (         dt, dtAns );
}
TEST ( Spline0computeDtDs_V_AV_Test, TPos_VNeg_APos_Above0 ) {
    double dt = 6, v = -3, av = +3.5; double dsAns, dtAns;
    dsAns = pfsd::computeDeltaS_V_AV(   dt, v, av); 
    dtAns = pfsd::computeDeltaT_V_AV(dsAns, v, av); EXPECT_DOUBLE_EQ (         dt, dtAns );
}
TEST ( Spline0computeDtDs_V_AV_Test, TNeg_VPos_ANeg_Below0 ) {
    double dt = -6, v = 3, av = -3.5; double dsAns, dtAns;
    dsAns = pfsd::computeDeltaS_V_AV(   dt, v, av);
    dtAns = pfsd::computeDeltaT_V_AV(dsAns, v, av); EXPECT_DOUBLE_EQ (         dt, dtAns );
}
TEST ( Spline0computeDtDs_V_AV_Test, TNeg_VNeg_APos_Above0 ) {
    double dt = -6, v = -3, av = +3.5; double dsAns, dtAns;
    dsAns = pfsd::computeDeltaS_V_AV(   dt, v, av); 
    dtAns = pfsd::computeDeltaT_V_AV(dsAns, v, av); EXPECT_DOUBLE_EQ (         dt, dtAns );
}
////////////////////////////////////////////--------------------------------------------------////////////////////////////////////////////

TEST_F ( ParamFuncManipSplineDistTest, distTimeV ) {
    
    ParamFuncsExtTypePtr funcss = dynamic_pointer_cast<ParamFuncsExtType>(funcs);
    
    using PFS = ParamFuncsStructure; using PfCpD = CtrlPtDim; using FeM = FuncEvalMode; using EaG = EvalArcGuarantee;
    size_t funcIdx = 0; vector<PFS> pf( 4 , PFS() );
    pf[0].ctrlPtsSize = 4; pf[0].ctrlPtsArcRefIdx = 0; 
    pf[1].ctrlPtsSize = 4; pf[1].ctrlPtsArcRefIdx = 0;
    pf[2].ctrlPtsSize = 7; pf[2].ctrlPtsArcRefIdx = 1;
    pf[3].ctrlPtsSize = 9; pf[3].ctrlPtsArcRefIdx = 2;
    
    pf[0].evalReq[(size_t)FeM::INT1] = true; pf[0].evalReq[(size_t)FeM::INT2] = true;
    funcss->init( pf );
    vector<size_t> idxCfV(1,0);
    funcss->setDistCfMode(TraveledDistCfMode::V, idxCfV);
    
    double initT = 3;
    funcss->ctrlPtVal(funcIdx, 0, PfCpD::VAL) = -1; funcss->ctrlPtVal(funcIdx, 0, PfCpD::ARC) = 0+initT;
    funcss->ctrlPtVal(funcIdx, 1, PfCpD::VAL) =  1; funcss->ctrlPtVal(funcIdx, 1, PfCpD::ARC) = 2+initT;
    funcss->ctrlPtVal(funcIdx, 2, PfCpD::VAL) = -1; funcss->ctrlPtVal(funcIdx, 2, PfCpD::ARC) = 3+initT;
    funcss->ctrlPtVal(funcIdx, 3, PfCpD::VAL) = -5; funcss->ctrlPtVal(funcIdx, 3, PfCpD::ARC) = 4+initT;
    funcss->precompute();
    
    funcss->setEvalArc(  1+initT, EaG::NONE); EXPECT_DOUBLE_EQ(     0.5, funcss->computeS() ); EXPECT_DOUBLE_EQ(  1+initT, funcss->computeT( 0.5));
    funcss->setEvalArc(  2+initT, EaG::NONE); EXPECT_DOUBLE_EQ(     1  , funcss->computeS() ); EXPECT_DOUBLE_EQ(  2+initT, funcss->computeT(   1));
    funcss->setEvalArc(  3+initT, EaG::NONE); EXPECT_DOUBLE_EQ(     1.5, funcss->computeS() ); EXPECT_DOUBLE_EQ(  3+initT, funcss->computeT( 1.5));
    funcss->setEvalArc(3.5+initT, EaG::NONE); EXPECT_DOUBLE_EQ(     2.5, funcss->computeS() ); EXPECT_DOUBLE_EQ(3.5+initT, funcss->computeT( 2.5));
    funcss->setEvalArc(  4+initT, EaG::NONE); EXPECT_DOUBLE_EQ(     4.5, funcss->computeS() ); EXPECT_DOUBLE_EQ(  4+initT, funcss->computeT( 4.5));
    
    funcss->setEvalArc(  1+initT, EaG::NONE); EXPECT_DOUBLE_EQ(     0.5, funcss->computeS() ); 
    
    EXPECT_DOUBLE_EQ(  1+initT, funcss->computeT( 0.5, EaG::NEAR_LAST));
    EXPECT_DOUBLE_EQ(  2+initT, funcss->computeT(   1, EaG::NEAR_LAST));
    EXPECT_DOUBLE_EQ(  3+initT, funcss->computeT( 1.5, EaG::NEAR_LAST));
    EXPECT_DOUBLE_EQ(3.5+initT, funcss->computeT( 2.5, EaG::NEAR_LAST));
    EXPECT_DOUBLE_EQ(  4+initT, funcss->computeT( 4.5, EaG::NEAR_LAST));
    
    funcss->setEvalArc(  2+initT, EaG::NEAR_LAST); EXPECT_DOUBLE_EQ(     1  , funcss->computeS() );
    funcss->setEvalArc(  3+initT, EaG::NEAR_LAST); EXPECT_DOUBLE_EQ(     1.5, funcss->computeS() ); 
    funcss->setEvalArc(3.5+initT, EaG::NEAR_LAST); EXPECT_DOUBLE_EQ(     2.5, funcss->computeS() );
    funcss->setEvalArc(  4+initT, EaG::NEAR_LAST); EXPECT_DOUBLE_EQ(     4.5, funcss->computeS() );
    
    double evalTempSol;
    for(size_t i = 0; i < 4; i++){
	funcss->setEvalArc(  1+initT, EaG::NONE); evalTempSol = funcss->computeFuncVal( i ); funcss->setEvalDist(0.5, EaG::NONE); EXPECT_DOUBLE_EQ( evalTempSol, funcss->computeFuncVal( i ) );
    }
    for(size_t i = 0; i < 4; i++){
	funcss->setEvalArc(  2+initT, EaG::NONE); evalTempSol = funcss->computeFuncVal( i ); funcss->setEvalDist(  1, EaG::NONE); EXPECT_DOUBLE_EQ( evalTempSol, funcss->computeFuncVal( i ) );
    }
    
    vector<double> sLattice(8,0); 
    sLattice[0] = 0; 
    sLattice[1] = 0.5; 
    sLattice[2] = 1; 
    sLattice[3] = 1.5; 
    sLattice[4] = 2.5; 
    sLattice[5] = 4.5;
    sLattice[6] = 6;
    sLattice[7] = -1;
    vector<double> tLattice(120,0); 
    funcss->computeS2TLattice(sLattice, tLattice);
    
    EXPECT_DOUBLE_EQ( 6, tLattice.size() );
    
    EXPECT_DOUBLE_EQ(  0+initT, tLattice[0] );
    EXPECT_DOUBLE_EQ(  1+initT, tLattice[1] );
    EXPECT_DOUBLE_EQ(  2+initT, tLattice[2] );
    EXPECT_DOUBLE_EQ(  3+initT, tLattice[3] );
    EXPECT_DOUBLE_EQ(3.5+initT, tLattice[4] );
    EXPECT_DOUBLE_EQ(  4+initT, tLattice[5] );
    
    sLattice.resize(4);
    sLattice[0] = 0; 
    sLattice[1] = 0.5; 
    sLattice[2] = 1; 
    sLattice[3] = 0.5; 
    funcss->computeS2TLattice(sLattice, tLattice);
    
    EXPECT_DOUBLE_EQ( 4, tLattice.size() );
    
    EXPECT_DOUBLE_EQ(  0+initT, tLattice[0] );
    EXPECT_DOUBLE_EQ(  1+initT, tLattice[1] );
    EXPECT_DOUBLE_EQ(  2+initT, tLattice[2] );
    EXPECT_DOUBLE_EQ(  4+initT, tLattice[3] );
    
    sLattice.resize(2);
    sLattice[0] = 0; 
    sLattice[1] = 100; 
    funcss->computeS2TLattice(sLattice, tLattice);
    
    EXPECT_DOUBLE_EQ(  2, tLattice.size() );
    
    EXPECT_DOUBLE_EQ(  0+initT, tLattice[0] );
    EXPECT_DOUBLE_EQ(  4+initT, tLattice[1] );
    
}

TEST_F ( ParamFuncManipSplineDistTest, timeShift ) {
    
    ParamFuncsExtTypePtr funcss = dynamic_pointer_cast<ParamFuncsExtType>(funcs);
    
    using PFS = ParamFuncsStructure; using PfCpD = CtrlPtDim; using FeM = FuncEvalMode; //using EaG = EvalArcGuarantee;
    size_t funcIdx = 0; vector<PFS> pf( 4 , PFS() );
    pf[0].ctrlPtsSize = 4; pf[0].ctrlPtsArcRefIdx = 0; 
    pf[1].ctrlPtsSize = 4; pf[1].ctrlPtsArcRefIdx = 0;
    pf[2].ctrlPtsSize = 3; pf[2].ctrlPtsArcRefIdx = 1;
    pf[3].ctrlPtsSize = 3; pf[3].ctrlPtsArcRefIdx = 2;
    
    pf[0].evalReq[(size_t)FeM::INT1] = true; pf[0].evalReq[(size_t)FeM::INT2] = true;
    funcss->init( pf );
    vector<size_t> idxCfV(1,0);
    funcss->setDistCfMode(TraveledDistCfMode::V, idxCfV);
    
    double initT = 0;
    funcIdx = 0;
    funcss->ctrlPtVal(funcIdx, 0, PfCpD::VAL) = -1; funcss->ctrlPtVal(funcIdx, 0, PfCpD::ARC) = 0+initT;
    funcss->ctrlPtVal(funcIdx, 1, PfCpD::VAL) =  1; funcss->ctrlPtVal(funcIdx, 1, PfCpD::ARC) = 2+initT;
    funcss->ctrlPtVal(funcIdx, 2, PfCpD::VAL) =  0; funcss->ctrlPtVal(funcIdx, 2, PfCpD::ARC) = 3+initT;
    funcss->ctrlPtVal(funcIdx, 3, PfCpD::VAL) = -5; funcss->ctrlPtVal(funcIdx, 3, PfCpD::ARC) = 4+initT;
    funcIdx = 1;
    funcss->ctrlPtVal(funcIdx, 0, PfCpD::VAL) =  0; funcss->ctrlPtVal(funcIdx, 0, PfCpD::ARC) = 0+initT;
    funcss->ctrlPtVal(funcIdx, 1, PfCpD::VAL) =  1; funcss->ctrlPtVal(funcIdx, 1, PfCpD::ARC) = 2+initT;
    funcss->ctrlPtVal(funcIdx, 2, PfCpD::VAL) = -1; funcss->ctrlPtVal(funcIdx, 2, PfCpD::ARC) = 3+initT;
    funcss->ctrlPtVal(funcIdx, 3, PfCpD::VAL) = -5; funcss->ctrlPtVal(funcIdx, 3, PfCpD::ARC) = 4+initT;
    funcIdx = 2;
    funcss->ctrlPtVal(funcIdx, 0, PfCpD::VAL) = -10; funcss->ctrlPtVal(funcIdx, 0, PfCpD::ARC) = 0+initT;
    funcss->ctrlPtVal(funcIdx, 1, PfCpD::VAL) =   6; funcss->ctrlPtVal(funcIdx, 1, PfCpD::ARC) = 2+initT;
    funcss->ctrlPtVal(funcIdx, 2, PfCpD::VAL) =   0; funcss->ctrlPtVal(funcIdx, 2, PfCpD::ARC) = 4+initT;
    funcIdx = 3;
    funcss->ctrlPtVal(funcIdx, 0, PfCpD::VAL) = -1; funcss->ctrlPtVal(funcIdx, 0, PfCpD::ARC) = 0+initT;
    funcss->ctrlPtVal(funcIdx, 1, PfCpD::VAL) =  1; funcss->ctrlPtVal(funcIdx, 1, PfCpD::ARC) = 2+initT;
    funcss->ctrlPtVal(funcIdx, 2, PfCpD::VAL) = -1; funcss->ctrlPtVal(funcIdx, 2, PfCpD::ARC) = 4+initT;
    funcss->precompute();
    
    double shift;
    shift = 1;
    funcss->shiftStartCtrlPt(shift);
    funcIdx = 0;
    EXPECT_DOUBLE_EQ(   0, funcss->ctrlPtVal(funcIdx, 0, PfCpD::VAL) ); EXPECT_DOUBLE_EQ(                     initT, funcss->ctrlPtVal(funcIdx, 0, PfCpD::ARC) );
    EXPECT_DOUBLE_EQ(   1, funcss->ctrlPtVal(funcIdx, 1, PfCpD::VAL) ); EXPECT_DOUBLE_EQ( fmax(2+initT-shift,initT), funcss->ctrlPtVal(funcIdx, 1, PfCpD::ARC) );
    EXPECT_DOUBLE_EQ(   0, funcss->ctrlPtVal(funcIdx, 2, PfCpD::VAL) ); EXPECT_DOUBLE_EQ( fmax(3+initT-shift,initT), funcss->ctrlPtVal(funcIdx, 2, PfCpD::ARC) );
    EXPECT_DOUBLE_EQ(  -5, funcss->ctrlPtVal(funcIdx, 3, PfCpD::VAL) ); EXPECT_DOUBLE_EQ( fmax(4+initT-shift,initT), funcss->ctrlPtVal(funcIdx, 3, PfCpD::ARC) );
    
    funcIdx = 1;
    EXPECT_DOUBLE_EQ( 0.5, funcss->ctrlPtVal(funcIdx, 0, PfCpD::VAL) ); EXPECT_DOUBLE_EQ(                     initT, funcss->ctrlPtVal(funcIdx, 0, PfCpD::ARC) );
    EXPECT_DOUBLE_EQ(   1, funcss->ctrlPtVal(funcIdx, 1, PfCpD::VAL) ); EXPECT_DOUBLE_EQ( fmax(2+initT-shift,initT), funcss->ctrlPtVal(funcIdx, 1, PfCpD::ARC) );
    EXPECT_DOUBLE_EQ(  -1, funcss->ctrlPtVal(funcIdx, 2, PfCpD::VAL) ); EXPECT_DOUBLE_EQ( fmax(3+initT-shift,initT), funcss->ctrlPtVal(funcIdx, 2, PfCpD::ARC) );
    EXPECT_DOUBLE_EQ(  -5, funcss->ctrlPtVal(funcIdx, 3, PfCpD::VAL) ); EXPECT_DOUBLE_EQ( fmax(4+initT-shift,initT), funcss->ctrlPtVal(funcIdx, 3, PfCpD::ARC) );
    
    funcIdx = 2;
    EXPECT_DOUBLE_EQ(  -2, funcss->ctrlPtVal(funcIdx, 0, PfCpD::VAL) ); EXPECT_DOUBLE_EQ(                     initT, funcss->ctrlPtVal(funcIdx, 0, PfCpD::ARC) );
    EXPECT_DOUBLE_EQ(   6, funcss->ctrlPtVal(funcIdx, 1, PfCpD::VAL) ); EXPECT_DOUBLE_EQ( fmax(2+initT-shift,initT), funcss->ctrlPtVal(funcIdx, 1, PfCpD::ARC) );
    EXPECT_DOUBLE_EQ(   0, funcss->ctrlPtVal(funcIdx, 2, PfCpD::VAL) ); EXPECT_DOUBLE_EQ( fmax(4+initT-shift,initT), funcss->ctrlPtVal(funcIdx, 2, PfCpD::ARC) );
    
    funcIdx = 3;
    EXPECT_DOUBLE_EQ(   0, funcss->ctrlPtVal(funcIdx, 0, PfCpD::VAL) ); EXPECT_DOUBLE_EQ(                     initT, funcss->ctrlPtVal(funcIdx, 0, PfCpD::ARC) );
    EXPECT_DOUBLE_EQ(   1, funcss->ctrlPtVal(funcIdx, 1, PfCpD::VAL) ); EXPECT_DOUBLE_EQ( fmax(2+initT-shift,initT), funcss->ctrlPtVal(funcIdx, 1, PfCpD::ARC) );
    EXPECT_DOUBLE_EQ(  -1, funcss->ctrlPtVal(funcIdx, 2, PfCpD::VAL) ); EXPECT_DOUBLE_EQ( fmax(4+initT-shift,initT), funcss->ctrlPtVal(funcIdx, 2, PfCpD::ARC) );
    
    shift = 1.5;
    funcss->shiftStartCtrlPt(shift);
    funcIdx = 0;
    EXPECT_DOUBLE_EQ( 0.5, funcss->ctrlPtVal(funcIdx, 0, PfCpD::VAL) ); EXPECT_DOUBLE_EQ(                       initT, funcss->ctrlPtVal(funcIdx, 0, PfCpD::ARC) );
    EXPECT_DOUBLE_EQ( 0.5, funcss->ctrlPtVal(funcIdx, 1, PfCpD::VAL) ); EXPECT_DOUBLE_EQ( fmax(2+initT-shift-1,initT), funcss->ctrlPtVal(funcIdx, 1, PfCpD::ARC) );
    EXPECT_DOUBLE_EQ(   0, funcss->ctrlPtVal(funcIdx, 2, PfCpD::VAL) ); EXPECT_DOUBLE_EQ( fmax(3+initT-shift-1,initT), funcss->ctrlPtVal(funcIdx, 2, PfCpD::ARC) );
    EXPECT_DOUBLE_EQ(  -5, funcss->ctrlPtVal(funcIdx, 3, PfCpD::VAL) ); EXPECT_DOUBLE_EQ( fmax(4+initT-shift-1,initT), funcss->ctrlPtVal(funcIdx, 3, PfCpD::ARC) );
    
    funcIdx = 1;
    EXPECT_DOUBLE_EQ(   0, funcss->ctrlPtVal(funcIdx, 0, PfCpD::VAL) ); EXPECT_DOUBLE_EQ(                       initT, funcss->ctrlPtVal(funcIdx, 0, PfCpD::ARC) );
    EXPECT_DOUBLE_EQ(   0, funcss->ctrlPtVal(funcIdx, 1, PfCpD::VAL) ); EXPECT_DOUBLE_EQ( fmax(2+initT-shift-1,initT), funcss->ctrlPtVal(funcIdx, 1, PfCpD::ARC) );
    EXPECT_DOUBLE_EQ(  -1, funcss->ctrlPtVal(funcIdx, 2, PfCpD::VAL) ); EXPECT_DOUBLE_EQ( fmax(3+initT-shift-1,initT), funcss->ctrlPtVal(funcIdx, 2, PfCpD::ARC) );
    EXPECT_DOUBLE_EQ(  -5, funcss->ctrlPtVal(funcIdx, 3, PfCpD::VAL) ); EXPECT_DOUBLE_EQ( fmax(4+initT-shift-1,initT), funcss->ctrlPtVal(funcIdx, 3, PfCpD::ARC) );
    
    funcIdx = 2;
    EXPECT_DOUBLE_EQ(4.5, funcss->ctrlPtVal(funcIdx, 0, PfCpD::VAL) ); EXPECT_DOUBLE_EQ(                      initT, funcss->ctrlPtVal(funcIdx, 0, PfCpD::ARC) );
    EXPECT_DOUBLE_EQ(4.5, funcss->ctrlPtVal(funcIdx, 1, PfCpD::VAL) ); EXPECT_DOUBLE_EQ( fmax(2+initT-shift-1,initT), funcss->ctrlPtVal(funcIdx, 1, PfCpD::ARC) );
    EXPECT_DOUBLE_EQ(  0, funcss->ctrlPtVal(funcIdx, 2, PfCpD::VAL) ); EXPECT_DOUBLE_EQ( fmax(4+initT-shift-1,initT), funcss->ctrlPtVal(funcIdx, 2, PfCpD::ARC) );
    
    funcIdx = 3;
    EXPECT_DOUBLE_EQ( 0.5, funcss->ctrlPtVal(funcIdx, 0, PfCpD::VAL) ); EXPECT_DOUBLE_EQ(                       initT, funcss->ctrlPtVal(funcIdx, 0, PfCpD::ARC) );
    EXPECT_DOUBLE_EQ( 0.5, funcss->ctrlPtVal(funcIdx, 1, PfCpD::VAL) ); EXPECT_DOUBLE_EQ( fmax(2+initT-shift-1,initT), funcss->ctrlPtVal(funcIdx, 1, PfCpD::ARC) );
    EXPECT_DOUBLE_EQ(  -1, funcss->ctrlPtVal(funcIdx, 2, PfCpD::VAL) ); EXPECT_DOUBLE_EQ( fmax(4+initT-shift-1,initT), funcss->ctrlPtVal(funcIdx, 2, PfCpD::ARC) );
}

// TEST_F ( ParamFuncManipSplineDistTest, evalArcFuncValFuncValInconsistentArc ) {
//     
//     using PFS = ParamFuncsStructure; using PfCpD = CtrlPtDim; using EaG = EvalArcGuarantee;
//     size_t funcIdx = 0; vector<PFS> pf( 2 , PFS() );
//     pf[0].ctrlPtsSize = 4; pf[0].ctrlPtsArcRefIdx = 0; 
//     pf[1].ctrlPtsSize = 6; pf[1].ctrlPtsArcRefIdx = 1;
//     
//     funcs->init( pf );
//     
//     double initT = 3;
//     funcIdx = 0;
//     funcs->ctrlPtVal(funcIdx, 0, PfCpD::VAL) = -1; funcs->ctrlPtVal(funcIdx, 0, PfCpD::ARC) = 0+initT;
//     funcs->ctrlPtVal(funcIdx, 1, PfCpD::VAL) =  1; funcs->ctrlPtVal(funcIdx, 1, PfCpD::ARC) = 2+initT;
//     funcs->ctrlPtVal(funcIdx, 2, PfCpD::VAL) = -1; funcs->ctrlPtVal(funcIdx, 2, PfCpD::ARC) = 3+initT;
//     funcs->ctrlPtVal(funcIdx, 3, PfCpD::VAL) = -5; funcs->ctrlPtVal(funcIdx, 3, PfCpD::ARC) = 4+initT;
//     
//     funcIdx = 1;
//     funcs->ctrlPtVal(funcIdx, 0, PfCpD::VAL) =  0;
//     funcs->ctrlPtVal(funcIdx, 1, PfCpD::VAL) = -1; funcs->ctrlPtVal(funcIdx, 1, PfCpD::ARC) = 3+initT;
//     funcs->ctrlPtVal(funcIdx, 2, PfCpD::VAL) =  1; funcs->ctrlPtVal(funcIdx, 2, PfCpD::ARC) = 5+initT;
//     funcs->ctrlPtVal(funcIdx, 3, PfCpD::VAL) =  3; funcs->ctrlPtVal(funcIdx, 3, PfCpD::ARC) = 4+initT;
//     funcs->ctrlPtVal(funcIdx, 4, PfCpD::VAL) = -5; funcs->ctrlPtVal(funcIdx, 4, PfCpD::ARC) = 6+initT;
//     funcs->ctrlPtVal(funcIdx, 5, PfCpD::VAL) =  0;
//     //so this temp val is 0 -1 1 3 -5 0
//     //so this temp arc is 0  3 5 4  7 4
//     funcs->precompute();
//     
//     funcs->setEvalArc(4.0+initT, EaG::NONE      ); EXPECT_DOUBLE_EQ(     0, funcs->computeFuncVal(1) ); 
//     funcs->setEvalArc(5.0+initT, EaG::NEAR_LAST); EXPECT_DOUBLE_EQ(    -1, funcs->computeFuncVal(1) );//goes till 2nd range where 5 is found...first range is [3,5)
//     
//     funcIdx = 1;
//     funcs->ctrlPtVal(funcIdx, 0, PfCpD::VAL) =  0;
//     funcs->ctrlPtVal(funcIdx, 1, PfCpD::VAL) =  1; funcs->ctrlPtVal(funcIdx, 1, PfCpD::ARC) = 3+initT;
//     funcs->ctrlPtVal(funcIdx, 2, PfCpD::VAL) =  1; funcs->ctrlPtVal(funcIdx, 2, PfCpD::ARC) = 2+initT;
//     funcs->ctrlPtVal(funcIdx, 3, PfCpD::VAL) =  3; funcs->ctrlPtVal(funcIdx, 3, PfCpD::ARC) = 2+initT;
//     funcs->ctrlPtVal(funcIdx, 4, PfCpD::VAL) = -3; funcs->ctrlPtVal(funcIdx, 4, PfCpD::ARC) = 1+initT;
//     funcs->ctrlPtVal(funcIdx, 5, PfCpD::VAL) = -1; funcs->ctrlPtVal(funcIdx, 4, PfCpD::ARC) =-1+initT;
//     //so this temp val is 0 -1 1 3 -5  0
//     //so this temp arc is 0  3 2 2  1 -1
//     funcs->precompute();
//     
//     funcs->setEvalArc(0.0+initT, EaG::NONE      ); EXPECT_DOUBLE_EQ(     0, funcs->computeFuncVal(1) ); 
// }


}  // namespace

int main ( int argc, char **argv ) {
    ::testing::InitGoogleTest ( &argc, argv );
    return RUN_ALL_TESTS();
}
