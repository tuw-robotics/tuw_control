/***************************************************************************
 *   Software License Agreement (BSD License)                              *
 *   Copyright (C) 2017 by Horatiu George Todoran <todorangrg@gmail.com>   *
 *                                                                         *
 *   Redistribution and use in source and binary forms, with or without    *
 *   modification, are permitted provided that the following conditions    *
 *   are met:                                                              *
 *                                                                         *
 *   1. Redistributions of source code must retain the above copyright     *
 *      notice, this list of conditions and the following disclaimer.      *
 *   2. Redistributions in binary form must reproduce the above copyright  *
 *      notice, this list of conditions and the following disclaimer in    *
 *      the documentation and/or other materials provided with the         *
 *      distribution.                                                      *
 *   3. Neither the name of the copyright holder nor the names of its      *
 *      contributors may be used to endorse or promote products derived    *
 *      from this software without specific prior written permission.      *
 *                                                                         *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS   *
 *   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT     *
 *   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS     *
 *   FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE        *
 *   COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,  *
 *   INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,  *
 *   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;      *
 *   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER      *
 *   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT    *
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY *
 *   WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           *
 *   POSSIBILITY OF SUCH DAMAGE.                                           *
 ***************************************************************************/

#ifndef AGV_DIFF_DRIVE_V_W_HPP
#define AGV_DIFF_DRIVE_V_W_HPP

#include <tuw_control/state_sim/state_sim.hpp>
#include <tuw_control/param_func_new/param_func_spline/param_func_spline0_dist.hpp>
#include <tuw_control/utils.h>

namespace tuw {

using NumType = double;


// Defining the system state

//Part that cannot be solved in closed-form, but rather numerically (nm)
class StateNmDiffDriveVW : public StateMapArray<NumType, NumType, 2> {
    public   : using StateMapArray::StateMapArray;
    public   : NumType&       x    ()       { return data()(0); }
    public   : const NumType& x    () const { return data()(0); }
    public   : NumType&       y    ()       { return data()(1); }
    public   : const NumType& y    () const { return data()(1); }
};
//Part that can be solved in closed-form (cf)
class StateCfDiffDriveVW : public StateMapArray<NumType, NumType, 5> {
    public   : using StateMapArray::StateMapArray;
    public   : NumType&       theta()       { return data()(0); }
    public   : const NumType& theta() const { return data()(0); }
    public   : NumType&       v    ()       { return data()(1); }
    public   : const NumType& v    () const { return data()(1); }
    public   : NumType&       w    ()       { return data()(2); }
    public   : const NumType& w    () const { return data()(2); }
    public   : NumType&       t    ()       { return data()(3); }
    public   : const NumType& t    () const { return data()(3); }
    public   : NumType&       s    ()       { return data()(4); }
    public   : const NumType& s    () const { return data()(4); }
};
//Full system state
using StateDiffDriveVW = StateMapSimBase<NumType, StateNmDiffDriveVW, StateCfDiffDriveVW>;


template<class TDerivedType, class TDerivedStateType, template<class> class TDiscretizationType>
class StateSimDiffDriveVWBase : public StateSimBase<TDerivedType, TDerivedStateType, TDiscretizationType> {

    //implement functions that evaluate temporal and distance arc variables
    public  : NumType  stateArcImpl  () { return this->state().stateCf().t(); }
    public  : NumType  stateDistImpl () { return this->state().stateCf().s(); }
    
    //choose a parametric function structure and implement accessor functions
    public   : enum class ParamFuncVars{ V, W };
    protected: ParamFuncsSpline0Dist<NumType,2,1> paramFuncs_;

    public  :       auto* paramFuncsImpl     ()       { return &paramFuncs_; }
    public  : const auto* paramFuncsImpl     () const { return &paramFuncs_; }
    public  :       auto* paramFuncsDistImpl ()       { return &paramFuncs_; }//if no dist implementation return nullptr
    public  : const auto* paramFuncsDistImpl () const { return &paramFuncs_; }
    
    //implement evaluation of the closed-form state at a given arc and a given evaluation order guarantee
    //first part is used when computing the numerical state derivative in the discretization function. Thus, only the variables of which the numerical state depends on have to be computed.
    public  : void setStateCfNmStepImpl ( const NumType& _arc, const PfEaG& _eAG ) { 
	paramFuncs_.setEvalArc (_arc, _eAG);
	auto& stateCf = this->state().stateCf();
	stateCf.t() = _arc;
	const NumType thetaIndef = paramFuncs_.computeFuncInt1  ( asInt(ParamFuncVars::W) );
	stateCf.theta() = thetaIndef + this->state0().stateCf().theta();
	cosTheta_ = cos(stateCf.theta());
	sinTheta_ = sin(stateCf.theta());
	stateCf.v    () = paramFuncs_.computeFuncVal(asInt(ParamFuncVars::V));
    }
    //same as before, but all closed-form variables have to be computed
    public  : void setStateCfImpl ( const NumType& _arc, const PfEaG& _eAG ) {
	setStateCfNmStepImpl(_arc, _eAG);
	auto& stateCf = this->state().stateCf();
	stateCf.w    () = paramFuncs_.computeFuncVal(asInt(ParamFuncVars::W));
	stateCf.s    () = paramFuncs_.computeS();
    }
    //implement evaluation of the numerical state variables derivatives at a given arc and a given evaluation order guarantee
    public  : auto& stateNmDotImpl () {
	auto& stateNmDot = this->stateNmDotCache_;
	stateNmDot.x()     = this->state().stateCf().v() * cosTheta_;
	stateNmDot.y()     = this->state().stateCf().v() * sinTheta_;
	return stateNmDot;
    }
    
    //internal helper variables
    protected: NumType cosTheta_;
    protected: NumType sinTheta_;
};

//---------------------------------------------------------------------Optimization parameters

static constexpr const size_t optParamBlockSize = 3;

template<typename TLeafType>
class OptVarStructDiffDriveVW : public StateMapArray<NumType, StateMapVector<NumType, TLeafType>, optParamBlockSize> {
    public   : using StateMapArray<NumType, StateMapVector<NumType, TLeafType>, optParamBlockSize>::StateMapArray;
    public   : auto&        optParamV()       { return this->sub(0); }//parameters influencing linear velocity
    public   : const auto&  optParamV() const { return this->sub(0); }
    public   : auto&        optParamW()       { return this->sub(1); }//parameters influencing angular velocity
    public   : const auto&  optParamW() const { return this->sub(1); }
    public   : auto&        optParamT()       { return this->sub(2); }//parameters influencing temporal location of previous parameters
    public   : const auto&  optParamT() const { return this->sub(2); }
    public   : void        setPBlockSize(const size_t& _i, const size_t& _n)       { this->sub(_i).subResize(_n); }
    public   :       auto& dxdpBlockI   (const size_t& _i)                         { return this->sub(_i); }
    public   : const auto& dxdpBlockI   (const size_t& _i)                   const { return this->sub(_i); }
};


class StateWithGradDiffDriveVW : public StateWithGradMapSimBase< NumType, StateNmDiffDriveVW, StateCfDiffDriveVW, OptVarStructDiffDriveVW > {
    public   : StateWithGradDiffDriveVW() : StateWithGradMapSimBase< NumType, StateNmDiffDriveVW, StateCfDiffDriveVW, OptVarStructDiffDriveVW > () {
	for( size_t i = 0; i < optParamBlockSize; ++i ) {
	    stateGradNm().setPBlockSize(i, 3);
	    stateGradCf().setPBlockSize(i, 3);
	}
    }
};

template<template<class> class TDiscretizationType>
class StateSimDiffDriveVW : public StateSimDiffDriveVWBase<StateSimDiffDriveVW<TDiscretizationType>, StateDiffDriveVW, TDiscretizationType>{};

template<template<class> class TDiscretizationType>
class StateWithGradSimDiffDriveVW : public StateSimDiffDriveVWBase<StateWithGradSimDiffDriveVW<TDiscretizationType>, StateWithGradDiffDriveVW, TDiscretizationType> {
    
    //implement evaluation of the numerical state variables derivatives at a given arc and a given evaluation order guarantee
    public  : void                  setStateCfWithGradNmStepImpl ( const NumType& _arc, const PfEaG& _eAG ) { 
	this->setStateCfNmStepImpl(_arc, _eAG);
	
	auto& stateGradCf       = this->state().stateGradCf();
	
	auto& stateGradCfParamV = stateGradCf.optParamV();
	for(size_t i = 0; i < stateGradCfParamV.subSize(); ++i){
	    auto& stateGradCfParamVI = stateGradCfParamV.sub(i).data();
	    
	    stateGradCfParamVI(1) = 0;
	    stateGradCfParamVI(4) = 0;
	    if(i+1 < stateGradCfParamV.subSize()) {
		const NumType& evalArcAbove = this->paramFuncs_.ctrlPtVal(0,i+2,CtrlPtDim::ARC);
		const NumType& evalArcBelow = this->paramFuncs_.ctrlPtVal(0,i+1,CtrlPtDim::ARC);
		if(_arc > evalArcBelow) {
		    const NumType arcIntEnd = fmin(_arc, evalArcAbove);
		    const NumType deltaEvalArc = evalArcAbove-evalArcBelow;
		    stateGradCfParamVI(4) = + (arcIntEnd - evalArcBelow) * (evalArcBelow - 2. * _arc + arcIntEnd) / (2.*deltaEvalArc);
		    stateGradCfParamVI(1) = - (arcIntEnd - evalArcBelow) / deltaEvalArc;
		}
	    }
	    const NumType& evalArcAbove = this->paramFuncs_.ctrlPtVal(0,i+1,CtrlPtDim::ARC);
	    const NumType& evalArcBelow = this->paramFuncs_.ctrlPtVal(0,i  ,CtrlPtDim::ARC);
	    if(_arc > evalArcBelow) {
		const NumType arcIntEnd = fmin(_arc, evalArcAbove);
		const NumType deltaEvalArc = evalArcAbove-evalArcBelow;
		stateGradCfParamVI(4) += - (arcIntEnd - evalArcBelow) * (evalArcBelow - 2. * _arc + arcIntEnd) / (2.*deltaEvalArc);
		stateGradCfParamVI(1) += + (arcIntEnd - evalArcBelow) / deltaEvalArc;
	    }
	}
	
	auto& stateGradCfParamW = stateGradCf.optParamW();
	for(size_t i = 0; i < stateGradCfParamW.subSize(); ++i){
	    auto& stateGradCfParamWI = stateGradCfParamW.sub(i).data();
	    
	    stateGradCfParamWI(0) = 0;
	    stateGradCfParamWI(2) = 0;
	    if(i+1 < stateGradCfParamW.subSize()) {
		const NumType& evalArcAbove = this->paramFuncs_.ctrlPtVal(1,i+2,CtrlPtDim::ARC);
		const NumType& evalArcBelow = this->paramFuncs_.ctrlPtVal(1,i+1,CtrlPtDim::ARC);
		if(_arc > evalArcBelow) {
		    const NumType arcIntEnd = fmin(_arc, evalArcAbove);
		    const NumType deltaEvalArc = evalArcAbove-evalArcBelow;
		    stateGradCfParamWI(0) = + (arcIntEnd - evalArcBelow) * (evalArcBelow - 2. * _arc + arcIntEnd) / (2.*deltaEvalArc);
		    stateGradCfParamWI(2) = - (arcIntEnd - evalArcBelow) / deltaEvalArc;
		}
	    }
	    const NumType& evalArcAbove = this->paramFuncs_.ctrlPtVal(1,i+1,CtrlPtDim::ARC);
	    const NumType& evalArcBelow = this->paramFuncs_.ctrlPtVal(1,i  ,CtrlPtDim::ARC);
	    if(_arc > evalArcBelow) {
		const NumType arcIntEnd = fmin(_arc, evalArcAbove);
		const NumType deltaEvalArc = evalArcAbove-evalArcBelow;
		stateGradCfParamWI(0) += - (arcIntEnd - evalArcBelow) * (evalArcBelow - 2. * _arc + arcIntEnd) / (2.*deltaEvalArc);
		stateGradCfParamWI(2) += + (arcIntEnd - evalArcBelow) / deltaEvalArc;
	    }
	}
	
	auto& stateGradCfParamT = stateGradCf.optParamT();
	for(size_t i = 0; i < stateGradCfParamT.subSize(); ++i) {
	    auto& stateGradCfParamTI = stateGradCfParamT.sub(i).data();

	    stateGradCfParamTI(0) = 0;
	    stateGradCfParamTI(2) = 0;
	    if ( i+1 < this->paramFuncs_.funcCtrlPtSize(0) ) {
		const NumType& evalArcAbove  = this->paramFuncs_.ctrlPtVal(1,i+1,CtrlPtDim::ARC);
		const NumType& evalArcBelow  = this->paramFuncs_.ctrlPtVal(1,i  ,CtrlPtDim::ARC);
		if ( ( _arc <= evalArcAbove ) && ( _arc > evalArcBelow ) ) {
		    const NumType& vP              = this->paramFuncs_.ctrlPtVal(0,i+1,CtrlPtDim::VAL);
		    const NumType& vM              = this->paramFuncs_.ctrlPtVal(0,i+0,CtrlPtDim::VAL);
		    const NumType& wP              = this->paramFuncs_.ctrlPtVal(1,i+1,CtrlPtDim::VAL);
		    const NumType& wM              = this->paramFuncs_.ctrlPtVal(1,i+0,CtrlPtDim::VAL);
		    const NumType arcIntEnd        = fmin(_arc, evalArcAbove);
		    const NumType deltaEvalArc     = evalArcAbove-evalArcBelow;
		    const NumType deltaEvalArcSqr  = deltaEvalArc*deltaEvalArc;
		    const NumType deltaArcIntBound = arcIntEnd - evalArcBelow;
		    stateGradCfParamTI(0)     = + deltaArcIntBound * (wP-wM) * (evalArcBelow - 2. * _arc + arcIntEnd) / (2.*deltaEvalArcSqr);
		    stateGradCfParamTI(1)     = - deltaArcIntBound * (vP-vM) / deltaEvalArcSqr;
		    stateGradCfParamTI(2)     = - deltaArcIntBound * (wP-wM) / deltaEvalArcSqr;
		    stateGradCfParamTI(4)     = + deltaArcIntBound * (vP-vM) * (evalArcBelow - 2. * _arc + arcIntEnd) / (2.*deltaEvalArcSqr);
		    
		} else if ( ( _arc > evalArcBelow ) && ( i+1 < stateGradCfParamT.subSize() ) ) {
		    const NumType& evalArcAbove2 = this->paramFuncs_.ctrlPtVal(1,i+2,CtrlPtDim::ARC);
		    if(_arc < evalArcAbove2){
			const NumType& vP              = this->paramFuncs_.ctrlPtVal(0,i+2,CtrlPtDim::VAL);
			const NumType& vM              = this->paramFuncs_.ctrlPtVal(0,i+1,CtrlPtDim::VAL);
			const NumType& vMM             = this->paramFuncs_.ctrlPtVal(0,i+0,CtrlPtDim::VAL);
			const NumType& wP              = this->paramFuncs_.ctrlPtVal(1,i+2,CtrlPtDim::VAL);
			const NumType& wM              = this->paramFuncs_.ctrlPtVal(1,i+1,CtrlPtDim::VAL);
			const NumType& wMM             = this->paramFuncs_.ctrlPtVal(1,i+0,CtrlPtDim::VAL);
			const NumType arcIntEnd        = fmin(_arc, evalArcAbove2);
			const NumType deltaEvalArc     = evalArcAbove2-evalArcAbove;
			const NumType deltaEvalArcSqr  = deltaEvalArc*deltaEvalArc;
			const NumType deltaArcIntBound = evalArcAbove2 - arcIntEnd;
			stateGradCfParamTI(0)     = - ( +(wM-wMM)* evalArcAbove2*evalArcAbove2
							+(wP-wMM)*(evalArcAbove*evalArcAbove-2.*evalArcAbove*evalArcAbove2)
							+(wP-wM )*(arcIntEnd*arcIntEnd + 2.*_arc*(evalArcAbove2 - arcIntEnd)) ) / (2.*deltaEvalArcSqr);
			stateGradCfParamTI(1)     = - deltaArcIntBound * (vP-vM) / deltaEvalArcSqr;
			stateGradCfParamTI(2)     = - deltaArcIntBound * (wP-wM) / deltaEvalArcSqr;
			stateGradCfParamTI(4)     = - ( +(vM-vMM)* evalArcAbove2*evalArcAbove2
							+(vP-vMM)*(evalArcAbove*evalArcAbove-2.*evalArcAbove*evalArcAbove2)
							+(vP-vM )*(arcIntEnd*arcIntEnd + 2.*_arc*(evalArcAbove2 - arcIntEnd)) ) / (2.*deltaEvalArcSqr);
		    } else {
			const NumType& vP      = this->paramFuncs_.ctrlPtVal(0,i+2,CtrlPtDim::VAL);
			const NumType& vM      = this->paramFuncs_.ctrlPtVal(0,i+0,CtrlPtDim::VAL);
			const NumType& wP      = this->paramFuncs_.ctrlPtVal(1,i+2,CtrlPtDim::VAL);
			const NumType& wM      = this->paramFuncs_.ctrlPtVal(1,i+0,CtrlPtDim::VAL);
			stateGradCfParamTI(0)     = - (wP-wM)/2.;
			stateGradCfParamTI(4)     = - (vP-vM)/2.;
		    }
		} else if ( _arc > evalArcBelow ) {
		    const NumType& vP      = this->paramFuncs_.ctrlPtVal(0,i+1,CtrlPtDim::VAL);
		    const NumType& vM      = this->paramFuncs_.ctrlPtVal(0,i+0,CtrlPtDim::VAL);
		    const NumType& wP      = this->paramFuncs_.ctrlPtVal(1,i+1,CtrlPtDim::VAL);
		    const NumType& wM      = this->paramFuncs_.ctrlPtVal(1,i+0,CtrlPtDim::VAL);
		    stateGradCfParamTI(0)     = - (wP-wM)/2.;
		    stateGradCfParamTI(4)     = - (vP-vM)/2.;
		}
	    }
	}
    }
    
    public  : void   setStateCfWithGradImpl       ( const NumType& _arc, const PfEaG& _eAG ) {
	setStateCfWithGradNmStepImpl(_arc, _eAG);
	
	auto& stateCf = this->state().stateCf();
	stateCf.w () = this->paramFuncs_.computeFuncVal(asInt(StateSimDiffDriveVWBase<StateWithGradSimDiffDriveVW<TDiscretizationType>, StateWithGradDiffDriveVW, TDiscretizationType>::ParamFuncVars::W));
	stateCf.s () = this->paramFuncs_.computeS();
    }
    
    public  : auto& stateWithGradNmDotImpl () {
	
	auto& stateNmDot = this->stateWithGradNmDotCache_.template sub<0>();
	stateNmDot.x () = this->state().stateCf().v() * this->cosTheta_;
	stateNmDot.y () = this->state().stateCf().v() * this->sinTheta_;
	
	const auto& stateCf     = this->state().stateCf();
	const auto& stateGradCf = this->state().stateGradCf();
	auto& stateGradNmDot    = this->stateWithGradNmDotCache_.template sub<1>();
	
	//combining dfdx * GradX + dfdu * dudp
	static Eigen::Matrix<NumType,2,1> dfduX;
	static Eigen::Matrix<NumType,2,1> dfduY;
	
	dfduX(0) = this->cosTheta_;
	dfduX(1) = this->sinTheta_;
	dfduY(0) = - stateCf.v() * this->sinTheta_;
	dfduY(1) = + stateCf.v() * this->cosTheta_;
	auto& stateGradNmDotParamV = stateGradNmDot.sub(0);
	auto& stateGradNmDotParamW = stateGradNmDot.sub(1);
	auto& stateGradNmDotParamT = stateGradNmDot.sub(2);
	for(size_t i = 0; i < stateGradNmDotParamV.subSize(); ++i) {
	    const auto& stateGradCfParamVIV     = stateGradCf.sub(0).sub(i).v();
	    const auto& stateGradCfParamWITheta = stateGradCf.sub(1).sub(i).theta();
	    const auto& stateGradCfParamTIV     = stateGradCf.sub(2).sub(i).v();
	    const auto& stateGradCfParamTITheta = stateGradCf.sub(2).sub(i).theta();
	    stateGradNmDotParamV.sub(i).data() = stateGradCfParamVIV * dfduX;
	    stateGradNmDotParamW.sub(i).data() = stateGradCfParamWITheta * dfduY;
	    stateGradNmDotParamT.sub(i).data() = stateGradCfParamTIV * dfduX + stateGradCfParamTITheta * dfduY;
	}
	return this->stateWithGradNmDotCache_;
    }
};
class StateSimDiffDriveVWGradMap : public StateWithGradMapBase<StateWithGradDiffDriveVW> {
    public   : StateSimDiffDriveVWGradMap() : StateWithGradMapBase<StateWithGradDiffDriveVW>() {
	for( size_t i = 0; i < optParamBlockSize; ++i ) {
	    stateGradNm().setPBlockSize(i, 3);
	    stateGradCf().setPBlockSize(i, 3);
	}
    }
};

// using StateDiffDriveType = StateDiffDriveVW;
using StateDiffDriveType    = StateWithGradDiffDriveVW;
template<template<class> class TDiscretizationType> using StateSimDiffDriveType = StateWithGradSimDiffDriveVW<TDiscretizationType>;

template<template<class> class TDiscretizationType> using StateSimDiffDriveVWCRTPType = StateSimBaseCRTP<StateSimDiffDriveType<TDiscretizationType>>;

using StateSimDiffDriveVWHeunType     = StateSimDiffDriveType<heun_abc>;
using StateSimDiffDriveVWHeunCRTPType = StateSimDiffDriveVWCRTPType<heun_abc>;



}

#endif // AGV_DIFF_DRIVE_V_W_HPP
