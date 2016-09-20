/***************************************************************************
 *   Software License Agreement (BSD License)                              *
 *   Copyright (C) 2016 by Horatiu George Todoran <todorangrg@gmail.com>   *
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

#include <tuw_control/trajectory_evaluator/trajectory_optimizer.h>
#include <tuw_control/utils.h>

#include <algorithm>

#include <iostream>

using namespace std;
using namespace tuw;

TrajectoryOptimizer::TrajectoryOptimizer ( StateSimPtr& _stateSim, unique_ptr< TrajectorySimulator::CostsEvaluatorClass > _costsEvaluator, OptimizationStateSPtr _optState ) :
      trajSimGrade_(_stateSim, std::move(_costsEvaluator) ),
      stateSim_(trajSimGrade_.trajSim()->stateSim()),
      paramFuncs_(trajSimGrade_.trajSim()->stateSim()->paramFuncs()),
      optState_(_optState) {
	
// 	state0varAccess_           = [this](size_t i)           -> double& { return stateSim_->state(i); };
//     	paramFuncCtrlPtAccess_     = [this](size_t i, size_t j) -> double& { return paramFuncs_->ctrlPtVal(i,j, ParamFuncs::CtrlPtDim::VAL); };
//     	paramFuncArcAccess_        = [this](size_t i, size_t j) -> double& { return paramFuncs_->funcsArc (i,j); };
//     	paramFuncCtrlPtEndAccess_  = [this](size_t i)           -> double& { return paramFuncs_->ctrlPtVal(i,paramFuncs_->funcCtrlPtSize(i)-1, ParamFuncs::CtrlPtDim::VAL); };
//     	paramFuncArcEndAccess_     = [this](size_t i)           -> double& { return paramFuncs_->funcsArc (i,paramFuncs_->funcsArcSize  (i)-1); };
	
// 	varsAccess_[asInt(AccessType::STATE_0      )] = [this](size_t i, size_t j) -> double& { return stateSim_->state0().value(i); };
//     	varsAccess_[asInt(AccessType::PARAM_CP     )] = [this](size_t i, size_t j) -> double& { return paramFuncs_->ctrlPtVal(i,j, ParamFuncs::CtrlPtDim::VAL); };
//     	varsAccess_[asInt(AccessType::PARAM_ARC    )] = [this](size_t i, size_t j) -> double& { return paramFuncs_->funcsArc (i,j); };
//     	varsAccess_[asInt(AccessType::PARAM_CP_END )] = [this](size_t i, size_t j) -> double& { return paramFuncs_->ctrlPtVal(i,paramFuncs_->funcCtrlPtSize(i)-1, ParamFuncs::CtrlPtDim::VAL); };
//     	varsAccess_[asInt(AccessType::PARAM_ARC_END)] = [this](size_t i, size_t j) -> double& { return paramFuncs_->funcsArc (i,paramFuncs_->funcsArcSize  (i)-1); };
}

void TrajectoryOptimizer::cacheVarsSize() {
    varsSize_[0] = varsIdx_[0].size();
    varsSize_[1] = 0; for(size_t i = 0; i < varsIdx_[1].size(); ++i) { varsSize_[1] += varsIdx_[1][i] * ( paramFuncs_->funcCtrlPtSize(i)-2 ); }
    varsSize_[2] = 0; for(size_t i = 0; i < varsIdx_[1].size(); ++i) { varsSize_[2] += varsIdx_[2][i] * ( paramFuncs_->funcsArcSize  (i)-2 ); }
    varsSize_[3] = varsIdx_[3].size();
    varsSize_[4] = varsIdx_[4].size();
}

void TrajectoryOptimizer::operateOnSimpleVar ( const std::size_t& _varClass, OperatorFunction _func ) {
    for(size_t i = 0; i < varsIdx_[_varClass].size(); ++i) {
	(this->*_func)( _varClass, i, 0);
    }
}
void TrajectoryOptimizer::operateOnCtrlPtVar ( TrajectoryOptimizer::OperatorFunction _func ) {
    for(size_t i = 0; i < varsIdx_[asInt(AccessType::PARAM_CP)].size(); ++i) {
	for(size_t j = 1; j < paramFuncs_->funcCtrlPtSize(i) - 1; ++j) {
	    (this->*_func)(asInt(AccessType::PARAM_CP), i, j);
	}
    }
}
void TrajectoryOptimizer::operateOnFuncArcVar ( TrajectoryOptimizer::OperatorFunction _func ) {
    for(size_t i = 0; i < varsIdx_[asInt(AccessType::PARAM_ARC)].size(); ++i) {
	for(size_t j = 1; j < paramFuncs_->funcsArcSize(i) - 1; ++j) {
	    (this->*_func)(asInt(AccessType::PARAM_ARC), i, j );
	}
    }
}

void TrajectoryOptimizer::computeJacobian() {
//     operateOnSimpleVar ( asInt(AccessType::STATE_0      ), &TrajectoryOptimizer::computeJacobianEntryGeneric );
//     operateOnCtrlPtVar (                                   &TrajectoryOptimizer::computeJacobianEntryGeneric );
//     operateOnFuncArcVar(                                   &TrajectoryOptimizer::computeJacobianEntryGeneric );
//     operateOnSimpleVar ( asInt(AccessType::PARAM_CP_END ), &TrajectoryOptimizer::computeJacobianEntryGeneric );
//     operateOnSimpleVar ( asInt(AccessType::PARAM_ARC_END), &TrajectoryOptimizer::computeJacobianEntryGeneric );
}


void TrajectoryOptimizer::computeJacobianEntryGeneric ( std::size_t _varClass, std::size_t _varType, std::size_t _varIdx ) {

}

