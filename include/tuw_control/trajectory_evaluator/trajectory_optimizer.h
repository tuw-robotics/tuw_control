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

#ifndef TRAJECTORY_OPTIMIZER_H
#define TRAJECTORY_OPTIMIZER_H

#include <tuw_control/trajectory_evaluator/trajectory_sim_grade.h>

#include <functional>
#include <memory>

namespace tuw {

class OptimizationState;
using OptimizationStateSPtr      = std::shared_ptr<OptimizationState>;
using OptimizationStateConstSPtr = std::shared_ptr<OptimizationState const>;
using OptimizationStateUPtr      = std::unique_ptr<OptimizationState>;
using OptimizationStateConstUPtr = std::unique_ptr<OptimizationState const>;
    
class OptimizationState : public State {
    
    public   : struct OptStateData {
	OptStateData ( double* _val, double* _arcBegin = 0 ) : val(_val), arcBegin(_arcBegin){}
	double* val;
	double* arcBegin;
    };
    
    public   : OptimizationState           ()                         = default;
    public   : ~OptimizationState          ()                         = default;
    public   : OptimizationState           (const OptimizationState&) = default;
    public   : OptimizationState& operator=(const OptimizationState&) = default;
    public   : OptimizationState           (OptimizationState&&)      = default;
    public   : OptimizationState& operator=(OptimizationState&&)      = default;
    
    public   : virtual void bindValues(TrajectorySimulator& _trajOpt) = 0;
    public   : double&       value      ( const std::size_t& _i )       override { return *values[_i].val; }
    public   : const double& value      ( const std::size_t& _i ) const override { return *values[_i].val; }
    public   : size_t        valueSize  () const override { return values.size(); }
    
    public   : const double& arcBegin   ( const std::size_t& _i ) const { return *values[_i].arcBegin; }
    
    protected: std::vector<OptStateData> values;
};

class OptimizationStateTest : public OptimizationState {
    public   : StateSPtr     cloneState () const override { return std::make_shared<OptimizationStateTest>(*this); }
    public   : void bindValues(TrajectorySimulator& _trajSim) override {
// 	static double valueZero = 0;
	values.clear();
// 	values.emplace_back( OptStateData( *_trajOpt.stateSim_->state0().value(0), valueZero ) );
	auto* paramFuncs = _trajSim.stateSim()->paramFuncs();
	for ( size_t i = 0; i < paramFuncs->funcsSize(); ++i ) {
	    for ( size_t j = 1; j < paramFuncs->funcCtrlPtSize(i); ++j ) {
		values.emplace_back(  &paramFuncs->ctrlPtVal ( i, j, ParamFuncs::CtrlPtDim::VAL ), &paramFuncs->ctrlPtVal( i, j-1, ParamFuncs::CtrlPtDim::ARC ) );
	    }
	}
	
	for ( size_t j = 1; j < paramFuncs->funcsArcSize(0); ++j ) {
	    values.emplace_back(  &paramFuncs->funcsArc ( 0, j ), &paramFuncs->funcsArc( 0, j-1 ) );
	}
    }
};
    
    
class TrajectoryOptimizer;
using TrajectoryOptimizerSPtr      = std::shared_ptr<TrajectoryOptimizer>;
using TrajectoryOptimizerConstSPtr = std::shared_ptr<TrajectoryOptimizer const>;
using TrajectoryOptimizerUPtr      = std::unique_ptr<TrajectoryOptimizer>;
using TrajectoryOptimizerConstUPtr = std::unique_ptr<TrajectoryOptimizer const>;

class TrajectoryOptimizer {
    public  : enum class AccessType {
	STATE_0,
	PARAM_CP,
	PARAM_ARC,
	ENUM_SIZE
    };
    
    //special class member functions
    public   : TrajectoryOptimizer           ( StateSimPtr& _stateSim, std::unique_ptr<TrajectorySimulator::CostsEvaluatorClass> _costsEvaluator, OptimizationStateSPtr _optState );
    public   : ~TrajectoryOptimizer          ()                          = default;
    public   : TrajectoryOptimizer           (const TrajectoryOptimizer&) = default;
    public   : TrajectoryOptimizer& operator=(const TrajectoryOptimizer&) = default;
    public   : TrajectoryOptimizer           (TrajectoryOptimizer&&)      = default;
    public   : TrajectoryOptimizer& operator=(TrajectoryOptimizer&&)      = default;
    
    protected: void cacheVarsSize();
    
    public   : virtual void setVars() = 0;
    
    using OperatorFunction = void(TrajectoryOptimizer::*)(size_t, size_t, size_t);
    
    private  : void operateOnSimpleVar ( const size_t& _varClass, OperatorFunction _func );
    private  : void operateOnCtrlPtVar ( OperatorFunction _func );
    private  : void operateOnFuncArcVar( OperatorFunction _func );
    
    private  : void computeJacobian ();
    private  : void computeJacobianEntryGeneric(size_t _varClass, size_t _varType, size_t _varIdx);
    
    public   : std::function<double&(size_t        )> state0varAccess_;
    public   : std::function<double&(size_t, size_t)> paramFuncCtrlPtAccess_;
    public   : std::function<double&(size_t, size_t)> paramFuncArcAccess_;
    public   : std::function<double&(size_t        )> paramFuncCtrlPtEndAccess_;
    public   : std::function<double&(size_t        )> paramFuncArcEndAccess_;
    
    protected: std::array<std::vector<size_t>                   , asInt(AccessType::ENUM_SIZE) > varsIdx_;
    private  : std::array<std::function<double&(size_t, size_t)>, asInt(AccessType::ENUM_SIZE) > varsAccess_;
    private  : std::array<size_t                                , asInt(AccessType::ENUM_SIZE) > varsSize_;
    
    private  : TrajectorySimGrade    trajSimGrade_;
    private  : StateSimPtr           stateSim_;
    private  : ParamFuncsSPtr        paramFuncs_;
    private  : OptimizationStateSPtr optState_;
    
//     public   : std::unique_ptr<CostsEvaluatorClass> costsEvaluator_;
};

}

#endif // TRAJECTORY_OPTIMIZER_H