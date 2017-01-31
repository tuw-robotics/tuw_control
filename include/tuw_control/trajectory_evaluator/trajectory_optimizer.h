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
	OptStateData ( double* _val, const double* _arcBegin = 0 ) : val(_val), arcBegin(_arcBegin){}
	virtual ~OptStateData() = default;
	double* val;
	const double* arcBegin;
    };
    
    public   : OptimizationState           ()                         = default;
    public   : ~OptimizationState          ()                         = default;
    public   : OptimizationState           (const OptimizationState&) = default;
    public   : OptimizationState& operator=(const OptimizationState&) = default;
    public   : OptimizationState           (OptimizationState&&)      = default;
    public   : OptimizationState& operator=(OptimizationState&&)      = default;
    
    public   : virtual void bindVariables(TrajectorySimulator& _trajOpt) = 0;
    public   : double&       value      ( const std::size_t& _i )       override { return *variables[_i].val; }
    public   : const double& value      ( const std::size_t& _i ) const override { return *variables[_i].val; }
    public   : size_t        valueSize  () const override { return variables.size(); }
    
    public   : const double& arcBegin   ( const std::size_t& _i ) const { return *variables[_i].arcBegin; }
    
    protected: std::vector<OptStateData>     variables;
    protected: static constexpr const double valueZero = 0;
};
    
    
class TrajectoryOptimizer;
using TrajectoryOptimizerSPtr      = std::shared_ptr<TrajectoryOptimizer>;
using TrajectoryOptimizerConstSPtr = std::shared_ptr<TrajectoryOptimizer const>;
using TrajectoryOptimizerUPtr      = std::unique_ptr<TrajectoryOptimizer>;
using TrajectoryOptimizerConstUPtr = std::unique_ptr<TrajectoryOptimizer const>;

class TrajectoryOptimizer : public TrajectorySimGrade {
    public  : enum class AccessType {
	STATE_0,
	PARAM_CP,
	PARAM_ARC,
	ENUM_SIZE
    };
    
    //special class member functions
    public   : TrajectoryOptimizer           ( StateSimPtr& _stateSim, std::unique_ptr<TrajectorySimulator::CostsEvaluatorClass> _costsEvaluator, OptimizationStateSPtr _optState );
    public   : ~TrajectoryOptimizer          ()                           = default;
    public   : TrajectoryOptimizer           (const TrajectoryOptimizer&) = default;
    public   : TrajectoryOptimizer& operator=(const TrajectoryOptimizer&) = default;
    public   : TrajectoryOptimizer           (TrajectoryOptimizer&&)      = default;
    public   : TrajectoryOptimizer& operator=(TrajectoryOptimizer&&)      = default;
    
    public   : virtual void optimize();
    public   : virtual void initState0ParamFuncsHValid (const size_t& _optFailCount);
    public   : void computeJacobian ( bool _efficient = true);
    public   : void computeJacobian1Entry ( size_t _idx, bool _efficient = true);
    public   : double& stepSize();
    
    public   : OptimizationStateSPtr optState_;
    public   : double fCache;
    public   : std::vector<double> hCache;
    public   : std::vector<double> gCache;
    private  : std::vector< TrajectorySimulator::LatticePointType > simulationLattice;
    private  : TrajectorySimulator::LatticeVecSPtrVec               partLattices;
    private  : double stepSize_;
};

}

#endif // TRAJECTORY_OPTIMIZER_H