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
      TrajectorySimGrade(_stateSim, std::move(_costsEvaluator) ), 
      optState_         (_optState) {
	  optState_->bindVariables( *trajSim() );
}

void TrajectoryOptimizer::optimize() {
    throw "not implemented";
}

double& TrajectoryOptimizer::stepSize() {
    return stepSize_;
}
void TrajectoryOptimizer::computeJacobian( bool _efficient ) {
    
    evaluateTrajectory(0);
    if(_efficient){
	simulationLattice.reserve(trajSim()->simLattice().size());
	for(size_t i = 0; i < simulationLattice.size(); ++i){
	    auto statePtr = trajSim()->simLattice().at(i).statePtr->cloneState();
	    simulationLattice.emplace_back(TrajectorySimulator::LatticePointType( trajSim()->simLattice().at(i).arc, 
										  trajSim()->simLattice().at(i).latticeType,
										  statePtr ) );
	}
	partLattices.resize( trajSim()->partLattices_.size() );
	for(size_t i = 0; i < partLattices.size(); ++i){
	    partLattices[i] = std::make_shared< TrajectorySimulator::LatticeVec >(*trajSim()->partLattices_[i]);
	}
    }
    
    auto& costsEvaluator = trajSim()->costsEvaluator_;
    
    costsEvaluator->gradF.resize( optState_->valueSize() );
    costsEvaluator->gradG.resize( costsEvaluator->g.size(), optState_->valueSize() );
    costsEvaluator->gradH.resize( costsEvaluator->h.size(), optState_->valueSize() );
    
    fCache = costsEvaluator->f;
    gCache = costsEvaluator->g;
    hCache = costsEvaluator->h;
    
    for ( size_t i = 0; i < optState_->valueSize(); ++i ) {
	computeJacobian1Entry( i, _efficient );
    }
    
    costsEvaluator->f = fCache;
    costsEvaluator->g = gCache;
    costsEvaluator->h = hCache;
}

void TrajectoryOptimizer::computeJacobian1Entry ( std::size_t _idx, bool _efficient ) {
    
    if(_efficient) {
	trajSim()->simLattice()  = simulationLattice;
	for(size_t i = 0; i < partLattices.size(); ++i){
	    trajSim()->partLattices_[i] = std::make_shared< TrajectorySimulator::LatticeVec >(*partLattices[i]);
	}
	trajSim()->costsEvaluator_->init(trajSim()->partLattices_);
	
	
    }
    
    const double optStateIFix = optState_->value(_idx);
    optState_->value(_idx) += stepSize_;
    
    
    evaluateTrajectory( _efficient * optState_->arcBegin(_idx) );
    
    auto& costsEvaluator = trajSim()->costsEvaluator_;
    costsEvaluator->gradF[_idx] = (costsEvaluator->f - fCache) / stepSize_;
    for ( int j = 0; j < costsEvaluator->gradG.rows(); ++j ){ costsEvaluator->gradG(j,_idx) = ( costsEvaluator->g[j] - gCache[j] ) / stepSize_; }
    for ( int j = 0; j < costsEvaluator->gradH.rows(); ++j ){ costsEvaluator->gradH(j,_idx) = ( costsEvaluator->h[j] - hCache[j] ) / stepSize_; }
    
    optState_->value(_idx) = optStateIFix;
}

