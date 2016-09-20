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

#include <tuw_control/trajectory_simulator/trajectory_simulator_online.h>
#include <tuw_control/utils.h>

#include <algorithm>

using namespace std;
using namespace tuw;

TrajectorySimulatorOnline::TrajectorySimulatorOnline (StateSimPtr _stateSim) : 
    TrajectorySimulator(_stateSim) {
}
TrajectorySimulatorOnline::TrajectorySimulatorOnline ( StateSimPtr _stateSim, unique_ptr< CostsEvaluatorClass > _costsEvaluator ) : 
    TrajectorySimulator ( _stateSim, std::move(_costsEvaluator) ) {

}


using BSLT = TrajectorySimulator::BaseSimLatticeType;

void TrajectorySimulatorOnline::simulateTrajectory( double _lastValidArc ) {
    _lastValidArc = 0;
    size_t firstLaticeInvalidIdx = 0;
    if ( initSimLatticeState0(_lastValidArc, firstLaticeInvalidIdx) ) { 
	
	populateTrajSimPartLattice( firstLaticeInvalidIdx ); 
	
	if ( costsEvaluator_ ) { costsEvaluator_->evaluateAllCosts(); }
    }
}

double TrajectorySimulatorOnline::toNextIntMult ( const double& _x, const double& _dx ) const {
    static double add, ans; static double temp;
    temp = ceil( _x / _dx );
    ans = temp * _dx; 
    add = 0;
    while ( _x >= ans /*- 1e-8*/ ) { ans = (++add + temp) * _dx; };
    return ans;
}

void TrajectorySimulatorOnline::populatePartSimLatticesGeneral(size_t _firstLaticeInvalidIdx, double _arcParamMax, double _minArcLatticeVal) {
    resizeBeginGeneral(_arcParamMax);
    setBeginEndArcsToLattices(0, _arcParamMax);
    initExtLatticeCache(_minArcLatticeVal);
    
    double& arcLatNextMin  = _minArcLatticeVal = 0;
    double  arcLatEqDtNext = 0;
    double  arcLatEqDsNext = 0;
    double  arcLatOldMin   = arcLatNextMin;
    StateSPtr newState;
    while ( arcLatNextMin < _arcParamMax ) { 
	
	// computing next equal arc lattice value
	arcLatEqDtNext = toNextIntMult( stateSim_->stateArc(), dt() ); 
	arcLatNextMin = arcLatEqDtNext;
	
	// computing next dist lattice value
	arcLatEqDsNext = FLT_MAX;
	if ( canComputeDistLattice_ && ( ds() > 0 ) ) {
	    static double arcTemp; arcTemp = stateSim_->stateArc();
	    double  distLatticeNext = toNextIntMult( stateSim_->stateDist(), ds() );
	    stateSim_->paramFuncsDist()->setEvalDist(distLatticeNext);
	    arcLatEqDsNext = stateSim_->paramFuncs()->funcsArcEval(); arcLatNextMin = fmin(arcLatNextMin, arcLatEqDsNext);
	    stateSim_->paramFuncs()->setEvalArc(arcTemp);
	}
	
	// computing next extended (i.e. equal knots) lattice values
	for ( size_t iPart = lattTypeIdx(0); iPart < partLattices_.size(); ++iPart ) { 
            if( partLattices_[iPart]->empty() ) { partLatIdxCache_[iPart] = -1; continue; }
            for ( ; partLatIdxCache_[iPart] < (int)partLattices_[iPart]->size(); ++partLatIdxCache_[iPart] ) { 
                if( partLattices_[iPart]->at(partLatIdxCache_[iPart]).arc > arcLatOldMin ) { ++partLatIdxCache_[iPart]; break;}
            }
            partLatIdxCache_[iPart] = max(0, partLatIdxCache_[iPart]-1);
            arcLatNextMin = fmin(arcLatNextMin, partLattices_[iPart]->at(partLatIdxCache_[iPart]).arc );
        }
	
	// simulating till next lattice point
	if (arcLatNextMin == 0) { continue; }
        static constexpr const int uninitLattTypeVal = -100;
	if ( arcLatNextMin <= _arcParamMax ) {
	    stateSim_->advance( arcLatNextMin );
	    newState = stateSim_->cloneState();
            simulationLattice_.emplace_back( LatticePointType( arcLatNextMin, uninitLattTypeVal, newState ) );
        }
        
        for ( size_t iPart = lattTypeIdx(asInt(BSLT::LATTICE_ARC_EQ_DT)); iPart < partLattices_.size(); ++iPart ) { 
	    if( partLattices_[iPart]->empty() ) { continue; }
	    if ( ( ( iPart == lattTypeIdx(asInt(BSLT::LATTICE_ARC_EQ_DT) ) ) && (fabs(arcLatNextMin - arcLatEqDtNext )<=FLT_MIN) ) ||
	         ( ( iPart == lattTypeIdx(asInt(BSLT::LATTICE_ARC_EQ_DS) ) ) && (fabs(arcLatNextMin - arcLatEqDsNext )<=FLT_MIN) ) ) { 
		if( arcLatNextMin < _arcParamMax ) {
		    partLattices_[iPart]->resize(partLattices_[iPart]->size()+1, LatticePoint(_arcParamMax)); 
		    partLattices_[iPart]->at(partLattices_[iPart]->size()-2) = LatticePoint(arcLatNextMin, newState); 
		} else {
		    partLattices_[iPart]->back() = LatticePoint(arcLatNextMin, newState); 
		}
		if( simulationLattice_.back().latticeType == uninitLattTypeVal ) { simulationLattice_.back().latticeType = iPart - asInt(BSLT::LATTICE_ENUM_SIZE); }
	    } else {
		int&                               extArcIdxCacheI = partLatIdxCache_[iPart];
		TrajectorySimulator::LatticePoint& partLattII      = partLattices_[iPart]->at(extArcIdxCacheI);
		if ( fabs(arcLatNextMin - partLattII.arc ) <= FLT_MIN ) { 
		    if( simulationLattice_.back().latticeType == uninitLattTypeVal ) { simulationLattice_.back().latticeType = iPart - asInt(BSLT::LATTICE_ENUM_SIZE); }
		    partLattII.statePtr = newState; if( extArcIdxCacheI + 1 < (int)partLattices_[iPart]->size()  ){ ++extArcIdxCacheI; }
		    partLattII.arc = arcLatNextMin;
		}
	    }
	}
	
	if ( costsEvaluator_ /*&& !firstCycle*/ /*&& (arcLatOldMin < arcLatNextMin)*/ ) { 
	    size_t tempDummy; double temppDummy;
	    if( !costsEvaluator_->evalValidCostStep(CostEvaluatorCostType::H, arcLatNextMin, tempDummy, temppDummy ) ) { 
		simulationLattice_.pop_back(); _arcParamMax = simulationLattice_.back().arc;
		for(size_t i = lattTypeIdx(asInt(BSLT::LATTICE_ARC_EQ_DT)); i < partLattices_.size(); ++i ) { 
		    if( partLattices_[i]->empty() ) { continue; }
		    partLattices_[i]->pop_back();
		}
		for(size_t i = lattTypeIdx(asInt(BSLT::LATTICE_ARC_EQ_DT)); i < lattTypeIdx(0); ++i ) { 
		    if( partLattices_[i]->empty() ) { continue; }
		    partLattices_[i]->pop_back();
		}
		
		break;
	    }
	}
	
	arcLatOldMin = arcLatNextMin;
    }
//     std::cout<<"simLattX:"; for(auto simLatI : simulationLattice_){ cout<<simLatI.statePtr->state(0)<<", "; }cout<<endl;
    if( simulationLattice_.empty() ){ return; }
    newState   = simulationLattice_.back().statePtr->cloneState();
    double arc = simulationLattice_.back().arc;
    for(size_t iPart = lattTypeIdx(asInt(BSLT::ARC_BG_BK)); iPart < partLattices_.size(); ++iPart ) { 
	if( partLattices_[iPart]->empty() ) { continue; }
	partLattices_[iPart]->back().statePtr  = newState;
	partLattices_[iPart]->back().arc       = arc;
    }
}

void TrajectorySimulatorOnline::resizeBeginGeneral( const double& _arcParamMax ) {
    size_t idxVecInit;
    StateSPtr& newState = simulationLattice_.back().statePtr;
    idxVecInit = lattTypeIdx(asInt(BSLT::LATTICE_ARC_EQ_DT)); 
    if ( (dt() > 0) ) { 
	partLattices_[idxVecInit]->assign(max(2, partLatIdxCache_[idxVecInit]+1), LatticePoint(_arcParamMax) ); 
	partLattices_[idxVecInit]->at(0) = LatticePoint(0, newState);
    }
    idxVecInit = lattTypeIdx(asInt(BSLT::LATTICE_ARC_EQ_DS)); 
    if ( (ds() > 0) ) { 
	partLattices_[idxVecInit]->assign(max(2, partLatIdxCache_[idxVecInit]+1), LatticePoint(_arcParamMax) ); 
	partLattices_[idxVecInit]->at(0) = LatticePoint(0, newState);
    }
}


void TrajectorySimulatorOnline::populatePartSimLatticesDtOnly ( const size_t& _firstLaticeInvalidIdx, double _arcParamMax ) {
    resizeBeginDtOnly(_arcParamMax);
    setBeginEndArcsToLattices(0, _arcParamMax);
    
    size_t itEnd = (int)_arcParamMax / dt() + 1;
    //push back DT lattice points
    for (size_t aPLIdx = _firstLaticeInvalidIdx+1; aPLIdx < itEnd; ++aPLIdx ) { 
	double arcLatNextMin = aPLIdx * dt();
	simAppendToSimPartLat ( arcLatNextMin, asInt(BSLT::LATTICE_ARC_EQ_DT), aPLIdx); 
	if ( costsEvaluator_ ) { 
	    size_t latKnotMin; 
	    if( !costsEvaluator_->evalValidCostStep(CostEvaluatorCostType::H, arcLatNextMin, latKnotMin, _arcParamMax ) ) { 
		simulationLattice_.pop_back(); 
		size_t i = 1; if(_arcParamMax == 0){ i = 0; }
		for(; i < partLattices_.size(); ++i){
		    if( partLattices_[i]->empty() ) { continue; }
		    size_t firstInvalidIdx = distance( partLattices_[i]->begin(), 
						       upper_bound( partLattices_[i]->begin(), 
								    partLattices_[i]->end(), 
								    _arcParamMax,
								    [](const TrajectorySimulator::LatticePoint& a, const TrajectorySimulator::LatticePoint& b){ return a.arc < b.arc; }
								  ) 
						     ) /*+ 1 * !(i == latKnotMin)*/;
		    partLattices_[i]->erase( partLattices_[i]->begin()+min(firstInvalidIdx, partLattices_[i]->size()), partLattices_[i]->end() );
		}
// 		partLattices_[lattTypeIdx(asInt(BSLT::LATTICE_ARC_EQ_DT))]->resize( aPLIdx );
		break;
	    }
	}
    } 
    
    //push back final lattice point
    setBeginEndArcsToLattices(0, _arcParamMax);///@todo inefficient lol
    setEndStateToLattices(_arcParamMax);
}

void TrajectorySimulatorOnline::resizeBeginDtOnly( const double& _arcParamMax ) {
    size_t simLatticeSize = max( 0, (int)( ceil( _arcParamMax / dt() ) + 1  ) ); 
    auto& partLatTime = partLattices_[lattTypeIdx(asInt(BSLT::LATTICE_ARC_EQ_DT))];
    partLatTime->assign(simLatticeSize, LatticePoint(0) );
    for(size_t i = 0; i < partLatTime->size(); ++i) { partLatTime->at(i).arc = i * dt(); }
    partLattices_[lattTypeIdx(asInt(BSLT::LATTICE_ARC_EQ_DT))]->at(0) = LatticePoint(0, simulationLattice_[0].statePtr);
}

