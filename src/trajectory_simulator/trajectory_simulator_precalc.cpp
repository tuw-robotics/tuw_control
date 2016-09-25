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

#include <tuw_control/trajectory_simulator/trajectory_simulator_precalc.h>
#include <tuw_control/utils.h>

#include <algorithm>
#include <iostream>

using namespace std;
using namespace tuw;

using BSLT = TrajectorySimulator::BaseSimLatticeType;

TrajectorySimulatorPrecalc::TrajectorySimulatorPrecalc (StateSimPtr _stateSim) : 
    TrajectorySimulator(_stateSim) {
}
TrajectorySimulatorPrecalc::TrajectorySimulatorPrecalc ( StateSimPtr _stateSim, unique_ptr< CostsEvaluatorClass > _costsEvaluator ) : 
    TrajectorySimulator ( _stateSim, std::move(_costsEvaluator) ) {

}


void TrajectorySimulatorPrecalc::simulateTrajectory( double _lastValidArc ) {  
    
    updateUserDefLattice();
    
    //resize equal dt lattice
    const double arcParamMax    = stateSim_->paramFuncs()->funcsArcEnd();
    size_t simLatticeSize = max( 0, (int)( ceil( arcParamMax / dt() ) + 1  ) ); 
    partLattices_[lattTypeIdx(asInt(BSLT::LATTICE_ARC_EQ_DT))]->resize(simLatticeSize, LatticePoint(FLT_MAX) );
    
    //set the dist-extended sim lattice points on the last lattice entries
    if ( canComputeDistLattice_ && ( ds() > 0 ) ) {
        static vector<double> dsLattice;
        stateSim_->paramFuncsDist()->computeS2TLattice( _lastValidArc, ds(), dsLattice ); 
        auto& partLatticeDs = partLattices_[lattTypeIdx(asInt(BSLT::LATTICE_ARC_EQ_DS))]; 
	
        const double& firstDsLattice = dsLattice[0]; 
        size_t idxFirstInvalidDs = max(0, (int)partLatticeDs->size() - 2);
        for ( size_t i = 1; i < partLatticeDs->size(); ++i ) { if ( partLatticeDs->at(i).arc > firstDsLattice + 1e-3 ) { idxFirstInvalidDs = --i; break; } }
        partLatticeDs->resize( idxFirstInvalidDs + dsLattice.size() ); for ( size_t i = 0; i < dsLattice.size(); ++i ) { partLatticeDs->at(i+idxFirstInvalidDs).arc = dsLattice[i]; }
        
    } else {
	partLattices_[lattTypeIdx(asInt(BSLT::LATTICE_ARC_EQ_DS))]->clear();
    }
    
    size_t firstLaticeInvalidIdx = 0;
    if ( initSimLatticeState0(_lastValidArc, firstLaticeInvalidIdx) ) {
	populateTrajSimPartLattice( firstLaticeInvalidIdx ); 
	if ( costsEvaluator_ ) { costsEvaluator_->evaluateAllCosts(); } 
    }
}

void TrajectorySimulatorPrecalc::populatePartSimLatticesGeneral( size_t _firstLaticeInvalidIdx, double _arcParamMax, double _minArcLatticeVal ) {
    setBeginEndArcsToLattices(0, _arcParamMax);
    initExtLatticeCache(_minArcLatticeVal);
    
    size_t arcParamLatticeIdx = max( 0, (int)( _minArcLatticeVal / dt() ) );
    size_t minArcLatCacheIdx = getMinArcLatCacheIdx();//find lattice type index of next smallest required simulation lattice value
    while ( ( _minArcLatticeVal = partLattices_[minArcLatCacheIdx]->at(partLatIdxCache_[minArcLatCacheIdx]).arc ) < _arcParamMax ) {
        const size_t deltaArcParamLattice = max( 0, (int)( _minArcLatticeVal / dt() ) - (int)arcParamLatticeIdx);
        const size_t simLatticeInjectEnd = ++_firstLaticeInvalidIdx + deltaArcParamLattice;

        for ( ; _firstLaticeInvalidIdx < simLatticeInjectEnd; ++_firstLaticeInvalidIdx ) { //push_back the equal time lattice points before the extended one first
            simAppendToSimPartLat ( ++arcParamLatticeIdx * dt(), asInt(BSLT::LATTICE_ARC_EQ_DT), arcParamLatticeIdx);
        } 
        simAppendToSimPartLat ( _minArcLatticeVal, (int)minArcLatCacheIdx - asInt(BSLT::LATTICE_ENUM_SIZE), partLatIdxCache_[minArcLatCacheIdx] ); //push back extended lattice point
        
        int& idxMinLatticePt = partLatIdxCache_[minArcLatCacheIdx]; 
        if ( idxMinLatticePt + 1 < (int)partLattices_[minArcLatCacheIdx]->size() ) { ++idxMinLatticePt; }
        minArcLatCacheIdx = getMinArcLatCacheIdx();//update the minimum extended lattice point
    }
    const double simLatticeInjectEnd = _arcParamMax / dt(); ++arcParamLatticeIdx;
    for (; arcParamLatticeIdx < simLatticeInjectEnd; ++arcParamLatticeIdx ) { 
        simAppendToSimPartLat ( arcParamLatticeIdx * dt(), asInt(BSLT::LATTICE_ARC_EQ_DT), arcParamLatticeIdx);
    } 
    
    setEndStateToLattices(_arcParamMax);
}


size_t TrajectorySimulatorPrecalc::getMinArcLatCacheIdx () const {
    size_t idxMin = 0;
    double minArc = FLT_MAX;
    for (  size_t iPart = extArcLatIdxBegin; iPart < partLattices_.size(); ++iPart ) {
	if( partLattices_[iPart]->empty() ) { continue; }
	const double& arcI = partLattices_[iPart]->at(partLatIdxCache_[iPart]).arc;
	if( minArc > arcI ){ minArc = arcI; idxMin = iPart;  }
    }
    return idxMin;
}
