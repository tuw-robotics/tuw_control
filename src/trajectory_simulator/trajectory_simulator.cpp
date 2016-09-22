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

#include <tuw_control/trajectory_simulator/trajectory_simulator.h>
#include <tuw_control/utils.h>

#include <algorithm>

#include <iostream>

using namespace std;
using namespace tuw;

using BSLT = TrajectorySimulator::BaseSimLatticeType;

TrajectorySimulator::TrajectorySimulator (StateSimPtr _stateSim) : 
    stateSim_              (_stateSim), 
    partLattices_          ( asInt(BaseSimLatticeType::LATTICE_ENUM_SIZE) ),
    partLatIdxCache_       ( asInt(BaseSimLatticeType::LATTICE_ENUM_SIZE) ),
    canComputeDistLattice_ ( stateSim_->paramFuncsDist() != nullptr ),
    dt_                    (-1), 
    ds_                    (-1)  {
	for ( size_t i = 0; i < partLattices_.size(); ++i ) { partLattices_[i] = make_shared<LatticeVec>(); }
	*(partLattices_[lattTypeIdx( asInt(BaseSimLatticeType::ARC_BG_BK) )]) = {LatticePoint(), LatticePoint()};
}

TrajectorySimulator::TrajectorySimulator ( StateSimPtr _stateSim, unique_ptr< TrajectorySimulator::CostsEvaluatorClass > _costsEvaluator ) : TrajectorySimulator(_stateSim) {
    costsEvaluator_ = std::move(_costsEvaluator);
    costsEvaluator_->init(partLattices_);
}


void TrajectorySimulator::setUserDefLattice ( const vector< vector< double* > >& _userDefLattices ) {
    userDefPartLattices_ = _userDefLattices;
    partLattices_   .resize(asInt(BaseSimLatticeType::LATTICE_ENUM_SIZE) + _userDefLattices.size()); 
    for(size_t i = asInt(BaseSimLatticeType::LATTICE_ENUM_SIZE); i < partLattices_.size(); ++i) { partLattices_[i] = make_shared<LatticeVec>(); }
    partLatIdxCache_.resize(partLattices_.size());
    for ( size_t i = 0; i < _userDefLattices.size(); ++i ) { partLattices_[lattTypeIdx(i)]->resize( _userDefLattices[i].size() ); }
    updateUserDefLattice();
    if(costsEvaluator_) { costsEvaluator_->init(partLattices_); }
}

void TrajectorySimulator::updateUserDefLattice() {
    
    for ( size_t i = 0; i < userDefPartLattices_.size(); ++i ) {
	for ( size_t j = 0; j < userDefPartLattices_[i].size(); ++j ) { partLattices_[lattTypeIdx(i)]->at(j).arc = *userDefPartLattices_[i][j]; }
    }
}

void TrajectorySimulator::simAppendToSimPartLat ( const double& _arcNow, const int& _latticePtType, const std::size_t& _minArcLatCacheIdx ) {
    if( (simulationLattice_.size() == 0) || ( simulationLattice_.back().arc < _arcNow ) ) {
        stateSim_->advance( _arcNow ); 
        StateSPtr stateNow = stateSim_->cloneState();
        simulationLattice_.emplace_back( LatticePointType( _arcNow, _latticePtType, stateNow ) ); 
    } 
    appendToPartLat( _arcNow, _latticePtType, _minArcLatCacheIdx );
}
void TrajectorySimulator::appendToPartLat ( const double& _arcNow, const int& _latticePtType, const std::size_t& _minArcLatCacheIdx ) {
    auto& partLatticeNow = partLattices_[lattTypeIdx(_latticePtType)]->at(_minArcLatCacheIdx);
    partLatticeNow.arc      = _arcNow;
    partLatticeNow.statePtr = simulationLattice_.back().statePtr;
}

void TrajectorySimulator::setBeginStateToLattices ( const double& _arcBegin ) {
    simAppendToSimPartLat ( _arcBegin, asInt(BSLT::ARC_BG_BK), 0 );
    for ( size_t i = lattTypeIdx(asInt(BSLT::ARC_BG_BK)); i < partLattices_.size(); ++i ) { 
        if ( !partLattices_[i]->empty() ) { partLattices_[i]->at(0).statePtr = simulationLattice_.back().statePtr; }
    }
}

void TrajectorySimulator::setEndStateToLattices ( const double& _arcEnd ) {
    simAppendToSimPartLat ( _arcEnd, asInt(BSLT::ARC_BG_BK), min(partLattices_[lattTypeIdx(asInt(BSLT::ARC_BG_BK))]->size()-1, (size_t)1) );
    for (size_t i = lattTypeIdx(asInt(BSLT::ARC_BG_BK)); i < partLattices_.size(); ++i) { 
        if ( !partLattices_[i]->empty() ) { auto& partLatI0 = partLattices_[i]->back(); partLatI0.statePtr = simulationLattice_.back().statePtr; }
    }
}

void TrajectorySimulator::setBeginEndArcsToLattices ( const double& _arcBegin, const double& _arcEnd ) {
    for ( size_t i = lattTypeIdx(asInt(BSLT::ARC_BG_BK)); i < partLattices_.size(); ++i ) { 
        if ( !partLattices_[i]->empty() ) { partLattices_[i]->at(0).arc = _arcBegin; partLattices_[i]->back().arc = _arcEnd; }
    }
}




bool cmpLatticePt(const TrajectorySimulator::LatticePoint &a, const TrajectorySimulator::LatticePoint &b) { return a.arc < b.arc; }

bool TrajectorySimulator::initSimLatticeState0 ( const double& _lastValidArc, size_t& _firstLaticeInvalidIdx ) {
    if( ( simulationLattice_.size() > 0 ) && (_lastValidArc > 0) ) { 
        _firstLaticeInvalidIdx = std::upper_bound(simulationLattice_.begin(), simulationLattice_.end(), LatticePoint(_lastValidArc), cmpLatticePt) - simulationLattice_.begin(); 
        if ( _firstLaticeInvalidIdx >= simulationLattice_.size() ) { return false; }
    }
    simulationLattice_.erase( simulationLattice_.begin() + _firstLaticeInvalidIdx, simulationLattice_.end() );
    if      ( _firstLaticeInvalidIdx == 0 )                         { stateSim_->toState0(); }
    else if ( _firstLaticeInvalidIdx <= simulationLattice_.size() ) { StateSPtr st = simulationLattice_[_firstLaticeInvalidIdx-1].statePtr->cloneState() ; stateSim_->setState(st); }
    return true;
}

void TrajectorySimulator::initExtLatticeCache( const double& _minArcLatticeVal ) {
    partLatIdxCache_.assign( partLattices_.size(), 0 );
    for ( size_t iPart = extArcLatIdxBegin; iPart < partLattices_.size(); ++iPart ) { 
        if( partLattices_[iPart]->empty() ) { partLatIdxCache_[iPart] = -1; continue; }
        for ( ; partLatIdxCache_[iPart] < (int)partLattices_[iPart]->size(); ++partLatIdxCache_[iPart] ) { 
            if( partLattices_[iPart]->at(partLatIdxCache_[iPart]).arc > _minArcLatticeVal ) { ++partLatIdxCache_[iPart]; break;}
        }
        partLatIdxCache_[iPart] = max(0, (int)partLatIdxCache_[iPart]-1);
    }
}

void TrajectorySimulator::populateTrajSimPartLattice( const size_t& _firstLaticeInvalidIdx ) {
    const double arcParamMax    = stateSim_->paramFuncs()->funcsArcEnd();
    size_t simLatticeSize = max( 0, (int)( ceil( arcParamMax / dt() ) + 1  ) ); 
    
    partLattices_[lattTypeIdx(asInt(BSLT::ARC_BG_BK))]->resize(2);
    //reserve maximum computable lattice points
    for ( size_t i = extArcLatIdxBegin; i < partLattices_.size(); ++i ) { simLatticeSize += partLattices_[i]->size(); } simLatticeSize = max(simLatticeSize, (size_t)0 );
    simulationLattice_.reserve(simLatticeSize);
    
    //compute equal time lattice initial value and multiplier; if init, push_back the 0 lattice point
    double minArcLatticeVal = 0;
    if ( ( _firstLaticeInvalidIdx == 0   ) || ( simulationLattice_.size() == 0 ) ) { setBeginStateToLattices(minArcLatticeVal);        } 
    else                                                                           { minArcLatticeVal = simulationLattice_.back().arc; }
    
    if ( costsEvaluator_ ) { costsEvaluator_->resetCostFunctions(CostEvaluatorCostType::H); }
    
    if (isEmptyAllExtLattices()&&(dt()>0)&&(ds()<=0) ) { populatePartSimLatticesDtOnly  ( _firstLaticeInvalidIdx, arcParamMax );                   } // case when only a dt lattice is available
    else                                               { populatePartSimLatticesGeneral ( _firstLaticeInvalidIdx, arcParamMax, minArcLatticeVal ); } // general case
}

void TrajectorySimulator::populatePartSimLatticesDtOnly( const size_t& _firstLaticeInvalidIdx, double _arcParamMax ) {
    setBeginEndArcsToLattices(0, _arcParamMax);
    
    size_t itEnd = (int)_arcParamMax / dt() + 1;
    //push back DT lattice points
    for (size_t aPLIdx = _firstLaticeInvalidIdx+1; aPLIdx < itEnd; ++aPLIdx ) { simAppendToSimPartLat ( aPLIdx * dt(), asInt(BSLT::LATTICE_ARC_EQ_DT), aPLIdx); } 
    
    //push back final lattice point
    setEndStateToLattices(_arcParamMax);
}


size_t TrajectorySimulator::lattTypeIdx ( int _enumIdx ) {
    return _enumIdx + asInt(BaseSimLatticeType::LATTICE_ENUM_SIZE); 
}

bool TrajectorySimulator::isEmptyAllExtLattices() const {
    bool emptyExtArcLat = true;
    for ( size_t i = extArcLatIdxBegin; i < partLattices_.size(); ++i ) { if( !partLattices_[i]->empty() ) { emptyExtArcLat = false; break; } }
    return emptyExtArcLat;
}

double& TrajectorySimulator::dt() {
    return dt_;
}
const double& TrajectorySimulator::dt() const {
    return dt_;
}
double& TrajectorySimulator::ds() {
    return ds_;
}
const double& TrajectorySimulator::ds() const {
    return ds_;
}
StateSimPtr TrajectorySimulator::stateSim() {
    return stateSim_;
}
const StateSimPtr TrajectorySimulator::stateSim() const {
    return stateSim_;
}
vector< TrajectorySimulator::LatticePointType >& TrajectorySimulator::simLattice() {
    return simulationLattice_;
}
const vector< TrajectorySimulator::LatticePointType >& TrajectorySimulator::simLattice() const {
    return simulationLattice_;
}