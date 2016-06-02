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

#include <tuw_control/param_func/param_func_spline/param_func_spline0_dist.h>
#include <tuw_control/utils.h>

#include <float.h>
#include <math.h>
#include <algorithm>


using namespace tuw;
using namespace std;

using TDM = ParamFuncsSpline0Dist::TraveledDistCfMode;

void ParamFuncsSpline0Dist::setDistCfMode ( TraveledDistCfMode _distCfMode, const vector< std::size_t >& _distRelFuncIdx ) {
    distCfMode_ = _distCfMode;
    switch (distCfMode_) {
	case TDM::NONE: break;
	case TDM::V   : 
	    distLinkedFuncIdx_.resize(1, _distRelFuncIdx[0]); 
	    distEvalCache_.resize(funcCtrlPt_[_distRelFuncIdx[0]].size(), 0); 
	    computeSFuncPtr_ = &ParamFuncsSpline0Dist::computeS_V;  
	    computeDs2DtPtr_ = &ParamFuncsSpline0Dist::computeT_V;
	    distLinkedArcIdx_= func2Arc_[_distRelFuncIdx[0]];
	    break;
	case TDM::AV  : 
	    distLinkedFuncIdx_.resize(1, _distRelFuncIdx[0]); 
	    distEvalCache_.resize(funcCtrlPt_[_distRelFuncIdx[0]].size(), 0); 
	    computeSFuncPtr_ = &ParamFuncsSpline0Dist::computeS_AV; 
	    computeDs2DtPtr_ = &ParamFuncsSpline0Dist::computeT_AV; 
	    distLinkedArcIdx_= func2Arc_[_distRelFuncIdx[0]];
	    break;
    }
}

void ParamFuncsSpline0Dist::precomputeDist() {
    if( distCfMode_ == TDM::NONE ) { return; }
    
    const size_t& func0Idx = distLinkedFuncIdx_[0];
    const size_t  ctrlPtSize = funcCtrlPt_[func0Idx].size()-1;
    distEvalCache_[0] = 0;
    for( size_t funcCtrlPtIdx = 0; funcCtrlPtIdx < ctrlPtSize; ++funcCtrlPtIdx ) { 
	funcsArcEval_ = ctrlPtVal( func0Idx, funcCtrlPtIdx+1, CtrlPtDim::ARC );
	funcEvalArcCacheIdxUnder_[distLinkedArcIdx_] = funcCtrlPtIdx;
	distEvalCache_[funcCtrlPtIdx+1] = computeS ();
    }
    setEvalArc(funcsArcBegin_, EvalArcGuarantee::AT_BEGIN);
}


double ParamFuncsSpline0Dist::computeS () const {
    return (this->*computeSFuncPtr_)();
}
double ParamFuncsSpline0Dist::computeS_V() const {
    const size_t& funcIdx          = distLinkedFuncIdx_[0];
    const FuncCacheData& cacheData = funcEvalCache_[funcIdx][funcEvalArcCacheIdxUnder_[func2Arc_[funcIdx]]];//*funcEvalCacheIter_[funcIdx];
    return distEvalCache_[funcEvalArcCacheIdxUnder_[distLinkedArcIdx_]] + computeDeltaS_V_AV( funcsArcEval_ - cacheData.arc, cacheData.val, computeFuncDiff1(funcIdx) ); 
}

double ParamFuncsSpline0Dist::computeS_AV() const {
const size_t& funcIdx          = distLinkedFuncIdx_[0];
    const FuncCacheData& cacheData = funcEvalCache_[funcIdx][funcEvalArcCacheIdxUnder_[func2Arc_[funcIdx]]];//*funcEvalCacheIter_[funcIdx];
    return distEvalCache_[funcEvalArcCacheIdxUnder_[distLinkedArcIdx_]] + computeDeltaS_V_AV( funcsArcEval_ - cacheData.arc, computeFuncInt1(funcIdx), cacheData.val  );
}

void ParamFuncsSpline0Dist::computeS2TLattice ( const double& _arc0, const double& _ds, vector< double >& _tLattice ) {
    setEvalArc ( _arc0, EvalArcGuarantee::NONE );
    _tLattice.clear(); _tLattice.reserve( distEvalCache_.back() / _ds );
    const size_t idxBeforeStart = static_cast<int>( computeS() / _ds );
    
    size_t i = 0;
    setEvalDist ( _ds * ( ++i + idxBeforeStart ), EvalArcGuarantee::AFTER_LAST );
    while( funcsArcEval_ < funcsArcEnd_ ){ 
	_tLattice.emplace_back(funcsArcEval_);
	setEvalDist ( _ds * ( ++i + idxBeforeStart ), EvalArcGuarantee::AFTER_LAST );
    }
    _tLattice.emplace_back(funcsArcEnd_);
    setEvalArc(funcsArcBegin_, EvalArcGuarantee::AT_BEGIN);
}


void ParamFuncsSpline0Dist::computeS2TLattice ( const vector< double >& _sLattice, vector< double >& _tLattice ) {
    const size_t slSize = _sLattice.size();
    _tLattice.clear(); _tLattice.reserve( _sLattice.size() );
    
    if( slSize > 0 ){
	size_t i = 0;
	setEvalDist ( _sLattice[i], EvalArcGuarantee::NONE );
	while( ( funcsArcEval_ < funcsArcEnd_ ) && ( i+1 < slSize ) ){ 
	    _tLattice.emplace_back(funcsArcEval_);
	    setEvalDist ( _sLattice[++i], EvalArcGuarantee::AFTER_LAST ); 
	}
    }
    _tLattice.emplace_back(funcsArcEnd_);
    setEvalArc ( funcsArcBegin_, EvalArcGuarantee::AT_BEGIN );
}
void ParamFuncsSpline0Dist::setEvalDist ( const double& _funcsDistEval, const EvalArcGuarantee& _evalArcGuarantee ) {    
    setEvalArc( computeTImpl( _funcsDistEval, _evalArcGuarantee ), _evalArcGuarantee );
}

double ParamFuncsSpline0Dist::computeT ( const double& _s, const EvalArcGuarantee& _evalArcGuarantee ) {
    size_t& arcIdx  = funcEvalArcCacheIdxUnder_[distLinkedArcIdx_];
    
    const size_t cacheCtrlPtIdxRestore = arcIdx;
    const size_t cacheArcEvalRestore   = funcsArcEval_;
    
    double resultT = computeTImpl( _s, _evalArcGuarantee );
    
    arcIdx        = cacheCtrlPtIdxRestore;
    funcsArcEval_ = cacheArcEvalRestore;
    
    return resultT;
}

double ParamFuncsSpline0Dist::computeTImpl ( const double& _s, const EvalArcGuarantee& _evalArcGuarantee ) {
    using eag = EvalArcGuarantee;
    size_t& arcCacheIdx  = funcEvalArcCacheIdxUnder_[distLinkedArcIdx_];
    const size_t ctrlPtSize = funcCtrlPt_[distLinkedFuncIdx_[0]].size();
    
    switch (_evalArcGuarantee) {
	case eag::AFTER_LAST : while( distEvalCache_[arcCacheIdx] <= _s ){ if( ++arcCacheIdx >= ctrlPtSize) { break; } } --arcCacheIdx; break;
	case eag::AT_BEGIN   : arcCacheIdx = 0; break;
	case eag::AT_END     : arcCacheIdx = funcCtrlPt_[distLinkedFuncIdx_[0]].size()-1; break;
	case eag::NONE       : arcCacheIdx = distance( distEvalCache_.begin(), upper_bound( distEvalCache_.begin(), distEvalCache_.end(), _s ) ) - 1; break;
	case eag::BEFORE_LAST: while( distEvalCache_[arcCacheIdx] >= _s ){ if( arcCacheIdx   == 0         ) { break; } --arcCacheIdx; } break;
    }
    funcsArcEval_ = ctrlPtVal(distLinkedFuncIdx_[0], arcCacheIdx, CtrlPtDim::ARC );
    
    return funcsArcEval_ + (this->*computeDs2DtPtr_)( _s - distEvalCache_[arcCacheIdx] );
}
double ParamFuncsSpline0Dist::computeT_V  ( const double& _ds ) const {
    const size_t& funcIdx  = distLinkedFuncIdx_[0];
    return computeDeltaT_V_AV( _ds, computeFuncVal( funcIdx ), computeFuncDiff1( funcIdx ) );
}
double ParamFuncsSpline0Dist::computeT_AV ( const double& _ds ) const {
    const size_t& funcIdx  = distLinkedFuncIdx_[0];
    return computeDeltaT_V_AV( _ds, computeFuncInt1 (funcIdx), computeFuncVal  ( funcIdx ) );
}

double ParamFuncsSpline0Dist::computeDeltaS_V_AV ( const double& _dt, const double& _v0, const double& _av ) {
    const double dtAbs = fabs(_dt); if(      dtAbs < FLT_MIN ) { return 0; }
    const double vAbs  = fabs(_v0); if(  fabs(_av) < FLT_MIN ) { return vAbs * _dt; }
    const double v1 = _v0 + _av * dtAbs;
    if ( signum(v1) == signum(_v0) ) { 
	return fabs(_v0 + v1) * _dt / 2.;
    } else { 
	int retSign = 1; if( _dt < 0 ) { retSign = -1; }
	double dt1 = - _v0 / _av; 
	return retSign * ( vAbs * dt1 + fabs( v1 ) * (dtAbs - dt1) ) / 2.;  
    }
}
double ParamFuncsSpline0Dist::computeDeltaT_V_AV ( const double& _ds, const double& _v0, const double& _av ) {
    double v0 = _v0, av = _av;
    if( v0 < 0 ) { av *= -1; v0 *= -1; }
    if( fabs(av) < FLT_MIN ){ return _ds / v0; }
    
    int retSign = -1; if( _ds < 0 ) { retSign = +1; }
    const double v0v0  = v0 * v0;
    
    double delta = v0v0 + 2. * av * fabs( _ds );
    if ( delta < 0 ){  return retSign * ( + v0 + sqrt(-delta) ) / av; } 
    else            {  return retSign * ( + v0 - sqrt( delta) ) / av; }
}