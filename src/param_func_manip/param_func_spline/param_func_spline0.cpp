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
#include <limits>
#include <vector>

using namespace tuw;
using namespace std;

void ParamFuncsSpline0Dist::initImpl() {
    const size_t funcSize = funcsSize();
    funcEvalCache_           .resize( funcSize       );  for( auto& funcEvalCacheI            : funcEvalCache_            ) { funcEvalCacheI           .clear(); }
    arc2func_                .resize( funcsArcSize() );  for( auto& arc2funcI                 : arc2func_                 ) { arc2funcI                .clear(); }
    funcEvalArcCacheIdxUnder_.resize( funcsArcSize() );
    
    for( size_t funcIdx = 0; funcIdx < funcSize; ++funcIdx ) {
	const size_t funcISize = funcCtrlPt_[funcIdx].size();
	funcEvalReq_[funcIdx][asInt(FuncEvalMode::DIFF1)] = true ;//force caching of DIFF1, used by computeFuncVal
	funcEvalReq_[funcIdx][asInt(FuncEvalMode::DIFF2)] = false;//second function derivative is allways 0
	for( size_t ctrlPtIdx = 0; ctrlPtIdx < funcISize; ++ctrlPtIdx ) {
	    funcEvalCache_[funcIdx].emplace_back( FuncCacheData( funcCtrlPt_[funcIdx][ctrlPtIdx] ) ); 
	}
	arc2func_[func2Arc_[funcIdx]].emplace_back(funcIdx);
    }
}

void ParamFuncsSpline0Dist::precompute() {
    using fem = FuncEvalMode;
    
    for( size_t arcIdx = 0; arcIdx < funcsArcSize(); ++arcIdx ) { 
	for( size_t arcKnot = 1; arcKnot < funcsArcSize(arcIdx); ++arcKnot ) { 
	    funcsArc(arcIdx, arcKnot) = fmax( funcsArc(arcIdx, arcKnot), funcsArc(arcIdx, arcKnot-1) );
	}
    }
    
    const double funcSize = funcCtrlPt_.size();
    for( size_t funcIdx = 0; funcIdx < funcSize; ++funcIdx ) {
	
	auto& funcIEvalCache        = funcEvalCache_[funcIdx];
	auto  funcIEvalCacheIter    = funcIEvalCache.begin();
	funcIEvalCacheIter->cache[asInt(fem::INT1)] = 0;
	funcIEvalCacheIter->cache[asInt(fem::INT2)] = 0;
	const auto& endIter = funcIEvalCache.end()-1;
	
	funcEvalArcCacheIdxUnder_[func2Arc_[funcIdx]] = 0;
	while( funcIEvalCacheIter != endIter ) { 
	    auto& funcIEvalCacheLow = *(  funcIEvalCacheIter);
	    auto& funcIEvalCacheHig = *(++funcIEvalCacheIter);
	    if(funcEvalReq_[funcIdx][asInt(fem::DIFF1)]) {
		if( fabs(funcIEvalCacheLow.arc - funcIEvalCacheHig.arc) <= FLT_MIN ) { 
		    funcIEvalCacheLow.cache[asInt(fem::DIFF1)] = 0; 
		} else { 
		    funcIEvalCacheLow.cache[asInt(fem::DIFF1)] = (funcIEvalCacheHig.val-funcIEvalCacheLow.val)/(funcIEvalCacheHig.arc-funcIEvalCacheLow.arc); 
		}
	    }
	    funcIEvalCacheLow.cache[asInt(fem::DIFF2)] = 0; 
	    if(funcEvalReq_[funcIdx][asInt(fem::INT1)]) {
		funcIEvalCacheHig.cache[asInt(fem::INT1)] = funcIEvalCacheLow.cache[asInt(fem::INT1)] + computeFuncDeltaInt1(funcIdx, funcIEvalCacheHig.arc); 
	    }
	    if(funcEvalReq_[funcIdx][asInt(fem::INT2)]) {
		funcIEvalCacheHig.cache[asInt(fem::INT2)] = funcIEvalCacheLow.cache[asInt(fem::INT2)] + computeFuncDeltaInt2(funcIdx, funcIEvalCacheHig.arc); 
	    }
	    
	    ++funcEvalArcCacheIdxUnder_[func2Arc_[funcIdx]];
	}
	--funcEvalArcCacheIdxUnder_[func2Arc_[funcIdx]];
	auto& funcIEvalCacheHig = *(  funcIEvalCacheIter);
	auto& funcIEvalCacheLow = *(--funcIEvalCacheIter);
	
	if(funcEvalReq_[funcIdx][asInt(fem::DIFF1)]) {
	    funcIEvalCacheHig.cache[asInt(fem::DIFF1)] = funcIEvalCacheLow.cache[asInt(fem::DIFF1)]; 
	}
	
	funcIEvalCacheHig.cache[asInt(fem::DIFF2)] = 0; 
	
	if(funcEvalReq_[funcIdx][asInt(fem::INT1)]) {
	    funcIEvalCacheHig.cache[asInt(fem::INT1)] = funcIEvalCacheLow.cache[asInt(fem::INT1)] + computeFuncDeltaInt1(funcIdx, funcIEvalCacheHig.arc); 
	}
	if(funcEvalReq_[funcIdx][asInt(fem::INT2)]) {
	    funcIEvalCacheHig.cache[asInt(fem::INT2)] = funcIEvalCacheLow.cache[asInt(fem::INT2)] + computeFuncDeltaInt2(funcIdx, funcIEvalCacheHig.arc); 
	}
    }
    setEvalArc(funcsArcBegin_, EvalArcGuarantee::AT_BEGIN);
    precomputeDist();
}

void ParamFuncsSpline0Dist::setEvalArc ( const double& _funcsArcEval, const EvalArcGuarantee& _evalArcGuarantee ) {
    using eag = EvalArcGuarantee;
    
    funcsArcEval_ = fmax( fmin( _funcsArcEval, funcsArcEnd_ ), funcsArcBegin_ );
    const size_t& funcsArcSz = funcsArcSize();
    switch (_evalArcGuarantee) {
	case eag::AFTER_LAST : 
	    for ( size_t i = 0; i < funcsArcSz; ++i ) { 
		const size_t arcISize = funcCtrlPt_[i].size();
		size_t&             j = funcEvalArcCacheIdxUnder_[i];
		while( funcsArc( i, j ) <  funcsArcEval_ ){ if( ++j >= arcISize) { break; } } 
// 		--j;
		j = std::max(0, (int)j - 1);
	    }
	    break;
	case eag::AT_BEGIN   :
	    for ( size_t i = 0; i < funcsArcSz; ++i ) { funcEvalArcCacheIdxUnder_[i] = 0; }
	    break;
	case eag::AT_END     : 
	    for ( size_t i = 0; i < funcsArcSz; ++i ) { funcEvalArcCacheIdxUnder_[i] = funcCtrlPt_[arc2func_[i][0]].size(); }
	    break;
	case eag::NONE       : 
	    for ( size_t i = 0; i < funcsArcSz; ++i ) { 
		auto& aFuncAtArc = funcCtrlPt_[arc2func_[i][0]];
		static double referenceArc; static FuncCtrlPt dummy(0, referenceArc);
		referenceArc = funcsArcEval_;
		funcEvalArcCacheIdxUnder_[i] = std::max( (int)distance( aFuncAtArc.begin(), 
							                upper_bound( aFuncAtArc.begin(), 
								                     aFuncAtArc.end(), 
									     	     dummy,
										     [](const FuncCtrlPt& a, const FuncCtrlPt& b){ return a.arc <= b.arc; } ) 
								       ) - 1, 0);
	    }
	    break;
	case eag::BEFORE_LAST: 
	    for ( size_t i = 0; i < funcsArcSz; ++i ) { 
		size_t& j = funcEvalArcCacheIdxUnder_[i];
		while( funcsArc( i, j ) >= funcsArcEval_ ){ if( j == 0 ) { break; } --j; }      
	    }
	    break;
    }
}

double ParamFuncsSpline0Dist::computeFuncVal   ( const size_t& _funcIdx ) const {
    const FuncCacheData& cacheData = funcEvalCache_[_funcIdx][funcEvalArcCacheIdxUnder_[func2Arc_[_funcIdx]]];
    return cacheData.val + cacheData.cache[ asInt(FuncEvalMode::DIFF1) ] * ( funcsArcEval_ - cacheData.arc );
}

double ParamFuncsSpline0Dist::computeFuncDiff1 ( const size_t& _funcIdx ) const {
    const FuncCacheData& cacheData = funcEvalCache_[_funcIdx][funcEvalArcCacheIdxUnder_[func2Arc_[_funcIdx]]];
    return cacheData.cache[ asInt(FuncEvalMode::DIFF1) ];
}
double ParamFuncsSpline0Dist::computeFuncDiff2 ( const size_t& _funcIdx ) const {
    return 0;
}

double ParamFuncsSpline0Dist::computeFuncInt1 ( const size_t& _funcIdx ) const {
    const FuncCacheData& cacheData = funcEvalCache_[_funcIdx][funcEvalArcCacheIdxUnder_[func2Arc_[_funcIdx]]];
    return cacheData.cache[ asInt(FuncEvalMode::INT1) ] + computeFuncDeltaInt1 ( _funcIdx, funcsArcEval_ );
}
double ParamFuncsSpline0Dist::computeFuncInt2 ( const size_t& _funcIdx ) const {
    const FuncCacheData& cacheData = funcEvalCache_[_funcIdx][funcEvalArcCacheIdxUnder_[func2Arc_[_funcIdx]]];
    return cacheData.cache[ asInt(FuncEvalMode::INT2) ] + computeFuncDeltaInt2 ( _funcIdx, funcsArcEval_ );
}

double ParamFuncsSpline0Dist::computeFuncDeltaInt1 ( const size_t& _funcIdx, const double& _arcEnd ) const {
    const FuncCacheData& ctrlPtLow  = funcEvalCache_[_funcIdx][funcEvalArcCacheIdxUnder_[func2Arc_[_funcIdx]]];
    const double dt = _arcEnd - ctrlPtLow.arc;
    return ctrlPtLow.val * dt +  computeFuncDiff1 ( _funcIdx ) * dt*dt / 2.;
}
double ParamFuncsSpline0Dist::computeFuncDeltaInt2 ( const size_t& _funcIdx, const double& _arcEnd ) const {
    const FuncCacheData& ctrlPtLow  = funcEvalCache_[_funcIdx][funcEvalArcCacheIdxUnder_[func2Arc_[_funcIdx]]];
    const double dt   = _arcEnd - ctrlPtLow.arc;
    const double dtdt = dt*dt;
    return ctrlPtLow.val * dtdt / 2.+ computeFuncDiff1 ( _funcIdx ) * dtdt * dt / 6.;
}



