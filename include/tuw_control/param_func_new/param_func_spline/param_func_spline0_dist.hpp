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

#ifndef PARAM_FUNC_SPLINE0_DIST_HPP
#define PARAM_FUNC_SPLINE0_DIST_HPP

#include <memory>
#include <vector>
#include <algorithm>
#include <float.h>

#include <tuw_control/param_func_new/param_func.hpp>
#include <tuw_control/param_func_new/param_func_dist.hpp>

namespace tuw {

template<typename TNumType>
struct FuncCacheData {
    FuncCacheData ( const FuncCtrlPt<TNumType>& _ctrlPt ) : val(_ctrlPt.val), arc(_ctrlPt.arc) {}
    const TNumType& val;
    const TNumType& arc;
    std::array<TNumType, nrModesMax_> cache;
};
    
template<typename TNumType>
struct ParamFuncsSpline0DistArcVarsDyn {
    ///@brief Maps the arc parametrizations to the functions that use them.
    protected: std::vector< std::vector<std::size_t> > arc2func_;
    ///@brief Contains the index of the \htmlonly<u>next</u>\endhtmlonly  control point relative to the last evaluation point.
    protected: std::vector< std::size_t              > funcEvalArcCacheIdxUnder_;
};
template<typename TNumType,size_t TArcLatticeSize>
struct ParamFuncsSpline0DistArcVarsStatic {
    ///@brief Maps the arc parametrizations to the functions that use them.
    protected: std::array< std::vector<std::size_t>, TArcLatticeSize > arc2func_;
    ///@brief Contains the index of the \htmlonly<u>next</u>\endhtmlonly  control point relative to the last evaluation point.
    protected: std::array< std::size_t            , TArcLatticeSize > funcEvalArcCacheIdxUnder_;
};

template<typename TNumType>
struct ParamFuncsSpline0DistFuncVarsDyn {
    ///@brief Cached values of the used function evaluation modes.
    protected: std::vector< std::vector< FuncCacheData<TNumType> > > funcEvalCache_;
};
template<typename TNumType,size_t TFuncSize>
struct ParamFuncsSpline0DistFuncVarsStatic {
    ///@brief Cached values of the used function evaluation modes.
    protected: std::array< std::vector< FuncCacheData<TNumType> >, TFuncSize  > funcEvalCache_;
};


    
/*!@class ParamFuncsSpline0Dist 
 * 
 * @todo Test distance computation for AV input. 
 * @todo Should s0 = 0 (the way it is now) or add another function that sets it?
 * @todo cout the control points and their arc parametrizations (override with dist also)
 */
template<typename TNumType, int TFuncSize, int TArcLatticeSize>
class ParamFuncsSpline0Dist : public ParamFuncsBase    < ParamFuncsSpline0Dist<TNumType, TFuncSize, TArcLatticeSize>, TNumType, TFuncSize, TArcLatticeSize >, 
                              public ParamFuncsDistBase< ParamFuncsSpline0Dist<TNumType, TFuncSize, TArcLatticeSize>, TNumType >,
			      public std::conditional <TFuncSize      ==-1, ParamFuncsSpline0DistFuncVarsDyn<TNumType>, ParamFuncsSpline0DistFuncVarsStatic<TNumType, TFuncSize      > >::type,
			      public std::conditional <TArcLatticeSize==-1, ParamFuncsSpline0DistArcVarsDyn <TNumType>, ParamFuncsSpline0DistArcVarsStatic <TNumType, TArcLatticeSize> >::type {
    
    using ParamFuncsBaseType     = ParamFuncsBase    < ParamFuncsSpline0Dist<TNumType, TFuncSize, TArcLatticeSize>, TNumType, TFuncSize, TArcLatticeSize >;
    using ParamFuncsDistBaseType = ParamFuncsDistBase< ParamFuncsSpline0Dist<TNumType, TFuncSize, TArcLatticeSize>, TNumType >;
    using FuncCtrlPtType = FuncCtrlPt<TNumType>;
    using FuncCacheDataType = FuncCacheData<TNumType>;
    private  : static constexpr const bool IsFuncDyn = TFuncSize==-1;
    private  : static constexpr const bool IsArcDyn  = TArcLatticeSize==-1;
    
    public   : using NumType = TNumType;
    //special class member functions
    public   : ParamFuncsSpline0Dist           ()                             = default;
    public   : virtual ~ParamFuncsSpline0Dist  ()                             = default;
    public   : ParamFuncsSpline0Dist           (const ParamFuncsSpline0Dist& _other ) : ParamFuncsBaseType(_other), ParamFuncsDistBaseType(_other) {
	initImplImpl();
	this->setDistCfMode(_other.distCfMode_, _other.distRelFuncIdx_);
    }
    public   : ParamFuncsSpline0Dist& operator=(const ParamFuncsSpline0Dist& _other ) {
	if( this != &_other ) {
	    ParamFuncsBaseType    ::operator=(_other);
	    ParamFuncsDistBaseType::operator=(_other);
	    this->setDistCfMode(_other.distCfMode_, _other.distRelFuncIdx_);
	}
	return *this;
    }
    public   : ParamFuncsSpline0Dist           (ParamFuncsSpline0Dist&&)      = delete;
    public   : ParamFuncsSpline0Dist& operator=(ParamFuncsSpline0Dist&&)      = delete;
    
    //--------------------------------------- ParamFunc implementation ---------------------------------------//

    protected: template<bool FuncDyn = (IsFuncDyn), bool ArcDyn = IsArcDyn, typename std::enable_if< (FuncDyn)&&(ArcDyn) >::type* = nullptr> 
	       void      initImplImpl         () {
		    const size_t funcSize = this->funcsSize();
		    this->funcEvalCache_           .resize( funcSize             );  for( auto& funcEvalCacheI : this->funcEvalCache_ ) { funcEvalCacheI.clear(); }
		    this->arc2func_                .resize( this->funcsArcSize() );  for( auto& arc2funcI      : this->arc2func_      ) { arc2funcI     .clear(); }
		    this->funcEvalArcCacheIdxUnder_.resize( this->funcsArcSize() );
		    
		    for( size_t funcIdx = 0; funcIdx < funcSize; ++funcIdx ) {
			const size_t funcISize = this->funcCtrlPt_[funcIdx].size();
			this->funcEvalReq_[funcIdx][asInt(FuncEvalMode::DIFF1)] = true ;//force caching of DIFF1, used by computeFuncVal
			this->funcEvalReq_[funcIdx][asInt(FuncEvalMode::DIFF2)] = false;//second function derivative is allways 0
			for( size_t ctrlPtIdx = 0; ctrlPtIdx < funcISize; ++ctrlPtIdx ) {
			    this->funcEvalCache_[funcIdx].emplace_back( FuncCacheDataType( this->funcCtrlPt_[funcIdx][ctrlPtIdx] ) ); 
			}
			this->arc2func_[this->func2Arc_[funcIdx]].emplace_back(funcIdx);
		    }
		}
    protected: template<bool FuncDyn = (IsFuncDyn), bool ArcDyn = IsArcDyn, typename std::enable_if< (!FuncDyn)&&(ArcDyn) >::type* = nullptr> 
	       void      initImplImpl         () {
		                                                                     for( auto& funcEvalCacheI : this->funcEvalCache_ ) { funcEvalCacheI.clear(); }
		    this->arc2func_                .resize( this->funcsArcSize() );  for( auto& arc2funcI      : this->arc2func_      ) { arc2funcI     .clear(); }
		    this->funcEvalArcCacheIdxUnder_.resize( this->funcsArcSize() );
		    
		    for( size_t funcIdx = 0; funcIdx < TFuncSize; ++funcIdx ) {
			const size_t funcISize = this->funcCtrlPt_[funcIdx].size();
			this->funcEvalReq_[funcIdx][asInt(FuncEvalMode::DIFF1)] = true ;//force caching of DIFF1, used by computeFuncVal
			this->funcEvalReq_[funcIdx][asInt(FuncEvalMode::DIFF2)] = false;//second function derivative is allways 0
			for( size_t ctrlPtIdx = 0; ctrlPtIdx < funcISize; ++ctrlPtIdx ) {
			    this->funcEvalCache_[funcIdx].emplace_back( FuncCacheDataType( this->funcCtrlPt_[funcIdx][ctrlPtIdx] ) ); 
			}
			this->arc2func_[this->func2Arc_[funcIdx]].emplace_back(funcIdx);
		    }
		}
    protected: template<bool FuncDyn = (IsFuncDyn), bool ArcDyn = IsArcDyn, typename std::enable_if< (FuncDyn)&&(!ArcDyn) >::type* = nullptr> 
	       void      initImplImpl         () {
		    const size_t funcSize = this->funcsSize();
		    this->funcEvalCache_           .resize( funcSize             );  for( auto& funcEvalCacheI : this->funcEvalCache_ ) { funcEvalCacheI.clear(); }
		                                                                     for( auto& arc2funcI      : this->arc2func_      ) { arc2funcI     .clear(); }
		    for( size_t funcIdx = 0; funcIdx < funcSize; ++funcIdx ) {
			const size_t funcISize = this->funcCtrlPt_[funcIdx].size();
			this->funcEvalReq_[funcIdx][asInt(FuncEvalMode::DIFF1)] = true ;//force caching of DIFF1, used by computeFuncVal
			this->funcEvalReq_[funcIdx][asInt(FuncEvalMode::DIFF2)] = false;//second function derivative is allways 0
			for( size_t ctrlPtIdx = 0; ctrlPtIdx < funcISize; ++ctrlPtIdx ) {
			    this->funcEvalCache_[funcIdx].emplace_back( FuncCacheDataType( this->funcCtrlPt_[funcIdx][ctrlPtIdx] ) ); 
			}
			this->arc2func_[this->func2Arc_[funcIdx]].emplace_back(funcIdx);
		    }
		}
    protected: template<bool FuncDyn = (IsFuncDyn), bool ArcDyn = IsArcDyn, typename std::enable_if< (!FuncDyn)&&(!ArcDyn) >::type* = nullptr> 
	       void      initImplImpl         () {
		    for( auto& funcEvalCacheI : this->funcEvalCache_ ) { funcEvalCacheI.clear(); }
		    for( auto& arc2funcI      : this->arc2func_      ) { arc2funcI     .clear(); }
		    for( size_t funcIdx = 0; funcIdx < TFuncSize; ++funcIdx ) {
			const size_t funcISize = this->funcCtrlPt_[funcIdx].size();
			this->funcEvalReq_[funcIdx][asInt(FuncEvalMode::DIFF1)] = true ;//force caching of DIFF1, used by computeFuncVal
			this->funcEvalReq_[funcIdx][asInt(FuncEvalMode::DIFF2)] = false;//second function derivative is allways 0
			for( size_t ctrlPtIdx = 0; ctrlPtIdx < funcISize; ++ctrlPtIdx ) {
			    this->funcEvalCache_[funcIdx].emplace_back( FuncCacheDataType( this->funcCtrlPt_[funcIdx][ctrlPtIdx] ) ); 
			}
			this->arc2func_[this->func2Arc_[funcIdx]].emplace_back(funcIdx);
		    }
		}
    ///@brief Precomputes cached data. To be called after ANY control point modifications.
    public   : void      precomputeImpl       () {
	using fem = FuncEvalMode;
	
	for( size_t arcIdx = 0; arcIdx < this->funcsArcSize(); ++arcIdx ) { 
	    for( size_t arcKnot = 1; arcKnot < this->funcsArcSize(arcIdx); ++arcKnot ) { 
		this->funcsArc(arcIdx, arcKnot) = fmax( this->funcsArc(arcIdx, arcKnot), this->funcsArc(arcIdx, arcKnot-1) );
	    }
	}
	
	const double funcSize = this->funcCtrlPt_.size();
	for( size_t funcIdx = 0; funcIdx < funcSize; ++funcIdx ) {
	    
	    auto& funcIEvalCache        = this->funcEvalCache_[funcIdx];
	    auto  funcIEvalCacheIter    = funcIEvalCache.begin();
	    funcIEvalCacheIter->cache[asInt(fem::INT1)] = 0;
	    funcIEvalCacheIter->cache[asInt(fem::INT2)] = 0;
	    const auto& endIter = funcIEvalCache.end()-1;
	    
	    this->funcEvalArcCacheIdxUnder_[this->func2Arc_[funcIdx]] = 0;
	    while( funcIEvalCacheIter != endIter ) { 
		auto& funcIEvalCacheLow = *(  funcIEvalCacheIter);
		auto& funcIEvalCacheHig = *(++funcIEvalCacheIter);
		if(this->funcEvalReq_[funcIdx][asInt(fem::DIFF1)]) {
		    if( fabs(funcIEvalCacheLow.arc - funcIEvalCacheHig.arc) <= FLT_MIN ) { 
			funcIEvalCacheLow.cache[asInt(fem::DIFF1)] = 0; 
		    } else { 
			funcIEvalCacheLow.cache[asInt(fem::DIFF1)] = (funcIEvalCacheHig.val-funcIEvalCacheLow.val)/(funcIEvalCacheHig.arc-funcIEvalCacheLow.arc); 
		    }
		}
		funcIEvalCacheLow.cache[asInt(fem::DIFF2)] = 0; 
		if(this->funcEvalReq_[funcIdx][asInt(fem::INT1)]) {
		    funcIEvalCacheHig.cache[asInt(fem::INT1)] = funcIEvalCacheLow.cache[asInt(fem::INT1)] + computeFuncDeltaInt1(funcIdx, funcIEvalCacheHig.arc); 
		}
		if(this->funcEvalReq_[funcIdx][asInt(fem::INT2)]) {
		    funcIEvalCacheHig.cache[asInt(fem::INT2)] = funcIEvalCacheLow.cache[asInt(fem::INT2)] + computeFuncDeltaInt2(funcIdx, funcIEvalCacheHig.arc); 
		}
		
		++this->funcEvalArcCacheIdxUnder_[this->func2Arc_[funcIdx]];
	    }
	    --this->funcEvalArcCacheIdxUnder_[this->func2Arc_[funcIdx]];
	    auto& funcIEvalCacheHig = *(  funcIEvalCacheIter);
	    auto& funcIEvalCacheLow = *(--funcIEvalCacheIter);
	    
	    if(this->funcEvalReq_[funcIdx][asInt(fem::DIFF1)]) {
		funcIEvalCacheHig.cache[asInt(fem::DIFF1)] = funcIEvalCacheLow.cache[asInt(fem::DIFF1)]; 
	    }
	    
	    funcIEvalCacheHig.cache[asInt(fem::DIFF2)] = 0; 
	    
	    if(this->funcEvalReq_[funcIdx][asInt(fem::INT1)]) {
		funcIEvalCacheHig.cache[asInt(fem::INT1)] = funcIEvalCacheLow.cache[asInt(fem::INT1)] + computeFuncDeltaInt1(funcIdx, funcIEvalCacheHig.arc); 
	    }
	    if(this->funcEvalReq_[funcIdx][asInt(fem::INT2)]) {
		funcIEvalCacheHig.cache[asInt(fem::INT2)] = funcIEvalCacheLow.cache[asInt(fem::INT2)] + computeFuncDeltaInt2(funcIdx, funcIEvalCacheHig.arc); 
	    }
	}
	this->funcsArcEval_ = -1;
	this->setEvalArc(this->funcsArcBegin_, EvalArcGuarantee::AT_BEGIN);
	precomputeDist();
    }
    public   : void      setEvalArcImpl       ( const TNumType& _arcEval , const EvalArcGuarantee& _eAG ) {
	using eag = EvalArcGuarantee;
	const double arcEvalNewBound = fmax( fmin( _arcEval, this->funcsArcEnd_ ), this->funcsArcBegin_ );
	if(arcEvalNewBound == this->funcsArcEval_) { return; }
	const size_t& funcsArcSz = this->funcsArcSize();
	switch (_eAG) {
	    case eag::NEAR_LAST : 
		if(_arcEval > this->funcsArcEval_) {
		    for ( size_t i = 0; i < funcsArcSz; ++i ) { 
			const size_t arcISize = this->funcCtrlPt_[i].size();
			size_t&             j = this->funcEvalArcCacheIdxUnder_[i];
			while( this->funcsArc( i, j ) <  arcEvalNewBound ){ if( ++j >= arcISize) { break; } } //--j;
			j = std::max(0, (int)j - 1);
		    }
		} else {
		    for ( size_t i = 0; i < funcsArcSz; ++i ) { 
			size_t& j = this->funcEvalArcCacheIdxUnder_[i];
			while( this->funcsArc( i, j ) >= arcEvalNewBound ){ if( j == 0 ) { break; } --j; }      
		    }
		}
		break;
	    case eag::AT_BEGIN   :
		for ( size_t i = 0; i < funcsArcSz; ++i ) { this->funcEvalArcCacheIdxUnder_[i] = 0; }
		break;
	    case eag::AT_END     : 
		for ( size_t i = 0; i < funcsArcSz; ++i ) { this->funcEvalArcCacheIdxUnder_[i] = this->funcCtrlPt_[this->arc2func_[i][0]].size()-1; }
		break;
	    case eag::NONE       : 
		for ( size_t i = 0; i < funcsArcSz; ++i ) { 
		    auto& aFuncAtArc = this->funcCtrlPt_[this->arc2func_[i][0]];
		    static double referenceArc; static FuncCtrlPtType dummy(0, referenceArc);
		    referenceArc = arcEvalNewBound;
		    this->funcEvalArcCacheIdxUnder_[i] = std::max( (int)std::distance( aFuncAtArc.begin(), 
									         std::upper_bound( aFuncAtArc.begin(), 
											           aFuncAtArc.end(), 
											           dummy,
											           [](const FuncCtrlPtType& a, const FuncCtrlPtType& b){ return a.arc <= b.arc; } ) 
									        ) - 1, 0);
		}
		break;
	}
	this->funcsArcEval_ = arcEvalNewBound;
    }
    public   : TNumType  computeFuncValImpl   ( const std::size_t& _funcIdx ) const {
	const FuncCacheDataType& cacheData = this->funcEvalCache_[_funcIdx][this->funcEvalArcCacheIdxUnder_[this->func2Arc_[_funcIdx]]];
	return cacheData.val + cacheData.cache[ asInt(FuncEvalMode::DIFF1) ] * ( this->funcsArcEval_ - cacheData.arc );
    }
    public   : TNumType  computeFuncDiff1Impl ( const std::size_t& _funcIdx ) const {
	const FuncCacheDataType& cacheData = this->funcEvalCache_[_funcIdx][this->funcEvalArcCacheIdxUnder_[this->func2Arc_[_funcIdx]]];
	return cacheData.cache[ asInt(FuncEvalMode::DIFF1) ];
    }
    public   : TNumType  computeFuncDiff2Impl ( const std::size_t& _funcIdx ) const {
	return 0;
    }
    public   : TNumType  computeFuncInt1Impl  ( const std::size_t& _funcIdx ) const {
	const FuncCacheDataType& cacheData = this->funcEvalCache_[_funcIdx][this->funcEvalArcCacheIdxUnder_[this->func2Arc_[_funcIdx]]];
	return cacheData.cache[ asInt(FuncEvalMode::INT1) ] + computeFuncDeltaInt1 ( _funcIdx, this->funcsArcEval_ );
    }
    public   : TNumType  computeFuncInt2Impl  ( const std::size_t& _funcIdx ) const {
	const FuncCacheDataType& cacheData = this->funcEvalCache_[_funcIdx][this->funcEvalArcCacheIdxUnder_[this->func2Arc_[_funcIdx]]];
	return cacheData.cache[ asInt(FuncEvalMode::INT2) ] + computeFuncDeltaInt2 ( _funcIdx, this->funcsArcEval_ );
    }
    
    ///@brief Computes the integral of the parametric function @ref _\funcIdx on interval [@ref funcEvalArcCacheIdxOld\_[func2Arc_[_funcIdx]] - 1, @ref \_arcEnd]
    private  : TNumType computeFuncDeltaInt1 ( const std::size_t& _funcIdx, const TNumType& _arcEnd ) const {
	const FuncCacheDataType& ctrlPtLow  = this->funcEvalCache_[_funcIdx][this->funcEvalArcCacheIdxUnder_[this->func2Arc_[_funcIdx]]];
	const double dt = _arcEnd - ctrlPtLow.arc;
	return ctrlPtLow.val * dt +  computeFuncDiff1Impl ( _funcIdx ) * dt*dt / 2.;
    }
    ///@brief Computes the double integral integral of the parametric function @ref _\funcIdx on interval [@ref funcEvalArcCacheIdxOld\_[func2Arc_[_funcIdx]] - 1, @ref \_arcEnd]
    private  : TNumType computeFuncDeltaInt2 ( const std::size_t& _funcIdx, const TNumType& _arcEnd ) const {
	const FuncCacheDataType& ctrlPtLow  = this->funcEvalCache_[_funcIdx][this->funcEvalArcCacheIdxUnder_[this->func2Arc_[_funcIdx]]];
	const double dt   = _arcEnd - ctrlPtLow.arc;
	const double dtdt = dt*dt;
	return ctrlPtLow.val * dtdt / 2.+ computeFuncDiff1Impl ( _funcIdx ) * dtdt * dt / 6.;
    }
    
//     ///@brief Contains the index of the \htmlonly<u>next</u>\endhtmlonly  control point relative to the last evaluation point.
//     protected: std::vector< std::size_t                  > funcEvalArcCacheIdxUnder_;
//     ///@brief Cached values of the used function evaluation modes.
//     protected: std::vector< std::vector< FuncCacheDataType > > funcEvalCache_;
//     ///@brief Maps the arc parametrizations to the functions that use them.
//     private  : std::vector< std::vector<std::size_t> > arc2func_;
    
    
    //--------------------------------------- ParamFuncDist implementation ---------------------------------------//
    
    using TDM = TraveledDistCfMode;
    
    //(re-)implemented virtual functions
    public   : void     setDistCfModeImpl ( TraveledDistCfMode _distCfMode, const std::vector<std::size_t>& _distRelFuncIdx ) {
	distCfMode_ = _distCfMode;
	distRelFuncIdx_ = _distRelFuncIdx;
	switch (distCfMode_) {
	    case TDM::NONE: break;
	    case TDM::V   : 
		distLinkedFuncIdx_.resize(1, _distRelFuncIdx[0]); 
		distEvalCache_.resize(this->funcCtrlPt_[_distRelFuncIdx[0]].size(), 0); 
		computeSFuncPtr_ = &ParamFuncsSpline0Dist::computeS_V;  
		computeDs2DtPtr_ = &ParamFuncsSpline0Dist::computeT_V;
		distLinkedArcIdx_= this->func2Arc_[_distRelFuncIdx[0]];
		break;
	    case TDM::AV  : 
		distLinkedFuncIdx_.resize(1, _distRelFuncIdx[0]); 
		distEvalCache_.resize(this->funcCtrlPt_[_distRelFuncIdx[0]].size(), 0); 
		computeSFuncPtr_ = &ParamFuncsSpline0Dist::computeS_AV; 
		computeDs2DtPtr_ = &ParamFuncsSpline0Dist::computeT_AV; 
		distLinkedArcIdx_= this->func2Arc_[_distRelFuncIdx[0]];
		break;
	}
    }
    public   : TNumType computeSImpl      () const { return (this->*computeSFuncPtr_)(); }
    public   : TNumType computeTImpl      ( const TNumType& _s, const EvalArcGuarantee& _evalArcGuarantee  ) {
	size_t& arcIdx  = this->funcEvalArcCacheIdxUnder_[distLinkedArcIdx_];
	
	const size_t cacheCtrlPtIdxRestore = arcIdx;
	const size_t cacheArcEvalRestore   = this->funcsArcEval_;
	
	double resultT = computeTImplInternal( _s, _evalArcGuarantee );
	
	arcIdx              = cacheCtrlPtIdxRestore;
	this->funcsArcEval_ = cacheArcEvalRestore;
	
	return resultT;
    }
    public   : void     setEvalDistImpl   ( const TNumType& _funcsDistEval, const EvalArcGuarantee& _evalArcGuarantee ) {    
	this->setEvalArc( computeTImplInternal( _funcsDistEval, _evalArcGuarantee ), _evalArcGuarantee );
    }
    public   : void   computeS2TLatticeImpl ( const std::vector<TNumType>& _sLattice, std::vector<TNumType>& _tLattice ) {
	const size_t slSize = _sLattice.size();
	_tLattice.clear(); _tLattice.reserve( _sLattice.size() );
	
	if( slSize > 0 ){
	    size_t i = 0;
	    this->setEvalDist ( _sLattice[i], EvalArcGuarantee::NONE );
	    while( ( this->funcsArcEval_ < this->funcsArcEnd_ ) && ( i+1 < slSize ) ){ 
		_tLattice.emplace_back(this->funcsArcEval_);
		this->setEvalDist ( _sLattice[++i], EvalArcGuarantee::NEAR_LAST ); 
	    }
	}
	_tLattice.emplace_back(this->funcsArcEnd_);
	this->setEvalArc ( this->funcsArcBegin_, EvalArcGuarantee::AT_BEGIN );
    }
    public   : void   computeS2TLatticeImpl ( const TNumType& _arc0, const TNumType& _ds, std::vector<TNumType>& _tLattice ) {
	this->setEvalArc ( _arc0, EvalArcGuarantee::NONE );
	_tLattice.clear(); _tLattice.reserve( distEvalCache_.back() / _ds );
	const size_t idxBeforeStart = static_cast<int>( computeSImpl() / _ds ) - 1;
	
	size_t i = 0;
	this->setEvalDist ( _ds * ( ++i + idxBeforeStart ), EvalArcGuarantee::NEAR_LAST );
	while( this->funcsArcEval_ < this->funcsArcEnd_ ){ 
	    _tLattice.emplace_back(this->funcsArcEval_);
	    this->setEvalDist ( _ds * ( ++i + idxBeforeStart ), EvalArcGuarantee::NEAR_LAST );
	}
	_tLattice.emplace_back(this->funcsArcEnd_);
	this->setEvalArc(this->funcsArcBegin_, EvalArcGuarantee::AT_BEGIN);
    }
    
    ///@brief Helper function that computes deltaS (from the @ref evalArc\_ position) operating on one function control point interval (used by @ref TraveledDistCfMode::V and @ref TraveledDistCfMode::AV modes).
    public   : static TNumType computeDeltaS_V_AV ( const TNumType& _dt, const TNumType& _v0, const TNumType& _av ) {
	const TNumType dtAbs = fabs(_dt); if(      dtAbs < FLT_MIN ) { return 0; }
	const TNumType vAbs  = fabs(_v0); if(  fabs(_av) < FLT_MIN ) { return vAbs * _dt; }
	const TNumType v1 = _v0 + _av * dtAbs;
	if ( signum(v1) == signum(_v0) ) { 
	    return fabs(_v0 + v1) * _dt / 2.;
	} else { 
	    int retSign = 1; if( _dt < 0 ) { retSign = -1; }
	    TNumType dt1 = - _v0 / _av; 
	    return retSign * ( vAbs * dt1 + fabs( v1 ) * (dtAbs - dt1) ) / 2.;  
	}
    }
    ///@brief Helper function that computes deltaT (from the @ref evalArc\_ position) operating on one function control point interval (used by @ref TraveledDistCfMode::V and @ref TraveledDistCfMode::AV modes).
    public   : static TNumType computeDeltaT_V_AV ( const TNumType& _ds, const TNumType& _v0, const TNumType& _av ) {
	TNumType v0 = _v0, av = _av;
	if( v0 < 0 ) { av *= -1; v0 *= -1; }
	if( fabs(av) < FLT_MIN ){ return _ds / v0; }
	
	int retSign = -1; if( _ds < 0 ) { retSign = +1; }
	const TNumType v0v0  = v0 * v0;
	
	TNumType delta = v0v0 + 2. * av * fabs( _ds );
	if ( delta < 0 ){  return retSign * ( + v0 + sqrt(-delta) ) / av; } 
	else            {  return retSign * ( + v0 - sqrt( delta) ) / av; }
    }
    
    ///@brief Precomputes distance invervals.
    private  : void     precomputeDist () {
	if( distCfMode_ == TDM::NONE ) { return; }
	
	const size_t& func0Idx = distLinkedFuncIdx_[0];
	const size_t  ctrlPtSize = this->funcCtrlPt_[func0Idx].size()-1;
	distEvalCache_[0] = 0;
	for( size_t funcCtrlPtIdx = 0; funcCtrlPtIdx < ctrlPtSize; ++funcCtrlPtIdx ) { 
	    this->funcsArcEval_ = this->ctrlPtVal( func0Idx, funcCtrlPtIdx+1, CtrlPtDim::ARC );
	    this->funcEvalArcCacheIdxUnder_[distLinkedArcIdx_] = funcCtrlPtIdx;
	    distEvalCache_[funcCtrlPtIdx+1] = computeSImpl();
	}
	this->setEvalArc(this->funcsArcBegin_, EvalArcGuarantee::AT_BEGIN);
    }
    ///@brief Internal implementation of computing the arc parametrization given a distance @ref _s.
    private  : TNumType   computeTImplInternal   ( const TNumType& _s, const EvalArcGuarantee& _evalArcGuarantee ) {
	using eag = EvalArcGuarantee;
	size_t& arcCacheIdx  = this->funcEvalArcCacheIdxUnder_[distLinkedArcIdx_];
	const size_t ctrlPtSize = this->funcCtrlPt_[distLinkedFuncIdx_[0]].size();
	
	switch (_evalArcGuarantee) {
	    case eag::NEAR_LAST  : {
		const double& distCache = distEvalCache_[arcCacheIdx];
		if(distCache <= _s) { while( distEvalCache_[arcCacheIdx] <= _s ){ if( ++arcCacheIdx >= ctrlPtSize) { break; }                } --arcCacheIdx; }
		else                { while( distEvalCache_[arcCacheIdx] >= _s ){ if( arcCacheIdx   == 0         ) { break; } --arcCacheIdx; }                }
		break;
	    }
	    case eag::AT_BEGIN   : arcCacheIdx = 0; break;
	    case eag::AT_END     : arcCacheIdx = this->funcCtrlPt_[distLinkedFuncIdx_[0]].size()-1; break;
	    case eag::NONE       : arcCacheIdx = std::distance( distEvalCache_.begin(), std::upper_bound( distEvalCache_.begin(), distEvalCache_.end(), _s ) ) - 1; break;
	}
	this->funcsArcEval_ = this->ctrlPtVal(distLinkedFuncIdx_[0], arcCacheIdx, CtrlPtDim::ARC );
	
	return this->funcsArcEval_ + (this->*computeDs2DtPtr_)( _s - distEvalCache_[arcCacheIdx] );
    }
    ///@brief Computes distance on a piecewise linear function describing the center linear velocity.
    private  : TNumType   computeS_V     () const {
	const size_t& funcIdx          = distLinkedFuncIdx_[0];
	const FuncCacheDataType& cacheData = this->funcEvalCache_[funcIdx][this->funcEvalArcCacheIdxUnder_[this->func2Arc_[funcIdx]]];//*funcEvalCacheIter_[funcIdx];
	return distEvalCache_[this->funcEvalArcCacheIdxUnder_[distLinkedArcIdx_]] + computeDeltaS_V_AV( this->funcsArcEval_ - cacheData.arc, cacheData.val, this->computeFuncDiff1(funcIdx) ); 
    }
    ///@brief Computes distance on a piecewise linear function describing the center linear acceleration.
    private  : TNumType   computeS_AV    () const {
	const size_t& funcIdx          = distLinkedFuncIdx_[0];
	const FuncCacheDataType& cacheData = this->funcEvalCache_[funcIdx][this->funcEvalArcCacheIdxUnder_[this->func2Arc_[funcIdx]]];//*funcEvalCacheIter_[funcIdx];
	return distEvalCache_[this->funcEvalArcCacheIdxUnder_[distLinkedArcIdx_]] + computeDeltaS_V_AV( this->funcsArcEval_ - cacheData.arc, this->computeFuncInt1(funcIdx), cacheData.val  );
    }
    ///@brief Computes arc parametrization on a piecewise linear function describing the center linear velocity at a variation @ref _ds.
    private  : TNumType   computeT_V     ( const TNumType& _ds ) const {
	const size_t& funcIdx  = distLinkedFuncIdx_[0];
	return computeDeltaT_V_AV( _ds, this->computeFuncVal( funcIdx ), this->computeFuncDiff1( funcIdx ) );
    }
    ///@brief Computes arc parametrization on a piecewise linear function describing the center linear acceleration at a variation @ref _ds.
    private  : TNumType   computeT_AV    ( const TNumType& _ds ) const {
	const size_t& funcIdx  = distLinkedFuncIdx_[0];
	return computeDeltaT_V_AV( _ds, this->computeFuncInt1 (funcIdx), this->computeFuncVal  ( funcIdx ) );
    }
    
    ///@brief Indexes of parametric functions used for computing the traveled distance.
    private  : std::vector<std::size_t> distLinkedFuncIdx_;
    ///@brief Cached values of traveled distance computation.
    private  : std::vector<TNumType     > distEvalCache_;
    ///@brief Closed form distance computation mode.
    private  : TraveledDistCfMode distCfMode_;
    ///@todo
    private  : std::vector< std::size_t > distRelFuncIdx_;
    ///@brief Index of the parametrized function that relates to distance computation.
    private  : std::size_t distLinkedArcIdx_;
    
    private  : using ComputeSFuncPtr = TNumType (ParamFuncsSpline0Dist::*)( ) const;
    private  : using ComputeDs2DtPtr = TNumType (ParamFuncsSpline0Dist::*)( const TNumType& ) const;
    private  : ComputeSFuncPtr computeSFuncPtr_;
    private  : ComputeDs2DtPtr computeDs2DtPtr_;
    
    
    template<typename TDerived2, typename TNumType2, int TFuncSize2, int TArcLatticeSize2> friend class ParamFuncsBase;
};

}

#endif // PARAM_FUNC_SPLINE0_DIST_HPP
