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

#ifndef COST_FUNCTIONS_H
#define COST_FUNCTIONS_H


#include <vector>
#include <math.h>
#include <float.h>
#include <functional>

#include <tuw_control/utils.h>

#include <iostream>

namespace tuw {

namespace cost_functions {

///@todo Proper special class member functions
    
class CostFunc {
    public   : CostFunc() : latBeginIdx_(0) {}
    
    public   : void setBoundIdxBegin( const std::size_t& _idxBoundBegin ) { latBeginIdx_ = _idxBoundBegin; }
    public   : void setBoundArcEnd  ( const double&      _arcBoundEnd   ) { latBackArc_  = _arcBoundEnd;   }
    public   : void setCost0        ( const double&      _cost0         ) { cost0_ = _cost0; }
    public   : void resetFunction   ()                                    { cost_ = cost0_; iterIdx_ = latBeginIdx_; finish_ = false; }
    public   : void calcCostsFull   () { 
	while ( !finish_ ) {  
	    if ( finish_ || !hasReqData(iterIdx_) ) { break; }
	    calcCosts1Step(); 
	} 
    }
    public   : void calcCostsFull   (const double& _arcLimit) {
	while ( !finish_ ) {  
// 	    std::cout<<std::endl<<"entered1StepLow, ";
	    if ( finish_  || !hasReqData(iterIdx_) ) { /*std::cout<<"finished1, cost="<<cost_<<std::endl;*/ break; }
	    if ( ( arcAcc(iterIdx_) > _arcLimit )  ) { /*std::cout<<"finished2, cost="<<cost_<<std::endl;*/ break; }
	    	
	    if      ( arcAcc(iterIdx_) <  latBackArc_  ) { cost_ = fL2(cost_, iterIdx_); /*std::cout<<"partialFL2, cost="<<cost_<<std::endl;*/ }
	    else if ( arcAcc(iterIdx_) == latBackArc_  ) { cost_ = fL2(cost_, iterIdx_); finish_ = true; /*std::cout<<"finishedFL23, cost="<<cost_<<std::endl;*/ }
// 	else                                         { cost_ = fL3(cost_); finish_ = true;                               /*std::cout<<"finishedFL 3, cost="<<cost_<<std::endl;*/ }
	    ++iterIdx_;
	}
    }
    public   : void calcCosts1Step  () {
	if      ( arcAcc(iterIdx_) <  latBackArc_  ) { cost_ = fL2(cost_, iterIdx_); /*std::cout<<"partialFL2, cost="<<cost_<<std::endl;*/ }
	else if ( arcAcc(iterIdx_) == latBackArc_  ) { cost_ = fL2(cost_, iterIdx_); cost_ = fL3(cost_); finish_ = true; /*std::cout<<"finishedFL23, cost="<<cost_<<std::endl;*/ }
// 	else                                         { cost_ = fL3(cost_); finish_ = true;                               /*std::cout<<"finishedFL 3, cost="<<cost_<<std::endl;*/ }
	++iterIdx_;
// 	std::cout<<"increment, ";
    }
    public   : const double& cost() const { return cost_; }
    
    private  : virtual bool hasReqData(size_t _iterIdx) { return true; }
    
    
    public   : bool   finish_;
    public   : size_t iterIdx_;
    
    protected: std::function<double(                const size_t& )> arcAcc;
    protected: std::function<double(                const size_t& )> stAcc;
    protected: std::function<double(                const size_t& )> fL1;
    protected: std::function<double( const double&, const size_t& )> fL2;
    protected: std::function<double( const double&                )> fL3;
    public   : size_t latBeginIdx_;
    protected: double latBackArc_;
    protected: double cost0_;
    private  : double cost_;
    
    friend class funcPredef_FL1;
    friend class funcPredef_FL2;
    friend class funcPredef_FL3;
    template<typename Lattice, typename MapData>
    friend class CostsArrayLatBase;
};



template<typename Lattice, typename MapData>
class LatticeMapWeight {
    public   : void initLatticeMap ( std::shared_ptr<Lattice>& _lattPtr, std::shared_ptr<MapData>& _mapDataPtr ) {  latticePtr_ = _lattPtr; mapDataPtr_ = _mapDataPtr; }
    public   : void setWeight      ( const double& _weight ) { weight_ = _weight; } 
    public   : std::shared_ptr<Lattice> latticePtr_;
    protected: std::shared_ptr<MapData> mapDataPtr_;
    public   : double   weight_;
};

template<typename Lattice, typename MapData>
class CFLatMap1WeightBase : public CostFunc, public LatticeMapWeight<Lattice, MapData> {
    private  : bool hasReqData (size_t _iterIdx) override { return (this->latticePtr_->size() >_iterIdx) ; } 
};

template<typename Lattice, typename MapData>
using CostFuncLatMap1WeightPtr = std::unique_ptr< CFLatMap1WeightBase<Lattice, MapData> >;

template<typename Lattice, typename MapData>
class CostsArrayLatBase {
    public   : CostsArrayLatBase() : weight_(1) {}
    public   : virtual size_t latFuncLayerIdx() = 0;
    public   : virtual size_t latKnotLayerIdx() = 0;
    private  : virtual CostFuncLatMap1WeightPtr<Lattice, MapData> allocateCostFunc() = 0;

    public   : const double& cost      ( const size_t& _i      ) const { return pieceWiseCosts[_i]->cost(); }
    public   : const size_t  costsSize ()                        const { return pieceWiseCosts.size(); }
    public   : void setWeight          ( const double& _weight )       { weight_ = _weight; for (auto& costI : pieceWiseCosts) { costI->setWeight(_weight); }  }
    public   : void calcCostsFull      ()                              { while ( !finish_ ) {  calcCosts1KnotStep(); } }
    public   : void calcCosts1KnotStep () { 
	while(!finish_){
	    if ( iterIdx_ >= pieceWiseCosts.size() ) { finish_ = true; return; } 
	    pieceWiseCosts[iterIdx_]->calcCostsFull(); 
	    if ( ++iterIdx_ >= pieceWiseCosts.size() ) { finish_ = true; return; } 
	    pieceWiseCosts[iterIdx_]->iterIdx_ = pieceWiseCosts[iterIdx_-1]->iterIdx_;
	}
    }
    public   : void calcCosts1KnotStep (const double& _arcLimit) { 
	iterIdx_ = iterIdxPartBegin_; finish_ = false;
// 	while(!finish_){
	    if ( iterIdx_ >= pieceWiseCosts.size() ) { iterIdxPartBegin_ = iterIdx_; finish_ = true; return; } 
	    pieceWiseCosts[iterIdx_]->calcCostsFull(_arcLimit); 
	    if ( pieceWiseCosts[iterIdx_]->finish_ ) {  iterIdxPartBegin_++; }
	    if ( ++iterIdx_ >= pieceWiseCosts.size() ) { finish_ = true; return; } 
	    pieceWiseCosts[iterIdx_]->iterIdx_ = pieceWiseCosts[iterIdx_-1]->iterIdx_;
// 	}
    }
    
    public   : void reset() { 
	finish_ = false; iterIdx_ = 0; iterIdxPartBegin_ = 0;
	resizeInitNew();
	resetBounds( 0 );
	for (auto& costI : pieceWiseCosts) { costI->resetFunction(); } 
    }
    public   : void resetNew() { 
	finish_ = false; 
	int firstResetBound = resizeInitNew(); 
	resetBounds( firstResetBound );
// 	for ( size_t i = firstResetBound; i < pieceWiseCosts.size(); ++i ) { pieceWiseCosts[i]->setBoundIdxBegin( pieceWiseCosts[i]->latBeginIdx_ ); }
    }
//     public   : double arcAtLattIdxPrev () const { return lattKnotPtr_->at(std::max(0,(int)pieceWiseCosts[std::max(0,(int)iterIdx_-1)]->iterIdx_-2) ).arc; }
    public   : double arcAtLattIdxPrev () const { return lattKnotPtr_->at(std::max(0,(int)iterIdx_-1) ).arc; }
    
    private  : void resetBounds(int _fistResetBound) {
	_fistResetBound = std::max(_fistResetBound, 0)+1;
	for (size_t i = _fistResetBound; i <= this->pieceWiseCosts.size(); ++i) { 
	    this->pieceWiseCosts[i-1]->setBoundIdxBegin( _fistResetBound );
	    this->pieceWiseCosts[i-1]->setBoundArcEnd  ( this->lattKnotPtr_->at(_fistResetBound++).arc );
	} 
    }
    private  : int resizeInitNew() { 
	int sizeOld = pieceWiseCosts.size();
	if( !lattKnotPtr_ || (lattKnotPtr_->size() < 2) ) { pieceWiseCosts.clear(); return 0; }
	pieceWiseCosts.resize( std::max(0, (int)lattKnotPtr_->size() - 1 ) );
	int deltaSize = (int)pieceWiseCosts.size() - sizeOld;
	if ( deltaSize > 0 ) { 
	    for(size_t i = sizeOld; i < this->pieceWiseCosts.size(); ++i) { 
		pieceWiseCosts[i] = allocateCostFunc(); 
		pieceWiseCosts[i]->initLatticeMap(lattFuncPtr_, mapDataPtr_); 
		pieceWiseCosts[i]->resetFunction();
		pieceWiseCosts[i]->setWeight(weight_);
	    } 
	}
	return sizeOld - 1;
    }
    public   : void initLatticeMap( std::vector<std::shared_ptr<Lattice>>& _lattPtr, std::shared_ptr<MapData>& _mapDataPtr) {
	size_t latIdx;
	latIdx = latFuncLayerIdx(); lattFuncPtr_ = nullptr; if(_lattPtr.size() > latIdx) { lattFuncPtr_ = _lattPtr[latIdx]; } 
	latIdx = latKnotLayerIdx(); lattKnotPtr_ = nullptr; if(_lattPtr.size() > latIdx) { lattKnotPtr_ = _lattPtr[latIdx]; knotLatIdx_ = latIdx; } 
	mapDataPtr_ = _mapDataPtr;
	pieceWiseCosts.clear(); pieceWiseCosts.reserve(1000);///@todo not nice!
	resizeInitNew();
    }
    public   : const size_t& knotLatIdx() const { return knotLatIdx_; }
    
    protected: std::vector< CostFuncLatMap1WeightPtr<Lattice,MapData> > pieceWiseCosts;
    protected: bool   finish_;
    public   : size_t iterIdx_;
    public   : size_t iterIdxPartBegin_;
    protected: std::shared_ptr<Lattice> lattFuncPtr_;
    protected: std::shared_ptr<Lattice> lattKnotPtr_;
    protected: std::shared_ptr<MapData> mapDataPtr_;
    private  : size_t knotLatIdx_;
    private  : double weight_;
};


template<typename Lattice, typename MapData>
class CFLatMap1Weight : public CFLatMap1WeightBase<Lattice, MapData> {};


class funcPredef_FL1 {
public:
    template<typename CF> static void lin(CF* _cf) { _cf->fL1 = [_cf](const size_t& _i ) { return _cf->stAcc(_i); }; }
    template<typename CF> static void sqr(CF* _cf) { _cf->fL1 = [_cf](const size_t& _i ) { return _cf->stAcc(_i) * _cf->stAcc(_i); }; }
    template<typename CF> static void ln (CF* _cf) { _cf->fL1 = [_cf](const size_t& _i ) { return log( _cf->stAcc(_i) ); }; }
};
class funcPredef_FL2 {
public:
    template<typename CF> static void sum    (CF* _cf) { _cf->fL2 = [_cf]( const double& _ansPrev, const size_t& _i ) { return _ansPrev + _cf->fL1(_i);        }; }
    template<typename CF> static void min    (CF* _cf) { _cf->fL2 = [_cf]( const double& _ansPrev, const size_t& _i ) { return fmin( _ansPrev, _cf->fL1(_i) ); }; }
    template<typename CF> static void max    (CF* _cf) { _cf->fL2 = [_cf]( const double& _ansPrev, const size_t& _i ) { return fmax( _ansPrev, _cf->fL1(_i) ); }; }
    template<typename CF> static void intTrap(CF* _cf) { _cf->fL2 = [_cf]( const double& _ansPrev, const size_t& _i ) { 
	if( ( _cf->iterIdx_ == 0 ) ) { return _cf->cost0_; } return _ansPrev + ( _cf->arcAcc(_i) - _cf->arcAcc(_i-1) ) * (_cf->fL1(_i) + _cf->fL1(_i-1) ) / 2.; }; 
    }
};
class funcPredef_FL3 {
public:
    template<typename CF> static void weight    (CF* _cf) { _cf->fL3 = [_cf]( const double& _ansPrev ) { return _ansPrev * _cf->weight_;      }; }
    template<typename CF> static void weightBy2 (CF* _cf) { _cf->fL3 = [_cf]( const double& _ansPrev ) { return _ansPrev * _cf->weight_ / 2.; }; }
    template<typename CF> static void weightNorm(CF* _cf) { _cf->fL3 = [_cf]( const double& _ansPrev ) { 
	int beginIdx = _cf->latBeginIdx_;
	if     ( _cf->iterIdx_ == 0 )                 { return 0.; } 
	/*else if( _cf->iterIdx_ == _cf->latBeginIdx_ ) { */beginIdx = std::max( 0, (int)_cf->latBeginIdx_-1 );/* }*/
	return _ansPrev * _cf->weight_ / ( _cf->arcAcc(_cf->iterIdx_)-_cf->arcAcc(beginIdx) ); }; 
    }
};


template<typename Lattice, typename MapData>
class CfSum        : public CFLatMap1Weight<Lattice, MapData> { public: CfSum       () { funcPredef_FL1::lin(this); funcPredef_FL2::sum    (this); funcPredef_FL3::weight    (this); } };
template<typename Lattice, typename MapData>
class CfSumSqr     : public CFLatMap1Weight<Lattice, MapData> { public: CfSumSqr    () { funcPredef_FL1::sqr(this); funcPredef_FL2::sum    (this); funcPredef_FL3::weight    (this); } };
template<typename Lattice, typename MapData>
class CfIntTrapSqr : public CFLatMap1Weight<Lattice, MapData> { public: CfIntTrapSqr() { funcPredef_FL1::sqr(this); funcPredef_FL2::intTrap(this); funcPredef_FL3::weightBy2 (this); } };
template<typename Lattice, typename MapData>
class CfPowTrapSqr : public CFLatMap1Weight<Lattice, MapData> { public: CfPowTrapSqr() { funcPredef_FL1::sqr(this); funcPredef_FL2::intTrap(this); funcPredef_FL3::weightNorm(this); } };





template<typename Lattice, typename MapData, typename CostFunction>
class CostsArrayLat : public CostsArrayLatBase<Lattice, MapData> {
    private  : CostFuncLatMap1WeightPtr<Lattice,MapData> allocateCostFunc() override { return std::unique_ptr<CostFunction>(new CostFunction); }
};


} // namespace cost_functions
 

} // namespace tuw

#endif // COST_FUNCTIONS_H
