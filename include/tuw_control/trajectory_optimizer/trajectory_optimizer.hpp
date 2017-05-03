/***************************************************************************
 *   Software License Agreement (BSD License)                              *
 *   Copyright (C) 2017 by Horatiu George Todoran <todorangrg@gmail.com>   *
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

#ifndef TRAJECTORY_OPTIMIZER_HPP
#define TRAJECTORY_OPTIMIZER_HPP

#include <tuw_control/utils.h>
#include <eigen3/Eigen/Eigen>

namespace tuw {

    
template<class TNumType>
bool isSame( const std::vector<TNumType>& _optVar0, const std::vector<TNumType>& _optVar1 ) {
    const size_t optVar0Size =  _optVar0.size();
    if ( optVar0Size != _optVar1.size() ) { return false; }
    for(size_t i = 0; i < optVar0Size; ++i) { if( fabs(_optVar0[i] - _optVar1[i])  > 2*FLT_MIN ) { return false; } }
    return true;
}
   
enum class OptCacheType { ITER_START, LAST, ENUM_SIZE };
   
template<class TNumType, class TTrajSim, class TMyParamType, template<class, class> class TOptVarMap>
class CostsEvaluatorCached : public TTrajSim {
    public   : CostsEvaluatorCached() : TTrajSim() {
	for ( size_t i = 0; i < asInt(OptCacheType::ENUM_SIZE); ++i ) {
	    xCacheConstr_    [i] = std::make_shared<std::vector<TNumType>           >();
	    xCacheGradConstr_[i] = std::make_shared<std::vector<TNumType>           >();
	    for(size_t j=0;j<TTrajSim::CostFuncsTypesNr; ++j){ 
		constCache_      [i][j] = std::make_shared<Eigen::Matrix<TNumType, -1,  1> >();
		gradConstrCache_ [i][j] = std::make_shared<Eigen::Matrix<TNumType, -1, -1> >();
	    }
	}
    }
    public   : auto& costs    () { return this->costs_; }
    public   : auto& gradCosts() { return this->gradCostsMap_; }
    public   : bool evaluateCosts( const std::vector<TNumType> & _x, const OptCacheType& _cacheType = OptCacheType::LAST ) {
	bool cached = false;
	for ( size_t i = 0; i < asInt(OptCacheType::ENUM_SIZE); ++i ) {
	    if ( isSame(_x, *xCacheConstr_[i]) ) { 
		for(size_t j=0;j<TTrajSim::CostFuncsTypesNr; ++j){ this->costs_.sub(j).data() = *constCache_[i][j]; } 
		cached = true; return cached; 
	    } 
	}
	if ( !cached ) {
// 	    std::cout<<"computing it!";
	    setOptVar(_x);
	    this->simulateTrajectory();
	    *xCacheConstr_[asInt(_cacheType)] = _x;
	    for(size_t j=0;j<TTrajSim::CostFuncsTypesNr; ++j){ 
		*constCache_  [asInt(_cacheType)][j] = this->costs_.sub(j).data();
	    }
	}
	return cached; 
    }
    public   : bool evaluateCostsWithGrad( const std::vector<TNumType> & _x, const OptCacheType& _cacheType = OptCacheType::LAST ) {
	bool cached = false;
	for ( size_t i = 0; i < asInt(OptCacheType::ENUM_SIZE); ++i ) {
	    if ( isSame(_x, *xCacheGradConstr_[i]) ) { 
		for(size_t j=0;j<TTrajSim::CostFuncsTypesNr; ++j) { 
		    this->costs_.sub(j).data() = *constCache_[i][j]; 
		    *this->gradCostsMap_[j] = *gradConstrCache_[i][j]; 
		} 
		cached = true; return cached; 
	    } 
	}
	if ( !cached ) {
// 	    std::cout<<"computing it!";
	    setOptVar(_x);
	    this->simulateTrajectoryWithGrad();
	    *xCacheConstr_   [asInt(_cacheType)] = *xCacheGradConstr_[asInt(_cacheType)] = _x;
	    for(size_t j=0;j<TTrajSim::CostFuncsTypesNr; ++j){ 
		*constCache_     [asInt(_cacheType)][j] = this->costs_.sub(j).data();
		*gradConstrCache_[asInt(_cacheType)][j] = *this->gradCostsMap_[j];
	    }
	}
	return cached; 
    }
    public   : void getOptVar(       std::vector<TNumType>& _optVarExt ) { TOptVarMap<TNumType, TMyParamType>::getOptVar(_optVarExt, *this->stateSim()->paramStruct.get()); }
    private  : void setOptVar( const std::vector<TNumType>& _optVarExt ) { TOptVarMap<TNumType, TMyParamType>::setOptVar(*this->stateSim()->paramStruct.get(), _optVarExt); }
    private  : std::array<std::shared_ptr<std::vector<TNumType>           >, asInt(OptCacheType::ENUM_SIZE)> xCacheConstr_;
    private  : std::array<std::shared_ptr<std::vector<TNumType>           >, asInt(OptCacheType::ENUM_SIZE)> xCacheGradConstr_;
    private  : std::array<std::array<std::shared_ptr<Eigen::Matrix<TNumType, -1,  1> >, TTrajSim::CostFuncsTypesNr>, asInt(OptCacheType::ENUM_SIZE)> constCache_;
    private  : std::array<std::array<std::shared_ptr<Eigen::Matrix<TNumType, -1, -1> >, TTrajSim::CostFuncsTypesNr>, asInt(OptCacheType::ENUM_SIZE)> gradConstrCache_;
    
    template<class TNumType2, class TTrajSimJGH2, class TTrajSimJ2, class TTrajSimG2, class TTrajSimH2, class TMyParamType2, template<class, class> class TOptVarMap2> friend class TrajectoryOptimizer;
};
    
template<class TNumType, class TTrajSimJGH, class TTrajSimJ, class TTrajSimG, class TTrajSimH, class TMyParamType, template<class, class> class TOptVarMap>
struct  TrajectoryOptimizer {
    TrajectoryOptimizer() {
	static constexpr const size_t idxIterStart = asInt(OptCacheType::ITER_START);
	trajSimJ.xCacheConstr_[idxIterStart] = trajSimJGH.xCacheConstr_[idxIterStart];
	trajSimG.xCacheConstr_[idxIterStart] = trajSimJGH.xCacheConstr_[idxIterStart];
	trajSimH.xCacheConstr_[idxIterStart] = trajSimJGH.xCacheConstr_[idxIterStart];
	
	trajSimJ.xCacheGradConstr_[idxIterStart] = trajSimJGH.xCacheGradConstr_[idxIterStart];
	trajSimG.xCacheGradConstr_[idxIterStart] = trajSimJGH.xCacheGradConstr_[idxIterStart];
	trajSimH.xCacheGradConstr_[idxIterStart] = trajSimJGH.xCacheGradConstr_[idxIterStart];
	
	trajSimJ.constCache_[idxIterStart][0] = trajSimJGH.constCache_[idxIterStart][0];
	trajSimG.constCache_[idxIterStart][0] = trajSimJGH.constCache_[idxIterStart][1];
	trajSimH.constCache_[idxIterStart][0] = trajSimJGH.constCache_[idxIterStart][2];
	
	trajSimJ.gradConstrCache_[idxIterStart][0] = trajSimJGH.gradConstrCache_[idxIterStart][0];
	trajSimG.gradConstrCache_[idxIterStart][0] = trajSimJGH.gradConstrCache_[idxIterStart][1];
	trajSimH.gradConstrCache_[idxIterStart][0] = trajSimJGH.gradConstrCache_[idxIterStart][2];
    }
    CostsEvaluatorCached<TNumType, TTrajSimJ  , TMyParamType, TOptVarMap> trajSimJ;
    CostsEvaluatorCached<TNumType, TTrajSimG  , TMyParamType, TOptVarMap> trajSimG;
    CostsEvaluatorCached<TNumType, TTrajSimH  , TMyParamType, TOptVarMap> trajSimH;
    CostsEvaluatorCached<TNumType, TTrajSimJGH, TMyParamType, TOptVarMap> trajSimJGH;
    void initParamStruct(std::shared_ptr<TMyParamType>& _paramStructPtr) {
	trajSimJGH.stateSim()->paramStruct = _paramStructPtr; 
	trajSimJ  .stateSim()->paramStruct = _paramStructPtr; 
	trajSimG  .stateSim()->paramStruct = _paramStructPtr; 
	trajSimH  .stateSim()->paramStruct = _paramStructPtr; 
	paramStructPtr                     = _paramStructPtr;
    }
    std::shared_ptr<TMyParamType>   paramStructPtr;
};

}

#endif // TRAJECTORY_OPTIMIZER_HPP