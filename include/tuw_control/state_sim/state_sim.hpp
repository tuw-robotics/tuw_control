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

#ifndef STATE_SIM_HPP
#define STATE_SIM_HPP

#include <tuw_control/state_map/state_map.hpp>
#include <tuw_control/state_sim/state_sim_base.hpp>

namespace tuw {

template<class TNum, class TStateNm, class TStateCf>
class StateMapSimBase : public StateMapTuple<TNum, TStateNm, TStateCf> {
    public   : using StateMapSimBaseType = StateMapSimBase;
    public   : using StateMapTuple<TNum, TStateNm, TStateCf>::StateMapTuple;
    public   :       auto& stateNm()       { return this->template sub<0>(); }
    public   : const auto& stateNm() const { return this->template sub<0>(); }
    public   :       auto& stateCf()       { return this->template sub<1>(); }
    public   : const auto& stateCf() const { return this->template sub<1>(); }
    public   :       auto& state  ()       { return *this; }
    public   : const auto& state  () const { return *this; }
};

namespace /*<anonymous>*/ {

template<class TNum, class TStateNm, class TStateCf>
struct StateMapSimBaseTraits<StateMapSimBase<TNum, TStateNm, TStateCf>> {
    public   : using StateType         = StateMapSimBase<TNum, TStateNm, TStateCf>;
    public   : using NumType           = TNum;
    public   : using StateCfType       = TStateCf;
    public   : using StateNmType       = TStateNm;
    public   : using StateVirtType     = StateMapBaseVirt<TNum>;
    public   : using StateNmNumType    = typename TStateNm::StateMapBaseCRTP::MatrixTypeCRTP;
    
    public   : using StateWithGradNmType     = EmptyGradType;
    public   : using StateWithGradNmNumType  = EmptyGradType;
};

} //namespace <anonymous>

template<class TNum, class TStateNmCf, size_t TGradBlockSize>
class GradStateMapNmCfSimBase : public StateMapArray<TNum, StateMapVector<TNum, TStateNmCf>, TGradBlockSize> {
    public   : using StateMapArray<TNum, StateMapVector<TNum, TStateNmCf>, TGradBlockSize>::StateMapArray;
    public   :       auto& dxdpBlockI   (const size_t& _i)                         { return this->sub(_i); }
    public   : const auto& dxdpBlockI   (const size_t& _i)                   const { return this->sub(_i); }
    public   : void        setPBlockSize(const size_t& _i, const size_t& _n)       { this->sub(_i).subResize(_n); }
};

template<class TNum, class TStateNm, class TStateCf, template<class> class TOptVarStruct>
class StateWithGradMapSimBase : public StateMapTuple<TNum, StateMapTuple<TNum, TStateNm, TOptVarStruct<TStateNm>>, StateMapTuple<TNum, TStateCf, TOptVarStruct<TStateCf>>> {
    public   : using StateMapSimBaseType = StateWithGradMapSimBase;
    public   : using StateMapTuple<TNum, StateMapTuple<TNum, TStateNm, TOptVarStruct<TStateNm>>, StateMapTuple<TNum, TStateCf, TOptVarStruct<TStateCf>>>::StateMapTuple;
    public   :       auto& stateWithGradNm()       { return this->template sub<0>(); }
    public   : const auto& stateWithGradNm() const { return this->template sub<0>(); }
    public   :       auto& stateWithGradCf()       { return this->template sub<1>(); }
    public   : const auto& stateWithGradCf() const { return this->template sub<1>(); }
    public   :       auto& stateNm()       { return this->template sub<0>().template sub<0>(); }
    public   : const auto& stateNm() const { return this->template sub<0>().template sub<0>(); }
    public   :       auto& stateCf()       { return this->template sub<1>().template sub<0>(); }
    public   : const auto& stateCf() const { return this->template sub<1>().template sub<0>(); }
    public   :       auto& stateGradNm()       { return this->template sub<0>().template sub<1>(); }
    public   : const auto& stateGradNm() const { return this->template sub<0>().template sub<1>(); }
    public   :       auto& stateGradCf()       { return this->template sub<1>().template sub<1>(); }
    public   : const auto& stateGradCf() const { return this->template sub<1>().template sub<1>(); }
    public   : void        setPBlockSize(const size_t& _i, const size_t& _n)       { stateGradNm().setPSize(_i, _n); stateGradCf().setPSize(_i, _n); }
};

template<class TStateWithGradMapSimType>
class StateWithGradMapBase :  public StateMapTuple<typename StateMapSimBaseTraits<typename TStateWithGradMapSimType::StateMapSimBaseType>::NumType, 
                                                   StateMapTuple<typename StateMapSimBaseTraits<typename TStateWithGradMapSimType::StateMapSimBaseType>::NumType, 
				 		                 typename StateMapSimBaseTraits<typename TStateWithGradMapSimType::StateMapSimBaseType>::StateNmType, 
								 typename StateMapSimBaseTraits<typename TStateWithGradMapSimType::StateMapSimBaseType>::StateCfType>, 
						   StateMapTuple<typename StateMapSimBaseTraits<typename TStateWithGradMapSimType::StateMapSimBaseType>::NumType, 
						                 typename StateMapSimBaseTraits<typename TStateWithGradMapSimType::StateMapSimBaseType>::StateNmGradType, 
								 typename StateMapSimBaseTraits<typename TStateWithGradMapSimType::StateMapSimBaseType>::StateCfGradType> > {
    public   : using StateMapTuple<typename StateMapSimBaseTraits<typename TStateWithGradMapSimType::StateMapSimBaseType>::NumType, 
				   StateMapTuple<typename StateMapSimBaseTraits<typename TStateWithGradMapSimType::StateMapSimBaseType>::NumType, 
				   		 typename StateMapSimBaseTraits<typename TStateWithGradMapSimType::StateMapSimBaseType>::StateNmType, 
						 typename StateMapSimBaseTraits<typename TStateWithGradMapSimType::StateMapSimBaseType>::StateCfType>, 
				   StateMapTuple<typename StateMapSimBaseTraits<typename TStateWithGradMapSimType::StateMapSimBaseType>::NumType, 
						 typename StateMapSimBaseTraits<typename TStateWithGradMapSimType::StateMapSimBaseType>::StateNmGradType, 
						 typename StateMapSimBaseTraits<typename TStateWithGradMapSimType::StateMapSimBaseType>::StateCfGradType> >::StateMapTuple;
						 
    using NumType = typename StateMapSimBaseTraits<typename TStateWithGradMapSimType::StateMapSimBaseType>::NumType;
    
    public   :       auto& state      ()       { return this->template sub<0>(); }
    public   : const auto& state      () const { return this->template sub<0>(); }
    public   :       auto& stateGrad  ()       { return this->template sub<1>(); }
    public   : const auto& stateGrad  () const { return this->template sub<1>(); }
    public   :       auto& stateNm    ()       { return this->template sub<0>().template sub<0>(); }
    public   : const auto& stateNm    () const { return this->template sub<0>().template sub<0>(); }
    public   :       auto& stateCf    ()       { return this->template sub<0>().template sub<1>(); }
    public   : const auto& stateCf    () const { return this->template sub<0>().template sub<1>(); }
    public   :       auto& stateGradNm()       { return this->template sub<1>().template sub<0>(); }
    public   : const auto& stateGradNm() const { return this->template sub<1>().template sub<0>(); }
    public   :       auto& stateGradCf()       { return this->template sub<1>().template sub<1>(); }
    public   : const auto& stateGradCf() const { return this->template sub<1>().template sub<1>(); }
    public   : void        setPBlockSize(const size_t& _i, const size_t& _n)       { stateGradNm().setPSize(_i, _n); stateGradCf().setPSize(_i, _n); }
    
    public   :       Eigen::Matrix<NumType,-1,-1>& stateGradMat  ()       { 
	const size_t cfSize = stateCf().data().size();
	const size_t nmSize = stateNm().data().size();
	gradMat_.resize(cfSize+nmSize, stateGradCf().subSize() * stateGradCf().template sub<0>().subSize()); 
	for(size_t j = 0; j < gradMat_.cols(); ++j) {
	    gradMat_.block(     0,j, nmSize,1) = stateGradNm().data().block(nmSize*j,0, nmSize,1);
	    gradMat_.block(nmSize,j, cfSize,1) = stateGradCf().data().block(cfSize*j,0, cfSize,1);
	}
	return gradMat_;
    }
    
    private  : Eigen::Matrix<NumType,-1,-1> gradMat_;
};


namespace /*<anonymous>*/ {

template<class TNum, class TStateNm, class TStateCf, template<class> class TOptVarStruct>
struct StateMapSimBaseTraits<StateWithGradMapSimBase<TNum, TStateNm, TStateCf, TOptVarStruct>> {
    public   : using StateType         = StateWithGradMapSimBase<TNum, TStateNm, TStateCf, TOptVarStruct>;
    public   : using NumType           = TNum;
    public   : using StateCfType       = TStateCf;
    public   : using StateNmType       = TStateNm;
    public   : using StateCfGradType   = TOptVarStruct<TStateCf>;
    public   : using StateNmGradType   = TOptVarStruct<TStateNm>;
    public   : using StateVirtType     = StateMapBaseVirt<TNum>;
    public   : using StateNmNumType    = typename TStateNm::StateMapBaseCRTP::MatrixTypeCRTP;
    
    public   : using StateWithGradNmType    = StateMapTuple<TNum, TStateNm, TOptVarStruct<TStateNm>>;
    public   : using StateWithGradNmNumType = typename StateMapTuple<TNum, TStateNm, TOptVarStruct<TStateNm>>::StateMapBaseCRTP::MatrixTypeCRTP;
};

} //namespace <anonymous>


template<class TNumType>
using StateSimBaseVirtMap = StateSimBaseVirt<TNumType,StateMapBaseVirt<TNumType>>;

}

#endif // STATE_SIM_HPP