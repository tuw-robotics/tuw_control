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

#ifndef STATE_SIM_TEMPLATE_H
#define STATE_SIM_TEMPLATE_H

#include <tuw_control/state/state_sim.h>
#include <tuw_control/discretization/discretization_runge_kutta.hpp>

namespace tuw {

    
template<std::size_t StateSize, std::size_t StateNmSize>
class StateSimTemplate;

template<std::size_t StateSize, std::size_t StateNmSize> 
using StateSimTemplateSPtr      = std::shared_ptr<StateSimTemplate<StateSize, StateNmSize> >;
template<std::size_t StateSize, std::size_t StateNmSize>
using StateSimTemplateConstSPtr = std::shared_ptr<StateSimTemplate<StateSize, StateNmSize> const>;

template<std::size_t StateSize, std::size_t StateNmSize>
using StateSimTemplateUPtr      = std::unique_ptr<StateSimTemplate<StateSize, StateNmSize> >;
template<std::size_t StateSize, std::size_t StateNmSize>
using StateSimTemplateConstUPtr = std::unique_ptr<StateSimTemplate<StateSize, StateNmSize> const>;
    
template<std::size_t StateSize, std::size_t StateNmSize>
class StateSimTemplate : public StateSim {
    
    //special class member functions
    public   : StateSimTemplate ()                                      = default;
    public   : virtual ~StateSimTemplate ()                             = default;
    public   : StateSimTemplate           (const StateSimTemplate& _o)  = default;
    public   : StateSimTemplate& operator=(const StateSimTemplate& _o)  = default;
    public   : StateSimTemplate           (StateSimTemplate&&)          = default;
    public   : StateSimTemplate& operator=(StateSimTemplate&&)          = default;
    
    //implemented virtual functions
    public   : virtual StateUPtr cloneState () const override {
	StateUPtr retState = make_unique< StateArray<StateSize> >();
	const std::size_t sNmS = stateNm_.stateSize();
	for( std::size_t i = 0; i < sNmS                ; i++ ){ retState->state(i     ) = stateNm_.state(i); } 
	for( std::size_t i = 0; i < stateCf_.stateSize(); i++ ){ retState->state(i+sNmS) = stateCf_.state(i); } 
	return retState;
    }
    public   : void toState0 () override { 
	setStateCf (0, ParamFuncs::EvalArcGuarantee::AT_BEGIN);
	const std::size_t sNmS = stateNm_.stateSize();
	for( std::size_t i = 0; i < sNmS                ; i++ ){ stateNm_.state(i) = state0_.state(i     ); } 
	for( std::size_t i = 0; i < stateCf_.stateSize(); i++ ){ stateCf_.state(i) = state0_.state(i+sNmS); } 
    }
    public   : void setDiscrType ( const RungeKutta::DiscretizationType& _discrType ) override { 
	discrFunc_ = RungeKutta::getDiscrFunc<StateNmSize>(_discrType); 
    }
    public   : void setState     ( StateUPtr& _otherState ) override {
	setStateCf (0, ParamFuncs::EvalArcGuarantee::AT_BEGIN);
	const std::size_t sNmS = stateNm_.stateSize();
	for( std::size_t i = 0; i < sNmS                ; i++ ){ stateNm_.state(i) = _otherState->state(i     ); } 
	for( std::size_t i = 0; i < stateCf_.stateSize(); i++ ){ stateCf_.state(i) = _otherState->state(i+sNmS); } 
    }
    
    public   : State&        state0    ()                              override { return state0_;   }
    public   : State&        stateNm   ()                              override { return stateNm_;  }
    public   : State&        stateCf   ()                              override { return stateCf_;  }
    public   : double        stateSize ()                        const override { return StateSize; }
    public   : double&       state     ( const std::size_t& _i )       override { if ( _i < StateNmSize ) { return stateNm_.state(_i); } else { return stateCf_.state(_i-StateNmSize); } };
    public   : const double& state     ( const std::size_t& _i ) const override { if ( _i < StateNmSize ) { return stateNm_.state(_i); } else { return stateCf_.state(_i-StateNmSize); } };
    public   : void          advance   ( double _arc   )               override { discrFunc_( *this, _arc ); }
    
    ///@brief State array storing the initial state.
    protected: StateArray<StateSize              > state0_;
    ///@brief State array storing the numerical-computed state.
    protected: StateArray<StateNmSize            > stateNm_ ;
    ///@brief State array caching the evaluation of the last call of the state transition function.
    protected: StateArray<StateNmSize            > stateNmDotCache_;
    ///@brief State array storing the closed-form-computed state.
    protected: StateArray<StateSize - StateNmSize> stateCf_ ;
    ///@brief Pointer to the active discretization-method function.
    private  : RungeKutta::DiscretizationFuncPtr   discrFunc_;
    
    private  : using StateSim::setStateCf;
};

}

#endif // STATE_SIM_TEMPLATE_H