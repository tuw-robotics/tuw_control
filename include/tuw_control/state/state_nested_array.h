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

#ifndef STATE_NESTED_ARRAY_H
#define STATE_NESTED_ARRAY_H


#include <float.h>
#include <memory>

#include <tuw_control/state/state.h>
#include <tuw_control/utils.h>

namespace tuw {

/*!@class StateNestedArray
 * 
 * @todo cout the the state variables (horizontally)
 * 
 */

template<typename SubState, size_t N>
class StateNestedArray;
template<typename SubState, size_t N>
using StateNestedArraySPtr      = std::shared_ptr< StateNestedArray<SubState, N> >;
template<typename SubState, size_t N>
using StateNestedArrayConstSPtr = std::shared_ptr< StateNestedArray<SubState, N> const>;
template<typename SubState, size_t N>
using StateNestedArrayUPtr      = std::unique_ptr< StateNestedArray<SubState, N> >;
template<typename SubState, size_t N>
using StateNestedArrayConstUPtr = std::unique_ptr< StateNestedArray<SubState, N> const>;


template<typename SubState, size_t N>
class StateNestedArray : public State {
    public   : StateNestedArray           (State* _parent) : State(_parent) { 
	for(size_t i = 0; i < N; ++i) { states_[i] = std::make_shared< SubState >(this); statesBase_[i] = states_[i]; } 
	this->callRootUpdateSize (); 
    }
    public   : StateNestedArray           ()               : State()        { 
	for(size_t i = 0; i < N; ++i) { states_[i] = std::make_shared< SubState >(this); statesBase_[i] = states_[i]; } 
	this->callRootUpdateSize ();
    }
    public   : virtual ~StateNestedArray  ()                        = default;
    public   : StateNestedArray           (const StateNestedArray&) = default;
    public   : StateNestedArray& operator=(const StateNestedArray&) = default;
    public   : StateNestedArray           (StateNestedArray&&)      = default;
    public   : StateNestedArray& operator=(StateNestedArray&&)      = default;
    
    
    public   : virtual StateSPtr              cloneState       () const override { return std::make_shared< StateNestedArray<SubState, N> >(*this); }
    public   : size_t        stateSize  ()                        const override { return N;               }
    public   : size_t        valueSize  ()                        const override { return valueSize_;      }
    public   : double&       value      ( const std::size_t& _i )       override { return *values_[_i];    }
    public   : const double& value      ( const std::size_t& _i ) const override { return *values_[_i];    }
    public   : StateSPtr&    state      ( const std::size_t& _i )       override { return statesBase_[_i]; }
    public   : std::shared_ptr<SubState>& stateScoped ( const size_t& _i )       { return this->states_[_i]; }
    
    public   : void updateSize () override { 
	valueSize_ = 0; 
	for(auto& stateI : states_){ stateI->updateSize(); valueSize_ += stateI->valueSize(); } 
	values_.resize(valueSize_);
	size_t valueSizeI, valueSizeSum = 0;
	for(auto& stateI : states_){ 
	    valueSizeI = stateI->valueSize();
	    for(size_t i = 0; i < valueSizeI; ++i){ values_[valueSizeSum + i] = &(stateI->value(i)); } 
	    valueSizeSum += valueSizeI;
	}
    }
    
    protected: size_t valueSize_;
    protected: size_t statesSize_ ;
    
    protected: std::array< std::shared_ptr<SubState>, N     > states_;
    protected: std::array< StateSPtr,                 N     > statesBase_;
    protected: std::vector< double*                         > values_;
};

template<typename EnumStateVals, typename SubState>
class StateNestedArrayScoped : public StateNestedArray<SubState, asInt(EnumStateVals::ENUM_SIZE)> {
    
    //special class member functions
    public   : StateNestedArrayScoped           (State* _parent) : StateNestedArray<SubState, asInt(EnumStateVals::ENUM_SIZE)>(_parent) {}
    public   : StateNestedArrayScoped           ()               : StateNestedArray<SubState, asInt(EnumStateVals::ENUM_SIZE)>()        {}
    public   : virtual ~StateNestedArrayScoped  ()                        = default;
    public   : StateNestedArrayScoped           (const StateNestedArrayScoped&) = default;
    public   : StateNestedArrayScoped& operator=(const StateNestedArrayScoped&) = default;
    public   : StateNestedArrayScoped           (StateNestedArrayScoped&&)      = default;
    public   : StateNestedArrayScoped& operator=(StateNestedArrayScoped&&)      = default;
    
    //implementation of virtual functions
    public   : virtual StateSPtr                                                cloneState    () const  override { return std::make_shared< StateNestedArrayScoped<EnumStateVals, SubState> >(*this); }
    public   : std::shared_ptr<StateNestedArrayScoped<EnumStateVals, SubState>> cloneStateExt () const           { return std::make_shared< StateNestedArrayScoped<EnumStateVals, SubState> >(*this); }
    public   : template<EnumStateVals _i>       double& value ()       { return StateNestedArray<SubState, asInt(EnumStateVals::ENUM_SIZE)>::value(asInt(_i)); }
    public   : template<EnumStateVals _i> const double& value () const { return StateNestedArray<SubState, asInt(EnumStateVals::ENUM_SIZE)>::value(asInt(_i)); }
    public   : template<EnumStateVals _i> typename std::shared_ptr<SubState>& state () { return this->states_[asInt(_i)]; }
    public   : using StateNestedArray<SubState, asInt(EnumStateVals::ENUM_SIZE)>::value;
    public   : using StateNestedArray<SubState, asInt(EnumStateVals::ENUM_SIZE)>::state;

    template<                         typename... NestedStates1> friend class StateNestedSet;
    template<typename EnumStateVals1, typename... NestedStates1> friend class StateNestedSetScoped;
    template<typename SubState1                                > friend class StateNestedVector;
    
};

}

#endif // STATE_NESTED_ARRAY_H