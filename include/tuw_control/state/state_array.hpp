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

#ifndef STATE_ARRAY_H
#define STATE_ARRAY_H

#include <float.h>
#include <memory>
#include <array>

#include <tuw_control/utils.h>
#include <tuw_control/state/state.h>

namespace tuw {

template<std::size_t N> 
class StateArray;

template<std::size_t N> 
using StateArraySPtr      = std::shared_ptr<StateArray<N> >;
template<std::size_t N> 
using StateArrayConstSPtr = std::shared_ptr<StateArray<N> const>;

template<std::size_t N> 
using StateArrayUPtr      = std::unique_ptr<StateArray<N> >;
template<std::size_t N> 
using StateArrayConstUPtr = std::unique_ptr<StateArray<N> const>;
    
template<std::size_t N>
class StateArray : public State {
    
    //special class member functions
    public   : StateArray           ()                  = default;
    public   : virtual ~StateArray  ()                  = default;
    public   : StateArray           (const StateArray&) = default;
    public   : StateArray& operator=(const StateArray&) = default;
    public   : StateArray           (StateArray&&)      = default;
    public   : StateArray& operator=(StateArray&&)      = default;
    
    //implementation of virtual functions
    public   : virtual StateSPtr     cloneState      () const  override { return std::make_shared< StateArray<N> >(*this); }
    public   : virtual double        stateSize       () const  override { return N; } 
    public   : virtual double&       state           ( const std::size_t& _i )       override { return state_[_i]; }
    public   : virtual const double& state           ( const std::size_t& _i ) const override { return state_[_i]; }
    //public   : virtual void add ( StatePtr& _other )                override { for ( size_t i = 0; i < N; i++ ) { state_[i] += _other->state(i); } }
    //public   : virtual void mlt ( const double _a  )                override { for ( size_t i = 0; i < N; i++ ) { state_[i] *= _a; } }
    
    ///@brief Reference to the state array.
    public   :       std::array<double, N>& stateArray ()       { return state_; }
    ///@brief Const reference to the state array.
    public   : const std::array<double, N>& stateArray () const { return state_; }
    ///@brief State array container.
    private  : std::array<double, N> state_;
};

}

#endif // STATE_ARRAY_H