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

#ifndef STATE_H
#define STATE_H

#include <float.h>
#include <memory>

namespace tuw {

/*!@class State 
 * 
 * @todo cout the the state variables (horizontally)
 * 
 */
class State;
using StateSPtr      = std::shared_ptr<State>;
using StateConstSPtr = std::shared_ptr<State const>;

using StateUPtr      = std::unique_ptr<State>;
using StateConstUPtr = std::unique_ptr<State const>;

using StateVectorSPtr     = std::shared_ptr<std::vector<State   > >;
using StateSPtrVectorSPtr = std::shared_ptr<std::vector<StateSPtr> >;

class State {
    
    //special class member functions
    public   : State           ()             = default;
    public   : virtual ~State  ()             = default;
    public   : State           (const State&) = default;
    public   : State& operator=(const State&) = default;
    public   : State           (State&&)      = default;
    public   : State& operator=(State&&)      = default;
    
    //pure virtual functions
    ///@brief Clone-to-base-class-ptr function.
    public   : virtual StateUPtr     cloneState () const  = 0;
    ///@brief Size of the full state variables.
    public   : virtual double        stateSize  () const = 0; 
    ///@brief Access state variable based on index @ref _i.
    public   : virtual double&       state ( const std::size_t& _i ) = 0;
    ///@brief Const access state variable based on index @ref _i.
    public   : virtual const double& state ( const std::size_t& _i ) const = 0;
//     ///@brief Adds all state variable with the values of @ref _other.
//     public   : virtual void add ( StatePtr& _other ) = 0;
//     ///@brief Multiplies all state variable with the values of @ref _a.
//     public   : virtual void mlt ( const double _a ) = 0;
    
//     private  : virtual StateUPtr doCloneState () const = 0;
    
    
};

}

#endif // STATE_H