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

#ifndef STATE_VECTOR_H
#define STATE_VECTOR_H


#include <float.h>
#include <memory>
#include <array>

#include <tuw_control/utils.h>
#include <tuw_control/state/state.h>

namespace tuw {

/*!@class StateVector
 * @brief Implementation of @ref State for a dynamic size vector of double values.
 */
class StateVector;
using StateVectorSPtr      = std::shared_ptr<StateVector >;
using StateVectorConstSPtr = std::shared_ptr<StateVector const>;
using StateVectorUPtr      = std::unique_ptr<StateVector >;
using StateVectorConstUPtr = std::unique_ptr<StateVector const>;

class StateVector : public State {
    
    //special class member functions
    public   : StateVector           (State* _parent) : State(_parent) { callRootUpdateSize(); }
    public   : StateVector           ()               : State()        { callRootUpdateSize(); }
    public   : virtual ~StateVector  ()                   = default;
    public   : StateVector           (const StateVector&) = default;
    public   : StateVector& operator=(const StateVector&) = default;
    public   : StateVector           (StateVector&&)      = default;
    public   : StateVector& operator=(StateVector&&)      = default;
    
    //implementation of virtual functions
    public   : virtual StateSPtr     cloneState      ()                        const override { return std::make_shared< StateVector >(*this); }
    public   : virtual double&       value           ( const std::size_t& _i )       override { return values_[_i]; }
    public   : virtual const double& value           ( const std::size_t& _i ) const override { return values_[_i]; }
    public   : virtual size_t        valueSize       ()                        const override { return values_.size(); }
    public   : virtual void          resize          ( const std::size_t& _i )       override { values_.resize(_i); }
    
    ///@brief Reference to the state variables array.
    public   :       std::vector<double>& valuesVector ()       { return values_; }
    ///@brief Const reference to the variables array.
    public   : const std::vector<double>& valuesVector () const { return values_; }
    protected: std::vector<double> values_;///< State array container
};

}

#endif // STATE_VECTOR_H