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

#ifndef STATE_SIM_H
#define STATE_SIM_H

#include <float.h>
#include <memory>
#include <vector>

#include <tuw_control/state/state.h>
#include <tuw_control/state/state_array.hpp>
#include <tuw_control/discretization/discretization_runge_kutta_alias.hpp>
#include <tuw_control/param_func/param_func.h>
#include <tuw_control/param_func/param_func_dist.h>

namespace tuw {

/*! @class StateSim 
 *  @todo cout the the state variables (horizontally) (override to split on Nm and Cf?)
 */
    
class StateSim;
using StateSimSPtr      = std::shared_ptr<StateSim>;
using StateSimConstSPtr = std::shared_ptr<StateSim const>;
using StateSimUPtr      = std::unique_ptr<StateSim>;
using StateSimConstUPtr = std::unique_ptr<StateSim const>;
class StateSim : public State {
    
    //special class member functions
    public   : StateSim           ()                = default;
    public   : virtual ~StateSim  ()                = default;
    public   : StateSim           (const StateSim&) = default;
    public   : StateSim& operator=(const StateSim&) = default;
    public   : StateSim           (StateSim&&)      = default;
    public   : StateSim& operator=(StateSim&&)      = default;
    
    //pure virtual functions
    ///@brief Clone-to-base-class-ptr function.
    public   : virtual StateSimUPtr cloneStateSim () const = 0;
    ///@brief Set discretization type used in the simulation.
    public   : virtual void         setDiscrType  ( const RungeKutta::DiscretizationType& _discrType ) = 0;
    ///@brief Performs one simulation step to parametrized arc length @ref _arc.
    public   : virtual void         advance       ( double _arc ) = 0;
    ///@brief Resets entire state to @ref state0.
    public   : virtual void         toState0      () = 0;
    ///@brief Reference to the initial state.
    public   : virtual State&       state0        () = 0;
    ///@brief Reference to the actual numerical-computed state.
    public   : virtual State&       stateNm       () = 0;
    ///@brief Reference to the actual closed-form state.
    public   : virtual State&       stateCf       () = 0;
    ///@brief Value of the current arc parametrization of the state.
    public   : virtual double       stateArc      () const = 0;
    ///@brief Value of the current traveled distance of the state.
    public   : virtual double       stateDist     () const = 0;
    
    ///@brief Sets closed-form state at arc @ref _arc.
    protected: virtual void     setStateCf       ( const double& _arc, const ParamFuncs::EvalArcGuarantee& _evalArcGuarantee = ParamFuncs::EvalArcGuarantee::AFTER_LAST  ) = 0;
    ///@brief Computes numerical continuous arc state transition (@ref return) based on internal closed-form state (@ref stateCf ).
    private  : virtual State&   stateNmDot       () = 0;
    ///@brief Computes numerical discrete state transition (@ref return) based on internal closed-form state (@ref stateCf ) and discretization interval @ref _dArc.
    private  : virtual State&   stateNmDelta     ( const double& _dArc ) = 0;
    
    template<std::size_t StateSize, std::size_t RKOrder, typename... RKCoeff>
    friend void RungeKutta::discretize( StateSim& _stateSim, const double& _arc );
    
    ///@todo documentation
    public   : virtual ParamFuncs&       paramFuncsManip () = 0;
};

}

#endif // STATE_SIM_H