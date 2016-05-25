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

#ifndef DISCRETIZATION_RUNGE_KUTTA_HPP
#define DISCRETIZATION_RUNGE_KUTTA_HPP

#include <float.h>
#include <memory>
#include <array>

#include <tuw_control/state/state_array.hpp>
#include <tuw_control/state/state_sim.h>

#include <algorithm>

namespace tuw {

namespace RungeKutta {
    
    namespace {
	template<std::size_t RKOrder> inline constexpr const std::size_t aijIdx(const std::size_t& _i, const std::size_t& _j)  { return 2 * RKOrder - 1 + _i*(_i+1)/2+_j; }
	template<std::size_t RKOrder> inline constexpr const std::size_t ciIdx (const std::size_t& _i)                         { return _i; }
	template<std::size_t RKOrder> inline constexpr const std::size_t bjIdx (const std::size_t& _j)                         { return RKOrder - 1 + _j; }
    }
    
    ///@todo make documentation of the functions
    
    template<std::size_t StateNmSize, std::size_t RKOrder, typename... RKCoeff>
    void discretize ( StateSim& _stateSim, const double& _arc ) {
	static constexpr const std::array<double, sizeof...(RKCoeff)> coeff = { {  RKCoeff::val... } };//RungeKutta coefficients (specialized on order).
	static std::array< StateArray<StateNmSize>, RKOrder> dX;//Array storing the partial continuous-time transition function evaluations of the system (specialized on state numeric variable size and order).
	static StateArray<StateNmSize> x0, deltaXi, deltaX;//Helper variables (Specialized on state size).
	static double _arc0, _dArc;
	_arc0 = _stateSim.stateArc();
	_dArc = _arc - _arc0;
	if( _stateSim.stateNm().stateSize() != StateNmSize ){ throw "Wrong specialization of RungeKutta::discretize called (system state size != function state size) !"; }
	
	State& stateDot = _stateSim.stateNmDot(); const double& b0 = coeff[bjIdx<RKOrder>(0)];
	
	for(std::size_t si=0;si<StateNmSize;++si) { x0.state(si)=_stateSim.stateNm().state(si); dX[0].state(si)= stateDot.state(si); deltaX.state(si)=b0*dX[0].state(si);   }
	
	for(std::size_t i=0;i<RKOrder-1;++i){ 
	    deltaXi.stateArray().fill(0);
	    for(std::size_t j=0;j<=i;++j) { const double& aij = coeff[aijIdx<RKOrder>(i,j)]; for( std::size_t si = 0; si < StateNmSize; ++si ) { deltaXi.state(si) += aij * dX[j].state(si); } }//computes deltaX for step i
	    
	    for( std::size_t si = 0; si <  StateNmSize; ++si ) { _stateSim.stateNm().state(si) = x0.state(si) + _dArc * deltaXi.state(si); }
	    _stateSim.setStateCf ( _arc0 + _dArc * coeff[ciIdx<RKOrder>(i)], ParamFuncs::EvalArcGuarantee::AFTER_LAST );//set the closed form state at new evaluation arc
	    _stateSim.stateNmDot();//compute continuous time state transition
	    
	    const double& bipp = coeff[bjIdx<RKOrder>(i+1)];
	    for(std::size_t si=0;si<StateNmSize;++si) { dX[i+1].state(si) = stateDot.state(si); deltaX.state(si) += bipp * dX[i+1].state(si); }//store new transition and combine partial answers of the numerical state
	}//computes all partial deltaXi of the numerical state for all steps
	_stateSim.setStateCf ( _arc0 + _dArc, ParamFuncs::EvalArcGuarantee::AFTER_LAST );
	for(std::size_t si=0;si<StateNmSize;++si) { _stateSim.stateNm().state(si) = x0.state(si) + _dArc * deltaX.state(si); }//set final state to the container
    }
    
    template<>
    void discretize<0,0> ( StateSim& _stateSim, const double& _arc ) {
	_stateSim.stateNmDelta( _arc - _stateSim.stateArc() );
    }
}

}

#endif // DISCRETIZATION_RUNGE_KUTTA_HPP