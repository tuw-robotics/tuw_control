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
#include <boost/array.hpp>

#include <algorithm>

namespace tuw
{
namespace RungeKutta
{
namespace
{
template <std::size_t RKOrder>
inline constexpr const std::size_t aijIdx(const std::size_t& _i, const std::size_t& _j)
{
  return 2 * RKOrder - 1 + _i * (_i + 1) / 2 + _j;
}
template <std::size_t RKOrder>
inline constexpr const std::size_t ciIdx(const std::size_t& _i)
{
  return _i;
}
template <std::size_t RKOrder>
inline constexpr const std::size_t bjIdx(const std::size_t& _j)
{
  return RKOrder - 1 + _j;
}
}

/** @brief Templetized generic discretization function.
 *
 *  Performs variants of the systematic Runge-Kutta methods.
 *  @tparam StateNmSize Size of the numerically-computed state.
 *  @tparam RKOrder Order of the Runge-Kutta method.
 *  @tparam RKCoeff Butcher tableau coefficients of the method. The specification order is c2..cn, b1..bn, a21 a31 a32
 *a41 a42 a43...ann-1
 *  @param _stateSim Entire simulated state. The function starts from its input values and modifies it according to the
 *discretization step.
 *  @param _arc      Evaluation point of the closed-form functions/state.
 */
template <std::size_t StateNmSize, std::size_t RKOrder, typename... RKCoeff>
void discretize(StateSim& _stateSim, const double& _arc)
{
  static constexpr const std::array<double, sizeof...(RKCoeff)> coeff = {
    { RKCoeff::val... }
  };                                                       // RungeKutta coefficients (specialized on order).
  static std::array<StateArray<StateNmSize>, RKOrder> dX;  // Array storing the partial continuous-time transition
                                                           // function evaluations of the system (specialized on state
                                                           // numeric variable size and order).
  static StateArray<StateNmSize> x0, deltaXi, deltaX;      // Helper variables (Specialized on state size).
  static double _arc0, _dArc;
  _arc0 = _stateSim.stateArc();
  _dArc = _arc - _arc0;
  if (_stateSim.stateNm().valueSize() != StateNmSize)
  {
    throw "Wrong specialization of RungeKutta::discretize called (system state value size != function state value "
          "size) !";
  }

  _stateSim.setStateCfNmStep(_arc0);
  State& stateDot = _stateSim.stateNmDot();
  const double& b0 = coeff[bjIdx<RKOrder>(0)];

  for (std::size_t si = 0; si < StateNmSize; ++si)
  {
    x0.value(si) = _stateSim.stateNm().value(si);
    dX[0].value(si) = stateDot.value(si);
    deltaX.value(si) = b0 * dX[0].value(si);
  }

  for (std::size_t i = 0; i < RKOrder - 1; ++i)
  {
    deltaXi.valuesArray().fill(0);
    for (std::size_t j = 0; j <= i; ++j)
    {
      const double& aij = coeff[aijIdx<RKOrder>(i, j)];
      for (std::size_t si = 0; si < StateNmSize; ++si)
      {
        deltaXi.value(si) += aij * dX[j].value(si);
      }
    }  // computes deltaX for step i

    for (std::size_t si = 0; si < StateNmSize; ++si)
    {
      _stateSim.stateNm().value(si) = x0.value(si) + _dArc * deltaXi.value(si);
    }
    _stateSim.setStateCfNmStep(
        _arc0 + _dArc * coeff[ciIdx<RKOrder>(i)],
        ParamFuncs::EvalArcGuarantee::AFTER_LAST);  // set the closed form state at new evaluation arc
    _stateSim.stateNmDot();                         // compute continuous time state transition

    const double& bipp = coeff[bjIdx<RKOrder>(i + 1)];
    for (std::size_t si = 0; si < StateNmSize; ++si)
    {
      dX[i + 1].value(si) = stateDot.value(si);
      deltaX.value(si) += bipp * dX[i + 1].value(si);
    }  // store new transition and combine partial answers of the numerical state
  }    // computes all partial deltaXi of the numerical state for all steps
  _stateSim.setStateCf(_arc0 + _dArc, ParamFuncs::EvalArcGuarantee::AFTER_LAST);
  for (std::size_t si = 0; si < StateNmSize; ++si)
  {
    _stateSim.stateNm().value(si) = x0.value(si) + _dArc * deltaX.value(si);
  }  // set final state to the container
}

///@brief Specialization for using a user-defined discretization function.
template <>
inline void discretize<0, 0>(StateSim& _stateSim, const double& _arc)
{
  static double _dArc;
  _dArc = _arc - _stateSim.stateArc();
  _stateSim.setStateCf(_arc, ParamFuncs::EvalArcGuarantee::AFTER_LAST);
  State& stateDelta = _stateSim.stateNmDelta(_dArc);
  for (std::size_t si = 0; si < _stateSim.stateNm().valueSize(); ++si)
  {
    _stateSim.stateNm().value(si) += stateDelta.value(si);
  }  // set final state to the container
}
}
}

#endif  // DISCRETIZATION_RUNGE_KUTTA_HPP