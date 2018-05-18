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

#ifndef DISCRETIZATION_RUNGE_KUTTA_ALIAS_HPP
#define DISCRETIZATION_RUNGE_KUTTA_ALIAS_HPP

#include <float.h>
#include <memory>
#include <functional>
#include <tuw_control/utils.h>

namespace tuw
{
class StateSim;
using StateSimPtr = std::shared_ptr<StateSim>;

namespace RungeKutta
{
template <std::size_t StateNmSize, std::size_t RKOrder, typename... RKCoeff>
void discretize(StateSim& _stateSim, const double& _arc);
template <>
void discretize<0, 0>(StateSim& _stateSim, const double& _arc);

using DiscretizationFuncPtr = void (*)(StateSim&, const double&);

///@brief Several discretization modes.
enum class DiscretizationType
{
  USER_DEF,
  EULER,
  HEUN,
  RK4,
  RK4_38
};

///@brief Returns a discretization function pointer for pre-defined Runge-Kutta specializations defined in @ref
/// DiscretizationType
template <std::size_t StateNmSize>
DiscretizationFuncPtr getDiscrFunc(DiscretizationType _discrType)
{
  using Dtp = DiscretizationType;
  switch (_discrType)
  {
    case (Dtp::USER_DEF):
      return discretize<0, 0>;
      break;

    case (Dtp::EULER):
      return discretize<StateNmSize, 1, RatioEval<1, 1> >;
      break;

    case (Dtp::HEUN):
      return discretize<StateNmSize, 2, RatioEval<1, 2>,   // ci  values
                        RatioEval<0, 1>, RatioEval<1, 1>,  // bj  values
                        RatioEval<1, 2>                    // aij values
                        >;
      break;

    case (Dtp::RK4):
      return discretize<StateNmSize, 4, RatioEval<1, 2>, RatioEval<1, 2>, RatioEval<1, 1>,   // ci
                        RatioEval<1, 6>, RatioEval<1, 3>, RatioEval<1, 3>, RatioEval<1, 6>,  // bj
                        RatioEval<1, 2>, RatioEval<0, 1>, RatioEval<1, 2>, RatioEval<0, 1>, RatioEval<0, 1>,
                        RatioEval<1, 1>  // aij
                        >;
      break;

    case (Dtp::RK4_38):
      return discretize<StateNmSize, 4, RatioEval<1, 3>, RatioEval<2, 3>, RatioEval<1, 1>,   // ci
                        RatioEval<1, 8>, RatioEval<3, 8>, RatioEval<3, 8>, RatioEval<1, 8>,  // bj
                        RatioEval<1, 3>, RatioEval<-1, 3>, RatioEval<1, 1>, RatioEval<1, 1>, RatioEval<-1, 1>,
                        RatioEval<1, 1>  // aij
                        >;
      break;
  }
  return nullptr;
}
}
}

#endif  // DISCRETIZATION_RUNGE_KUTTA_ALIAS_HPP