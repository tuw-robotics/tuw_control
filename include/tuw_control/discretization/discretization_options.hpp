/***************************************************************************
 *   Software License Agreement (BSD License)                              *
 *   Copyright (C) 2017 by Horatiu George Todoran <todorangrg@gmail.com>   *
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

#ifndef DISCRETIZATION_OPTIONS_HPP
#define DISCRETIZATION_OPTIONS_HPP

#include <float.h>
#include <memory>
#include <array>

#include <boost/numeric/odeint.hpp>
#include <boost/array.hpp>

#include <algorithm>

namespace tuw
{
namespace odeint = boost::numeric::odeint;

template <class TValue = double>
struct rk4_38_abc
{
  static constexpr const size_t stageNr = 4;
  static constexpr const boost::array<TValue, 1> a1 = { { +1.0 / 3.0 } };
  static constexpr const boost::array<TValue, 2> a2 = { { -1.0 / 3.0, +1.0 } };
  static constexpr const boost::array<TValue, 3> a3 = { { +1.0, -1.0, +1.0 } };
  static constexpr const boost::array<TValue, stageNr> b = { { +1.0 / 8.0, +3.0 / 8.0, +3.0 / 8.0, +1.0 / 8.0 } };
  static constexpr const boost::array<TValue, stageNr> c = { { +0.0, +1.0 / 3.0, +2.0 / 3.0, +1.0 } };
  static auto a()
  {
    return std::move(boost::fusion::make_vector(a1, a2, a3));
  }
};
template <class Value>
constexpr const boost::array<Value, 1> rk4_38_abc<Value>::a1;
template <class Value>
constexpr const boost::array<Value, 2> rk4_38_abc<Value>::a2;
template <class Value>
constexpr const boost::array<Value, 3> rk4_38_abc<Value>::a3;
template <class Value>
constexpr const boost::array<Value, rk4_38_abc<Value>::stageNr> rk4_38_abc<Value>::b;
template <class Value>
constexpr const boost::array<Value, rk4_38_abc<Value>::stageNr> rk4_38_abc<Value>::c;

template <class TValue = double>
struct heun_abc
{
  static constexpr const size_t stageNr = 2;
  static constexpr const boost::array<TValue, 1> a1 = { { 1.0 / 2.0 } };
  static constexpr const boost::array<TValue, stageNr> b = { { 0.0, 1.0 } };
  static constexpr const boost::array<TValue, stageNr> c = { { 0.0, 1.0 / 2.0 } };
  static auto a()
  {
    return std::move(boost::fusion::make_vector(a1));
  }
};
template <class Value>
constexpr const boost::array<Value, 1> heun_abc<Value>::a1;
template <class Value>
constexpr const boost::array<Value, heun_abc<Value>::stageNr> heun_abc<Value>::b;
template <class Value>
constexpr const boost::array<Value, heun_abc<Value>::stageNr> heun_abc<Value>::c;

template <class TValue = double>
struct euler_abc
{
  static constexpr const size_t stageNr = 1;
  static constexpr const boost::array<TValue, 0> a1 = { {} };
  static constexpr const boost::array<TValue, stageNr> b = { { 1.0 } };
  static constexpr const boost::array<TValue, stageNr> c = { { 1.0 } };
  static auto a()
  {
    return std::move(boost::fusion::make_vector());
  }
};
template <class Value>
constexpr const boost::array<Value, 0> euler_abc<Value>::a1;
template <class Value>
constexpr const boost::array<Value, euler_abc<Value>::stageNr> euler_abc<Value>::b;
template <class Value>
constexpr const boost::array<Value, euler_abc<Value>::stageNr> euler_abc<Value>::c;

template <template <class> class MethodType, class State, class Value = double, class Deriv = State, class Time = Value,
          class Algebra = odeint::range_algebra, class Operations = odeint::default_operations,
          class Resizer = odeint::initially_resizer>
class explicit_generic_rk_impl
    : public odeint::explicit_generic_rk<MethodType<Value>::stageNr, MethodType<Value>::stageNr, State, Value, Deriv,
                                         Time, Algebra, Operations, Resizer>
{
public:
  using stepper_base_type = odeint::explicit_generic_rk<MethodType<Value>::stageNr, MethodType<Value>::stageNr, State,
                                                        Value, Deriv, Time, Algebra, Operations, Resizer>;
  using state_type = typename stepper_base_type::state_type;
  using wrapped_state_type = typename stepper_base_type::wrapped_state_type;
  using value_type = typename stepper_base_type::value_type;
  using deriv_type = typename stepper_base_type::deriv_type;
  using wrapped_deriv_type = typename stepper_base_type::wrapped_deriv_type;
  using time_type = typename stepper_base_type::time_type;
  using algebra_type = typename stepper_base_type::algebra_type;
  using operations_type = typename stepper_base_type::operations_type;
  using resizer_type = typename stepper_base_type::resizer_type;
  using stepper_type = typename stepper_base_type::stepper_type;

  explicit_generic_rk_impl(const algebra_type &algebra = algebra_type())
    : stepper_base_type(MethodType<Value>::a(), MethodType<Value>::b, MethodType<Value>::c, algebra)
  {
  }

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

template <size_t HistSize, class TRKType>
using explicit_adams_bashforth =
    odeint::adams_bashforth<HistSize, typename TRKType::state_type, typename TRKType::value_type,
                            typename TRKType::deriv_type, typename TRKType::time_type, typename TRKType::algebra_type,
                            typename TRKType::operations_type, typename TRKType::resizer_type, TRKType>;
}

#endif  // DISCRETIZATION_OPTIONS_HPP