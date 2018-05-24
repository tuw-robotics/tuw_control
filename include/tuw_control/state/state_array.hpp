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

namespace tuw
{
/*!@class StateArray
 * @brief Implementation of @ref State for a fixed size array of double values.
 * @tparam N Number of values contained in the stateArray
 */
template <std::size_t N>
class StateArray;
template <std::size_t N>
using StateArraySPtr = std::shared_ptr<StateArray<N>>;
template <std::size_t N>
using StateArrayConstSPtr = std::shared_ptr<StateArray<N> const>;
template <std::size_t N>
using StateArrayUPtr = std::unique_ptr<StateArray<N>>;
template <std::size_t N>
using StateArrayConstUPtr = std::unique_ptr<StateArray<N> const>;
template <std::size_t N>
class StateArray : public State
{
  // special class member functions
public:
  StateArray(State* _parent) : State(_parent)
  {
    callRootUpdateSize();
  }

public:
  StateArray() : State()
  {
    callRootUpdateSize();
  }

public:
  virtual ~StateArray() = default;

public:
  StateArray(const StateArray&) = default;

public:
  StateArray& operator=(const StateArray&) = default;

public:
  StateArray(StateArray&&) = default;

public:
  StateArray& operator=(StateArray&&) = default;

  // implementation of virtual functions
public:
  virtual StateSPtr cloneState() const override
  {
    return  std::shared_ptr<StateArray<N>>(new StateArray<N>(*this));
  }

public:
  virtual double& value(const std::size_t& _i) override
  {
    return values_[_i];
  }

public:
  virtual const double& value(const std::size_t& _i) const override
  {
    return values_[_i];
  }

public:
  virtual size_t valueSize() const override
  {
    return N;
  }

  ///@brief Reference to the state variables array.
public:
  std::array<double, N>& valuesArray()
  {
    return values_;
  }
  ///@brief Const reference to the variables array.
public:
  const std::array<double, N>& valuesArray() const
  {
    return values_;
  }

protected:
  std::array<double, N> values_;  ///< State array container
};

/*!@class StateArrayScoped
 * @brief Extension of @ref StateArray providing value access based on a scoped enumeration (compile-time).
 * @tparam EnumStateVals Scoped enumeration that defines semantic access to the values of the state array. Has to have
 * ENUM_SIZE representing the number of enum values.
 */
template <typename EnumStateVals>
class StateArrayScoped : public StateArray<asInt(EnumStateVals::ENUM_SIZE)>
{
  // special class member functions
public:
  StateArrayScoped(State* _parent) : StateArray<asInt(EnumStateVals::ENUM_SIZE)>(_parent)
  {
  }

public:
  StateArrayScoped() : StateArray<asInt(EnumStateVals::ENUM_SIZE)>()
  {
  }

public:
  virtual ~StateArrayScoped() = default;

public:
  StateArrayScoped(const StateArrayScoped&) = default;

public:
  StateArrayScoped& operator=(const StateArrayScoped&) = default;

public:
  StateArrayScoped(StateArrayScoped&&) = default;

public:
  StateArrayScoped& operator=(StateArrayScoped&&) = default;

  // implementation of virtual functions
public:
  virtual StateSPtr cloneState() const override
  {
    return std::shared_ptr<StateArrayScoped<EnumStateVals>>(new StateArrayScoped<EnumStateVals>(*this));
  }
  ///@brief Clone-to-this-class-ptr function.
public:
  std::shared_ptr<StateArrayScoped<EnumStateVals>> cloneStateExt() const
  {
    return std::shared_ptr<StateArrayScoped<EnumStateVals>>(new StateArrayScoped<EnumStateVals>(*this));
  }
  ///@brief Scoped access (compile-time) to the values of the state object.
public:
  template <EnumStateVals _i>
  double& value()
  {
    return StateArray<asInt(EnumStateVals::ENUM_SIZE)>::value(asInt(_i));
  }
  ///@brief Const scoped access (compile-time) to the values of the state object.
public:
  template <EnumStateVals _i>
  const double& value() const
  {
    return StateArray<asInt(EnumStateVals::ENUM_SIZE)>::value(asInt(_i));
  }

public:
  using StateArray<asInt(EnumStateVals::ENUM_SIZE)>::value;

public:
  using StateArray<asInt(EnumStateVals::ENUM_SIZE)>::state;

  template <typename... NestedStates1>
  friend class StateNestedSet;
  template <typename EnumStateVals1, typename... NestedStates1>
  friend class StateNestedSetScoped;
  template <typename SubState>
  friend class StateNestedVector;
};
}

#endif  // STATE_ARRAY_H
