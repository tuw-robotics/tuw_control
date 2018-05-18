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

#ifndef STATE_NESTED_SET_H
#define STATE_NESTED_SET_H

#include <float.h>
#include <memory>

#include <tuw_control/state/state.h>
#include <tuw_control/utils.h>

namespace tuw
{
/*!@class StateNestedSet
 * @brief Implementation of @ref State being formed by tuple of substates.
 * @tparam NestedStates Parameter pack of the underlying substates types.
 */
template <typename... NestedStates>
class StateNestedSet;
template <typename... NestedStates>
using StateNestedSetSPtr = std::shared_ptr<StateNestedSet<NestedStates...> >;
template <typename... NestedStates>
using StateNestedSetConstSPtr = std::shared_ptr<StateNestedSet<NestedStates...> const>;
template <typename... NestedStates>
using StateNestedSetUPtr = std::unique_ptr<StateNestedSet<NestedStates...> >;
template <typename... NestedStates>
using StateNestedSetConstUPtr = std::unique_ptr<StateNestedSet<NestedStates...> const>;
template <typename... NestedStates>
class StateNestedSet : public State
{
  // special class member functions
public:
  StateNestedSet(State* _parent) : State(_parent), isInit_(false)
  {
    states_ = std::make_tuple(std::make_shared<NestedStates>(this)...);
    statesBase_ = std::initializer_list<StateSPtr>{ std::make_shared<NestedStates>(this)... };
    isInit_ = true;
    callRootUpdateSize();
  }

public:
  StateNestedSet() : State(), isInit_(false)
  {
    states_ = std::make_tuple(std::make_shared<NestedStates>(this)...);
    statesBase_ = std::initializer_list<StateSPtr>{ std::make_shared<NestedStates>(this)... };
    isInit_ = true;
    callRootUpdateSize();
  }

public:
  virtual ~StateNestedSet() = default;

public:
  StateNestedSet(const StateNestedSet&) = default;

public:
  StateNestedSet& operator=(const StateNestedSet&) = default;

public:
  StateNestedSet(StateNestedSet&&) = default;

public:
  StateNestedSet& operator=(StateNestedSet&&) = default;

public:
  virtual StateSPtr cloneState() const override
  {
    return std::make_shared<StateNestedSet<NestedStates...> >(*this);
  }

public:
  size_t stateSize() const override
  {
    return statesSize_;
  }

public:
  size_t valueSize() const override
  {
    return valueSize_;
  }

public:
  double& value(const std::size_t& _i) override
  {
    return *values_[_i];
  }

public:
  const double& value(const std::size_t& _i) const override
  {
    return *values_[_i];
  }

public:
  StateSPtr& state(const std::size_t& _i) override
  {
    return statesBase_[_i];
  }

public:
  void updateSize() override
  {
    if (!isInit_)
    {
      return;
    }
    valueSize_ = 0;
    size_t _i = 0;
    for_each_tuple(states_, [this, &_i](StateSPtr stateI)
                   {
                     statesBase_[_i] = stateI;
                     _i++;
                     stateI->updateSize();
                     valueSize_ += stateI->valueSize();
                   });
    values_.resize(valueSize_);
    size_t valSizeSum = 0;
    auto funcBindVals = [this, &valSizeSum](StateSPtr stateI)
    {
      size_t valSizeI = stateI->valueSize();
      for (size_t i = 0; i < valSizeI; ++i)
      {
        values_[valSizeSum + i] = &stateI->value(i);
      }
      valSizeSum += valSizeI;
    };
    for_each_tuple(states_, funcBindVals);
  }

protected:
  size_t valueSize_;

protected:
  static constexpr const size_t statesSize_ = sizeof...(NestedStates);

protected:
  std::tuple<std::shared_ptr<NestedStates>...> states_;

protected:
  std::vector<StateSPtr> statesBase_;

protected:
  std::vector<double*> values_;

protected:
  bool isInit_;
};

/*!@class StateNestedSetScoped
 * @brief Extension of @ref StateNestedSet providing sub-state access based on a scoped enumeration (compile-time).
 * @tparam EnumStateVals Scoped enumeration that defines semantic access to the values of the state array. Has to have
 * ENUM_SIZE representing the number of enum values.
 * @tparam NestedStates Parameter pack of the underlying substates types.
 */
template <typename EnumStateVals, typename... NestedStates>
class StateNestedSetScoped : public StateNestedSet<NestedStates...>
{
  // special class member functions
public:
  StateNestedSetScoped(State* _parent) : StateNestedSet<NestedStates...>(_parent)
  {
  }

public:
  StateNestedSetScoped() : StateNestedSet<NestedStates...>()
  {
  }

public:
  virtual ~StateNestedSetScoped() = default;

public:
  StateNestedSetScoped(const StateNestedSetScoped&) = default;

public:
  StateNestedSetScoped& operator=(const StateNestedSetScoped&) = default;

public:
  StateNestedSetScoped(StateNestedSetScoped&&) = default;

public:
  StateNestedSetScoped& operator=(StateNestedSetScoped&&) = default;

public:
  using StateNestedSet<NestedStates...>::value;

public:
  using StateNestedSet<NestedStates...>::state;
  // public   : template<EnumStateVals _i>       double& value ()       { return *this->values_[asInt(_i)]; }
  // public   : template<EnumStateVals _i> const double& value () const { return *this->values_[asInt(_i)]; }
  ///@brief Scoped access (compile-time) to the sub-states of the state object.
public:
  template <EnumStateVals _i>
  typename std::tuple_element<asInt(_i), std::tuple<std::shared_ptr<NestedStates>...> >::type& state()
  {
    return std::get<asInt(_i)>(this->states_);
  }
};
}

#endif  // STATE_NESTED_SET_H
