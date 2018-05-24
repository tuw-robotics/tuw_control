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

#ifndef STATE_NESTED_VECTOR_H
#define STATE_NESTED_VECTOR_H

#include <float.h>
#include <memory>

#include <tuw_control/state/state.h>
#include <tuw_control/utils.h>

namespace tuw
{
/*!@class StateNestedSet
 * @brief Implementation of @ref State being formed by a vector (variable size) of sub-states.
 * @tparam SubState Type of state defining the sub-states of the object.
 */
template <typename SubState>
class StateNestedVector;
template <typename SubState>
using StateNestedVectorSPtr = std::shared_ptr<StateNestedVector<SubState> >;
template <typename SubState>
using StateNestedVectorConstSPtr = std::shared_ptr<StateNestedVector<SubState> const>;
template <typename SubState>
using StateNestedVectorUPtr = std::unique_ptr<StateNestedVector<SubState> >;
template <typename SubState>
using StateNestedVectorConstUPtr = std::unique_ptr<StateNestedVector<SubState> const>;
template <typename SubState>
class StateNestedVector : public State
{
public:
  StateNestedVector(State* _parent) : State(_parent)
  {
    this->callRootUpdateSize();
  }

public:
  StateNestedVector() : State()
  {
    this->callRootUpdateSize();
  }

public:
  virtual ~StateNestedVector() = default;

public:
  StateNestedVector(const StateNestedVector&) = default;

public:
  StateNestedVector& operator=(const StateNestedVector&) = default;

public:
  StateNestedVector(StateNestedVector&&) = default;

public:
  StateNestedVector& operator=(StateNestedVector&&) = default;

public:
  virtual StateSPtr cloneState() const override
  {
    return std::shared_ptr<StateNestedVector<SubState> >(new StateNestedVector<SubState>(*this));
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
    valueSize_ = 0;
    for (auto& stateI : states_)
    {
      stateI->updateSize();
      valueSize_ += stateI->valueSize();
    }
    values_.resize(valueSize_);
    size_t valueSizeI, valueSizeSum = 0;
    for (auto& stateI : states_)
    {
      valueSizeI = stateI->valueSize();
      for (size_t i = 0; i < valueSizeI; ++i)
      {
        values_[valueSizeSum + i] = &(stateI->value(i));
      }
      valueSizeSum += valueSizeI;
    }
  }

public:
  void resize(const size_t& _i) override
  {
    statesSize_ = _i;
    statesBase_.resize(_i);
    if (_i < states_.size())
    {
      states_.resize(_i);
    }
    else
    {
      for (size_t i = states_.size(); i < _i; ++i)
      {
        states_.emplace_back(std::shared_ptr<SubState>(new SubState(this)));
        statesBase_[i] = states_[i];
      }
    }
    this->callRootUpdateSize();
  }
  ///@brief returns pointer to the extended class of the sub-state at index _i.
public:
  std::shared_ptr<SubState>& stateScoped(const size_t& _i)
  {
    return this->states_[_i];
  }

protected:
  size_t valueSize_;

protected:
  size_t statesSize_;

protected:
  std::vector<std::shared_ptr<SubState> > states_;

protected:
  std::vector<StateSPtr> statesBase_;

protected:
  std::vector<double*> values_;

public:
  const SubState& operator[](size_t i) const
  {
    return *states_[i];
  }

public:
  SubState& operator[](size_t i)
  {
    return *states_[i];
  }

public:
  const std::shared_ptr<SubState>& at(size_t i) const
  {
    return states_[i];
  }

public:
  std::shared_ptr<SubState>& at(size_t i)
  {
    return states_[i];
  }
};
}

#endif  // STATE_NESTED_ARRAY_H
