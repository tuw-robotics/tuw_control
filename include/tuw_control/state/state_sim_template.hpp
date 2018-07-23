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

#ifndef STATE_SIM_TEMPLATE_H
#define STATE_SIM_TEMPLATE_H

#include <tuw_control/state/state_sim.h>
#include <tuw_control/discretization/discretization_runge_kutta.hpp>
#include <boost/numeric/odeint.hpp>

namespace tuw
{
/*! @class StateSimTemplate
 *  @brief Templated partial implementation of @ref StateSim.
 *  @tparam StateSize Size of the full state.
 *  @tparam StateNmSize Size of the numerically-evaluated state.
 */
template <std::size_t StateSize, std::size_t StateNmSize>
class StateSimTemplate;
template <std::size_t StateSize, std::size_t StateNmSize>
using StateSimTemplateSPtr = std::shared_ptr<StateSimTemplate<StateSize, StateNmSize>>;
template <std::size_t StateSize, std::size_t StateNmSize>
using StateSimTemplateConstSPtr = std::shared_ptr<StateSimTemplate<StateSize, StateNmSize> const>;
template <std::size_t StateSize, std::size_t StateNmSize>
using StateSimTemplateUPtr = std::unique_ptr<StateSimTemplate<StateSize, StateNmSize>>;
template <std::size_t StateSize, std::size_t StateNmSize>
using StateSimTemplateConstUPtr = std::unique_ptr<StateSimTemplate<StateSize, StateNmSize> const>;
template <std::size_t StateSize, std::size_t StateNmSize>
class StateSimTemplate : public StateSim
{
  // special class member functions
public:
  StateSimTemplate() = default;

public:
  virtual ~StateSimTemplate() = default;

public:
  StateSimTemplate(const StateSimTemplate& _o) : discrFunc_(_o.discrFunc_)
  {
    for (size_t i = 0; i < valueSize(); ++i)
    {
      value(i) = _o.value(i);
      state0_.value(i) = _o.state0_.value(i);
    }
  }  //= default;
public:
  StateSimTemplate& operator=(const StateSimTemplate& _o)
  {
    discrFunc_ = _o.discrFunc_;
    for (size_t i = 0; i < valueSize(); ++i)
    {
      value(i) = _o.value(i);
      state0_.value(i) = _o.state0_.value(i);
    }
    return *this;
  }  //= default;
public:
  StateSimTemplate(StateSimTemplate&&) = delete;  // default;
public:
  StateSimTemplate& operator=(StateSimTemplate&&) = delete;  // default;

  // implemented virtual functions
public:
  virtual StateSPtr cloneState() const override
  {
    StateSPtr retState = std::make_shared<StateArray<StateSize>>();
    const std::size_t sNmS = stateNm_.valueSize();
    for (std::size_t i = 0; i < sNmS; i++)
    {
      retState->value(i) = stateNm_.value(i);
    }
    for (std::size_t i = 0; i < stateCf_.valueSize(); i++)
    {
      retState->value(i + sNmS) = stateCf_.value(i);
    }
    return retState;
  }

public:
  void toState0() override
  {
    const std::size_t sNmS = stateNm_.valueSize();
    for (std::size_t i = 0; i < sNmS; i++)
    {
      stateNm_.value(i) = state0_.value(i);
    }
    // 	for( std::size_t i = 0; i < stateCf_.valueSize(); i++ ){ stateCf_.value(i) = state0_.value(i+sNmS); }
    setStateCf(0, ParamFuncs::EvalArcGuarantee::AT_BEGIN);
    rk = std::make_shared<boost::numeric::odeint::runge_kutta4<std::array<double, StateNmSize>>>();
    arcOld = 0;
  }

public:
  void setDiscrType(const RungeKutta::DiscretizationType& _discrType) override
  {
    discrFunc_ = RungeKutta::getDiscrFunc<StateNmSize>(_discrType);
  }

public:
  void setState(StateSPtr& _otherState) override
  {
    setStateCf(0, ParamFuncs::EvalArcGuarantee::AT_BEGIN);
    const std::size_t sNmS = stateNm_.valueSize();
    for (std::size_t i = 0; i < sNmS; i++)
    {
      stateNm_.value(i) = _otherState->value(i);
    }
    for (std::size_t i = 0; i < stateCf_.valueSize(); i++)
    {
      stateCf_.value(i) = _otherState->value(i + sNmS);
    }
  }

public:
  State& state0() override
  {
    return state0_;
  }

public:
  State& stateNm() override
  {
    return stateNm_;
  }

public:
  State& stateCf() override
  {
    return stateCf_;
  }

public:
  size_t valueSize() const override
  {
    return StateSize;
  }

public:
  double& value(const std::size_t& _i) override
  {
    if (_i < StateNmSize)
    {
      return stateNm_.value(_i);
    }
    else
    {
      return stateCf_.value(_i - StateNmSize);
    }
  };

public:
  const double& value(const std::size_t& _i) const override
  {
    if (_i < StateNmSize)
    {
      return stateNm_.value(_i);
    }
    else
    {
      return stateCf_.value(_i - StateNmSize);
    }
  };

public:
  void advance(double _arc) override
  {
    discrFunc_(*this, _arc);
  }

public:
  void advanceODEInt(double _arc)
  {
    // 	setStateCf (arcOld, ParamFuncs::EvalArcGuarantee::NONE);
    rk->do_step(
        [this](const std::array<double, StateNmSize>& _x, std::array<double, StateNmSize>& _dxdt, const double _t)
        {
          setStateCfNmStep(_t, ParamFuncs::EvalArcGuarantee::AFTER_LAST);
          stateNm_.valuesArray() = _x;
          stateNmDot();
          _dxdt = stateNmDotCache_.valuesArray();
        },
        stateNm_.valuesArray(), arcOld, _arc - arcOld);
    arcOld = _arc;
    setStateCf(_arc, ParamFuncs::EvalArcGuarantee::AFTER_LAST);
  }

public:
  void sys1(const std::array<double, StateNmSize>& _x, std::array<double, StateNmSize>& _dxdt, const double _t)
  {
    setStateCfNmStep(_t, ParamFuncs::EvalArcGuarantee::AFTER_LAST);
    stateNmDot();
    _dxdt = stateNm_.valuesArray();
  }

  double arcOld;

protected:
  std::shared_ptr<boost::numeric::odeint::runge_kutta4<std::array<double, StateNmSize>>> rk;

  ///@brief State array storing the initial value.
protected:
  StateArray<StateSize> state0_;
  ///@brief State array storing the numerical-computed value.
protected:
  StateArray<StateNmSize> stateNm_;
  ///@brief State array caching the evaluation of the last call of the value transition function.
protected:
  StateArray<StateNmSize> stateNmDotCache_;
  ///@brief State array storing the closed-form-computed value.
protected:
  StateArray<StateSize - StateNmSize> stateCf_;
  ///@brief Pointer to the active discretization-method function.
private:
  RungeKutta::DiscretizationFuncPtr discrFunc_;

private:
  using StateSim::setStateCf;
};
}

#endif  // STATE_SIM_TEMPLATE_H