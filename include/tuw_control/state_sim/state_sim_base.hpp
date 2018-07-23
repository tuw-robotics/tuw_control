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

#ifndef STATE_SIM_BASE_HPP
#define STATE_SIM_BASE_HPP

#include <tuw_control/param_func_new/param_func_dist.hpp>
#include <tuw_control/discretization/discretization_options.hpp>
#include <eigen3/Eigen/Eigen>

namespace tuw
{
namespace /*<anonymous>*/
{
using PfEaG = EvalArcGuarantee;

template <class TDerived>
struct StateMapBaseTraits;
template <class TDerived>
struct StateMapBaseTraits
{
public:
  using StateType = typename StateMapBaseTraits<typename TDerived::StateMapBaseType>::StateType;

public:
  using StateForSimType = typename StateMapBaseTraits<typename TDerived::StateMapBaseType>::StateForSimType;

public:
  using NumType = typename StateMapBaseTraits<typename TDerived::StateMapBaseType>::NumType;

public:
  using StateCfType = typename StateMapBaseTraits<typename TDerived::StateMapBaseType>::StateCfType;

public:
  using StateNmType = typename StateMapBaseTraits<typename TDerived::StateMapBaseType>::StateNmType;

public:
  using StateVirtType = typename StateMapBaseTraits<typename TDerived::StateMapBaseType>::StateVirtType;

public:
  using StateNmNumType = typename StateMapBaseTraits<typename TDerived::StateMapBaseType>::StateNmNumType;

public:
  using StateWithGradNmType = typename StateMapBaseTraits<typename TDerived::StateMapBaseType>::StateWithGradNmType;

public:
  using StateWithGradCfType = typename StateMapBaseTraits<typename TDerived::StateMapBaseType>::StateWithGradCfType;

public:
  using StateWithGradNmNumType =
      typename StateMapBaseTraits<typename TDerived::StateMapBaseType>::StateWithGradNmNumType;
};

template <class TDerived>
struct StateSimBaseCRTPTraits;

class EmptyGradType
{
};

}  // namespace <anonymous>

template <class TDerived>
class StateSimBaseCRTP
{
public:
  using StateType = typename StateSimBaseCRTPTraits<TDerived>::StateType;

public:
  using StateForSimType = typename StateSimBaseCRTPTraits<TDerived>::StateForSimType;

private:
  using NumType = typename StateSimBaseCRTPTraits<TDerived>::NumType;

private:
  using StateNmType = typename StateSimBaseCRTPTraits<TDerived>::StateNmType;

public:
  static constexpr const bool hasStateGrad =
      !std::is_same<EmptyGradType, typename StateSimBaseCRTPTraits<TDerived>::StateWithGradNmType>::value;

public:
  StateSimBaseCRTP() = default;

public:
  ~StateSimBaseCRTP() = default;

public:
  StateSimBaseCRTP(const StateSimBaseCRTP&) = default;

public:
  StateSimBaseCRTP& operator=(const StateSimBaseCRTP&) = default;

public:
  StateSimBaseCRTP(StateSimBaseCRTP&&) = default;

public:
  StateSimBaseCRTP& operator=(StateSimBaseCRTP&&) = default;

public:
  template <bool stateGradientRepresentation = hasStateGrad,
            typename std::enable_if<(stateGradientRepresentation)>::type* = nullptr>
  void advanceWithGrad(const NumType& _arc)
  {
    thisDerived().advanceWithGradImplCRTP(_arc);
  }

public:
  void advance(const NumType& _arc)
  {
    thisDerived().advanceImplCRTP(_arc);
  }

public:
  void simToT(const NumType& _arcEnd, const NumType& _dt)
  {
    thisDerived().simToTImplCRTP(_arcEnd, _dt);
  }

public:
  void toState0()
  {
    thisDerived().toState0ImplCRTP();
  }
  //     public   :       StateForSimType&        state0          () { return thisDerived().state0ImplCRTP        (); }
  //     public   : const StateForSimType&        state0          ()
  //     const { return thisDerived().state0ImplCRTP        (); }
public:
  StateForSimType& state()
  {
    return thisDerived().stateImplCRTP();
  }

public:
  const StateForSimType& state() const
  {
    return thisDerived().stateImplCRTP();
  }

public:
  void advanceSet0(auto& _state0, const NumType& _tEnd, const NumType& _dt)
  {
    thisDerived().advanceSet0ImplCRTP(_state0, _tEnd, _dt);
  }

private:
  TDerived& thisDerived()
  {
    return static_cast<TDerived&>(*this);
  }

private:
  const TDerived& thisDerived() const
  {
    return static_cast<const TDerived&>(*this);
  }
};

template <class TNumType, class StateVirtType>
class StateSimBaseVirt
{
  // special class member functions
public:
  StateSimBaseVirt() = default;

public:
  ~StateSimBaseVirt() = default;

public:
  StateSimBaseVirt(const StateSimBaseVirt&) = default;

public:
  StateSimBaseVirt& operator=(const StateSimBaseVirt&) = default;

public:
  StateSimBaseVirt(StateSimBaseVirt&&) = default;

public:
  StateSimBaseVirt& operator=(StateSimBaseVirt&&) = default;

public:
  void advance(const TNumType& _arc)
  {
    advanceImplVirt(_arc);
  }

public:
  void advanceWithGrad(const TNumType& _arc)
  {
    advanceWithGradImplVirt(_arc);
  }

public:
  void toState0()
  {
    toState0ImplVirt();
  }

public:
  void toState()
  {
    toState0ImplVirt();
  }

  // pure virtual functions
private:
  virtual void advanceImplVirt(const TNumType& _arc) = 0;

private:
  virtual void advanceWithGradImplVirt(const TNumType& _arc) = 0;

private:
  virtual void toState0ImplVirt() = 0;
};

template <class NumType, class StateWithGradNmNumType, template <class> class TDiscretizationType>
class OdeStateSolverRealAlias
{
  //     private  : using OdeStateGradSolverRKType   = explicit_generic_rk_impl<  TDiscretizationType,
  // 	                                                                 StateWithGradNmNumType,
  // 								         NumType,
  // 								         StateWithGradNmNumType,
  // 								         NumType,
  // 								         odeint::vector_space_algebra >;
  //     private  : using OdeStateGradSolverType = explicit_adams_bashforth<5, OdeStateGradSolverRKType>;
private:
  using OdeStateGradSolverType =
      explicit_generic_rk_impl<TDiscretizationType, StateWithGradNmNumType, NumType, StateWithGradNmNumType, NumType,
                               odeint::vector_space_algebra>;

private:
  OdeStateGradSolverType rkGrad_;

  template <class TDerived2, class TParamType2, class TStateType2, template <class> class TDiscretizationType2,
            class... TFuncsType2>
  friend class StateSimBase;
};

class OdeStateSolverDummyAlias
{
  using OdeStateGradSolverType = EmptyGradType;

  template <class TDerived2, class TParamType2, class TStateType2, template <class> class TDiscretizationType2,
            class... TFuncsType2>
  friend class StateSimBase;
};

template <class TDerived, class TParamType, class TStateType, template <class> class TDiscretizationType,
          class... TFuncsType>
class StateSimBase
    : public StateSimBaseCRTP<StateSimBase<TDerived, TParamType, TStateType, TDiscretizationType, TFuncsType...>>,
      public StateSimBaseVirt<typename StateMapBaseTraits<typename TStateType::StateMapBaseType>::NumType,
                              typename StateMapBaseTraits<typename TStateType::StateMapBaseType>::StateVirtType>,
      public std::conditional<
          !std::is_same<EmptyGradType,
                        typename StateMapBaseTraits<typename TStateType::StateMapBaseType>::StateWithGradNmType>::value,
          OdeStateSolverRealAlias<
              typename StateMapBaseTraits<typename TStateType::StateMapBaseType>::NumType,
              typename StateMapBaseTraits<typename TStateType::StateMapBaseType>::StateWithGradNmNumType,
              TDiscretizationType>,
          OdeStateSolverDummyAlias>::type
{
public:
  using StateType = TStateType;

public:
  using StateForSimType = typename StateMapBaseTraits<typename TStateType::StateMapBaseType>::StateForSimType;

public:
  using ParamStructType = TParamType;

private:
  using StateSimBaseType = StateSimBase<TDerived, TParamType, TStateType, TDiscretizationType, TFuncsType...>;

private:
  using NumType = typename StateMapBaseTraits<typename TStateType::StateMapBaseType>::NumType;

private:
  using StateVirtType = typename StateMapBaseTraits<typename TStateType::StateMapBaseType>::StateVirtType;

private:
  using StateNmType = typename StateMapBaseTraits<typename TStateType::StateMapBaseType>::StateNmType;

private:
  using StateCfType = typename StateMapBaseTraits<typename TStateType::StateMapBaseType>::StateCfType;

private:
  using StateNmNumType = typename StateMapBaseTraits<typename TStateType::StateMapBaseType>::StateNmNumType;

private:
  using StateWithGradNmType = typename StateMapBaseTraits<typename TStateType::StateMapBaseType>::StateWithGradNmType;

private:
  using StateWithGradCfType = typename StateMapBaseTraits<typename TStateType::StateMapBaseType>::StateWithGradCfType;

private:
  using StateWithGradNmNumType =
      typename StateMapBaseTraits<typename TStateType::StateMapBaseType>::StateWithGradNmNumType;

private:
  using OdeStateSolverType /*OdeStateSolverRKType*/ =
      explicit_generic_rk_impl<TDiscretizationType, StateNmNumType, NumType, StateNmNumType, NumType,
                               odeint::vector_space_algebra>;
  //     private  : using OdeStateSolverType     = explicit_adams_bashforth<5, OdeStateSolverRKType>;
public:
  static constexpr const bool hasStateGrad = !std::is_same<EmptyGradType, StateWithGradNmType>::value;

public:
  using StateNumSimType = typename std::conditional<hasStateGrad, StateWithGradNmType, StateNmType>::type;

  // special class member functions
public:
  StateSimBase() = default;

public:
  ~StateSimBase() = default;

public:
  StateSimBase(const StateSimBase&) = default;

public:
  StateSimBase& operator=(const StateSimBase&) = default;

public:
  StateSimBase(StateSimBase&&) = delete;

public:
  StateSimBase& operator=(StateSimBase&&) = delete;

public:
  using StateSimBaseCRTP<StateSimBaseType>::advance;

public:
  using StateSimBaseCRTP<StateSimBaseType>::advanceWithGrad;

public:
  using StateSimBaseCRTP<StateSimBaseType>::toState0;

public:
  using StateSimBaseCRTP<StateSimBaseType>::state;

public:
  using StateSimBaseCRTP<StateSimBaseType>::advanceSet0;

private:
  void advanceImplVirt(const NumType& _arc) override final
  {
    advanceImplCRTP(_arc);
  }

private:
  void advanceWithGradImplVirt(const NumType& _arc) override final
  {
    advanceWithGradImplVirtDispatch(_arc);
  }

private:
  void toState0ImplVirt() override final
  {
    toState0ImplCRTP();
  }

private:
  template <bool stateGradientRepresentation = hasStateGrad,
            typename std::enable_if<(stateGradientRepresentation)>::type* = nullptr>
  void advanceWithGradImplVirtDispatch(const NumType& _arc)
  {
    advanceWithGradImplCRTP(_arc);
  }

private:
  template <bool stateGradientRepresentation = hasStateGrad,
            typename std::enable_if<(!stateGradientRepresentation)>::type* = nullptr>
  void advanceWithGradImplVirtDispatch(const NumType& _arc)
  {
    throw std::runtime_error("Cannot advance with gradient info (State class not suited)");
  }

private:
  StateForSimType& stateImplCRTP()
  {
    return state_;
  }

private:
  const StateForSimType& stateImplCRTP() const
  {
    return state_;
  }

private:
  void advanceImplCRTP(const NumType& _arc)
  {
    rk_.do_step(
        [this](const StateNmNumType& _x, StateNmNumType& _dxdt, const NumType _t)
        {
          static NumType* memStartRefNm;
          memStartRefNm = state_.stateNm().memStartRef();
          state_.stateNm().bindToMemory((NumType*)_x.data());
          setXNmDot(_t, PfEaG::NEAR_LAST);

          _dxdt = stateWithGradNmDotCache_.state().data();
          state_.stateNm().bindToMemory(memStartRefNm);
        },
        state_.stateNm().data(), arcOld_, _arc - arcOld_);
    arcOld_ = _arc;
  }

private:
  template <bool stateGradientRepresentation = hasStateGrad,
            typename std::enable_if<(stateGradientRepresentation)>::type* = nullptr>
  void advanceWithGradImplCRTP(const NumType& _arc)
  {
    this->rkGrad_.do_step(
        [this](const StateWithGradNmNumType& _x, StateWithGradNmNumType& _dxdt, const NumType _t)
        {
          static NumType* memStartRefNm;
          memStartRefNm = state_.stateWithGradNm().memStartRef();
          state_.stateWithGradNm().bindToMemory((NumType*)_x.data());
          setXNmDot(_t, PfEaG::NEAR_LAST);
          setGradXNmDot(_t, PfEaG::NEAR_LAST);

          _dxdt = stateWithGradNmDotCache_.data();
          state_.stateWithGradNm().bindToMemory(memStartRefNm);
        },
        state_.stateWithGradNm().data(), arcOld_, _arc - arcOld_);
    arcOld_ = _arc;
  }

private:
  template <bool stateGradientRepresentation = hasStateGrad,
            typename std::enable_if<(!stateGradientRepresentation)>::type* = nullptr>
  void toState0ImplCRTP()
  {
    adjustXSize();

    for_each_tuple(funcs_, [this](auto& funcI)
                   {
                     funcI.precompute(*this);
                   });

    setXNm0();
    stateWithGradNmDotCache_ = state_.stateNm();

    setXCf(0, PfEaG::AT_BEGIN);
    setXNmDot(0, PfEaG::AT_BEGIN);
    arcOld_ = 0;

    rk_.adjust_size(state_.stateNm().data().size());
    // 		    rk_.reset();
  }

private:
  template <bool stateGradientRepresentation = hasStateGrad,
            typename std::enable_if<(stateGradientRepresentation)>::type* = nullptr>
  void toState0ImplCRTP()
  {
    adjustXSize();
    adjustGradXSize();

    for_each_tuple(funcs_, [this](auto& funcI)
                   {
                     funcI.precompute(*this);
                   });

    setXNm0();
    setGradXNm0();
    stateWithGradNmDotCache_ = state_.stateWithGradNm();

    setXCf(0, PfEaG::AT_BEGIN);
    setGradXCf(0, PfEaG::AT_BEGIN);
    setXNmDot(0, PfEaG::AT_BEGIN);
    setGradXNmDot(0, PfEaG::AT_BEGIN);
    arcOld_ = 0;

    rk_.adjust_size(state_.stateNm().data().size());
    this->rkGrad_.adjust_size(state_.stateWithGradNm().data().size());
    // 		    rk_.reset();
    // 		    this->rkGrad_.reset();
  }

private:
  void simToTImplCRTP(const NumType& _tEnd, const NumType& _dt)
  {
    toState0();
    NumType tSim = _dt;
    while (tSim < _tEnd)
    {
      advance(tSim);
      tSim += _dt;
    }
    advance(_tEnd);
  }

private:
  void advanceSet0ImplCRTP(auto& _state0, const NumType& _tEnd, const NumType& _dt)
  {
    simToTImplCRTP(_tEnd, _dt);
    _state0.stateNm().data() = state_.stateNm().data();
    _state0.stateCf().data() = state_.stateCf().data();
  }

  // no guarantee on when it is called; for caching, parts of XCf might have been set by setXNmDot
public:
  void setXCf(const NumType& _arc, const PfEaG& _eAG)
  {
    thisSimDerived().setXCfImpl(state_.stateCf(), _arc, _eAG);
  }
  // guaranteed to be called after setXCf
public:
  void setXCfDot(const NumType& _arc, const PfEaG& _eAG)
  {
    thisSimDerived().setXCfDotImpl(stateCfDotCache_, state_.stateCf(), _arc, _eAG);
  }
  // no guarantee on when it is called; for caching, parts of XCf can be precalculated
public:
  void setXNmDot(const NumType& _arc, const PfEaG& _eAG)
  {
    thisSimDerived().setXNmDotImpl(stateWithGradNmDotCache_.state(), state_.stateCf(), state_.stateNm(), _arc, _eAG);
    for_each_tuple(funcs_, [this, &_arc](auto& funcI)
                   {
                     funcI.computeFuncDot(stateWithGradNmDotCache_.state(), state_.stateNm(), state_.stateCf(), *this,
                                          _arc, PfEaG::NEAR_LAST);
                   });
  }
  // guaranteed to be called after setXCf; for caching, parts of gradXCf might have been set by setGradXNmDot
public:
  template <bool stateGradientRepresentation = hasStateGrad,
            typename std::enable_if<(stateGradientRepresentation)>::type* = nullptr>
  void setGradXCf(const NumType& _arc, const PfEaG& _eAG)
  {
    thisSimDerived().setGradXCfImpl(state_.stateGradCf(), state_.stateCf(), _arc, _eAG);
  }
  // no guarantee on when it is called; for caching, parts of GradXCf can be precalculated
public:
  template <bool stateGradientRepresentation = hasStateGrad,
            typename std::enable_if<(stateGradientRepresentation)>::type* = nullptr>
  void setGradXNmDot(const NumType& _arc, const PfEaG& _eAG)
  {
    thisSimDerived().setGradXNmDotImpl(stateWithGradNmDotCache_.stateGrad(), state_.stateWithGradCf(),
                                       state_.stateWithGradNm(), _arc, _eAG);
    for_each_tuple(funcs_, [this, &_arc](auto& funcI)
                   {
                     funcI.computeGradFuncDot(stateWithGradNmDotCache_.stateGrad(), state_.stateWithGradNm(),
                                              state_.stateWithGradCf(), *this, _arc, PfEaG::NEAR_LAST);
                   });
  }

public:
  void init(std::shared_ptr<TParamType> _paramStructPtr)
  {
    paramStruct = _paramStructPtr;
  }

private:
  TDerived& thisSimDerived()
  {
    return static_cast<TDerived&>(*this);
  }

private:
  const TDerived& thisSimDerived() const
  {
    return static_cast<const TDerived&>(*this);
  }

public:
  const StateNmType& stateNmDotCache() const
  {
    return stateWithGradNmDotCache_.state();
  }

public:
  const StateCfType& stateCfDotCache() const
  {
    return stateCfDotCache_;
  }

public:
  NumType arc() const
  {
    return arcOld_;
  }

public:
  void setXNm0()
  {
    thisSimDerived().setXNm0Impl(state_.stateNm());
  }

public:
  void setGradXNm0()
  {
    thisSimDerived().setGradXNm0Impl(state_.stateGradNm(), state_.stateNm());
  }

public:
  void adjustXSize()
  {
    thisSimDerived().adjustXSizeImpl(state_.stateNm(), state_.stateCf());
  }

public:
  void adjustGradXSize()
  {
    thisSimDerived().adjustGradXSizeImpl(state_.stateGradNm(), state_.stateGradCf());
  }

private:
  StateCfType stateCfDotCache_;

private:
  StateNumSimType stateWithGradNmDotCache_;

private:
  NumType arcOld_;

private:
  OdeStateSolverType rk_;

private:
  StateForSimType state_;

public:
  std::shared_ptr<TParamType> paramStruct;

private:
  std::tuple<TFuncsType...> funcs_;

  template <class TNumType2, class StateVirtType2>
  friend class StateSimBaseVirt;
  template <class TDerived2>
  friend class StateSimBaseCRTP;
};

namespace /*<anonymous>*/
{
template <class TDerived, class TParamType, class TStateType, template <class> class TDiscretizationType,
          class... TFuncsType>
struct StateSimBaseCRTPTraits<StateSimBase<TDerived, TParamType, TStateType, TDiscretizationType, TFuncsType...>>
{
public:
  using StateType = TStateType;

public:
  using StateForSimType = typename StateMapBaseTraits<typename TStateType::StateMapBaseType>::StateForSimType;

public:
  using NumType = typename StateMapBaseTraits<typename TStateType::StateMapBaseType>::NumType;

public:
  using StateCfType = typename StateMapBaseTraits<typename TStateType::StateMapBaseType>::StateCfType;

public:
  using StateNmType = typename StateMapBaseTraits<typename TStateType::StateMapBaseType>::StateNmType;

public:
  using StateVirtType = typename StateMapBaseTraits<typename TStateType::StateMapBaseType>::StateVirtType;

public:
  using StateNmNumType = typename StateMapBaseTraits<typename TStateType::StateMapBaseType>::StateNmNumType;

public:
  using StateWithGradNmType = typename StateMapBaseTraits<typename TStateType::StateMapBaseType>::StateWithGradNmType;

public:
  using StateWithGradCfType = typename StateMapBaseTraits<typename TStateType::StateMapBaseType>::StateWithGradCfType;

public:
  using StateWithGradNmNumType =
      typename StateMapBaseTraits<typename TStateType::StateMapBaseType>::StateWithGradNmNumType;
};

}  // namespace <anonymous>
}

#endif  // STATE_SIM_BASE_HPP
