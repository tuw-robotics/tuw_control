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

#ifndef STATE_SIM_HPP
#define STATE_SIM_HPP

#include <tuw_control/state_map/state_map.hpp>
#include <tuw_control/state_sim/state_sim_base.hpp>

namespace tuw
{
template <class TNum, class TStateNm, class TStateCf>
class StateMapBase : public StateMapTuple<TNum, TStateNm, TStateCf>
{
public:
  using StateMapSimBaseType = StateMapBase<TNum, TStateNm, TStateCf>;

public:
  using StateMapBaseType = StateMapBase<TNum, TStateNm, TStateCf>;

public:
  using StateMapTuple<TNum, TStateNm, TStateCf>::StateMapTuple;

public:
  auto& stateNm()
  {
    return this->template sub<0>();
  }

public:
  const auto& stateNm() const
  {
    return this->template sub<0>();
  }

public:
  auto& stateCf()
  {
    return this->template sub<1>();
  }

public:
  const auto& stateCf() const
  {
    return this->template sub<1>();
  }

public:
  auto& state()
  {
    return *this;
  }

public:
  const auto& state() const
  {
    return *this;
  }
};

template <class TNum, template <class, class> class TStateNm, template <class, class> class TStateCf>
using StateMap = StateMapBase<TNum, TStateNm<TNum, TNum>, TStateCf<TNum, TNum>>;

template <class TNum, class TStateNmByOptVar, class TStateCfByOptVar>
class StateGradMapBase : public StateMapBase<TNum, TStateNmByOptVar, TStateCfByOptVar>
{
public:
  using StateMapSimBaseType = StateMapBase<TNum, TStateNmByOptVar, TStateCfByOptVar>;

public:
  using StateMapBaseType = StateGradMapBase<TNum, TStateNmByOptVar, TStateCfByOptVar>;

public:
  StateGradMapBase()
    : StateMapBase<TNum, TStateNmByOptVar, TStateCfByOptVar>::StateMapBase(), gradMatMap_(nullptr, 0, 0)
  {
  }

public:
  StateGradMapBase(auto&& _a, auto&& _b)
    : StateMapBase<TNum, TStateNmByOptVar, TStateCfByOptVar>::StateMapBase(_a, _b), gradMatMap_(nullptr, 0, 0)
  {
  }

public:
  auto& mat()
  {
    return gradMatMap_;
  }

public:
  const auto& mat() const
  {
    return gradMatMap_;
  }

public:
  void bindMat()
  {
    new (&gradMatMap_) Eigen::Map<Eigen::Matrix<TNum, -1, -1, Eigen::RowMajor>, MapAlignment>(
        this->memStartRef(), this->stateNm().varSize() + this->stateCf().varSize(),
        this->stateNm().var(0).data().size());
    this->stateNm().bindMat();
    this->stateCf().bindMat();
  }

private:
  Eigen::Map<Eigen::Matrix<TNum, -1, -1, Eigen::RowMajor>, MapAlignment> gradMatMap_;
};

template <class TNum, class TstateGrad>
class StateGradWithMatMap : public TstateGrad
{
public:
  StateGradWithMatMap() : TstateGrad(), gradMatMap_(nullptr, 0, 0)
  {
  }

public:
  StateGradWithMatMap(auto&& _a, auto&& _b) : TstateGrad(_a, _b), gradMatMap_(nullptr, 0, 0)
  {
  }

public:
  StateGradWithMatMap(const StateGradWithMatMap& _rhs) : TstateGrad(_rhs), gradMatMap_(nullptr, 0, 0)
  {
  }

public:
  StateGradWithMatMap& operator=(const StateGradWithMatMap& _rhs)
  {
    TstateGrad::operator=(_rhs);
    return *this;
  }

public:
  StateGradWithMatMap(StateGradWithMatMap&&) = default;

public:
  StateGradWithMatMap& operator=(StateGradWithMatMap&&) = default;

public:
  auto& mat()
  {
    return gradMatMap_;
  }

public:
  const auto& mat() const
  {
    return gradMatMap_;
  }

public:
  void bindMat()
  {
    new (&gradMatMap_) Eigen::Map<Eigen::Matrix<TNum, -1, -1, Eigen::RowMajor>, MapAlignment>(
        this->memStartRef(), this->varSize(), this->var(0).data().size());
  }

private:
  Eigen::Map<Eigen::Matrix<TNum, -1, -1, Eigen::RowMajor>, MapAlignment> gradMatMap_;
};

namespace /*<anonymous>*/
{
template <class TNum, class TStateNm, class TStateCf>
struct StateMapBaseTraits<StateMapBase<TNum, TStateNm, TStateCf>>
{
public:
  using StateType = StateMapBase<TNum, TStateNm, TStateCf>;

public:
  using StateForSimType = StateMapBase<TNum, TStateNm, TStateCf>;

public:
  using NumType = TNum;

public:
  using StateCfType = TStateCf;

public:
  using StateNmType = TStateNm;

public:
  using StateVirtType = StateMapBaseVirt<TNum>;

public:
  using StateNmNumType = typename TStateNm::StateMapBaseCRTP::MatrixTypeCRTP;

public:
  using StateWithGradNmType = EmptyGradType;

public:
  using StateWithGradCfType = EmptyGradType;

public:
  using StateWithGradNmNumType = EmptyGradType;
};
template <class TNum, class TStateNm, class TStateCf>
struct StateMapBaseTraits<StateGradMapBase<TNum, TStateNm, TStateCf>>
{
public:
  using StateType = StateGradMapBase<TNum, TStateNm, TStateCf>;

public:
  using StateForSimType = StateMapBase<TNum, TStateNm, TStateCf>;

public:
  using NumType = TNum;

public:
  using StateCfType = TStateCf;

public:
  using StateNmType = TStateNm;

public:
  using StateVirtType = StateMapBaseVirt<TNum>;

public:
  using StateNmNumType = typename TStateNm::StateMapBaseCRTP::MatrixTypeCRTP;

public:
  using StateWithGradNmType = EmptyGradType;

public:
  using StateWithGradCfType = EmptyGradType;

public:
  using StateWithGradNmNumType = EmptyGradType;
};
template <class TNum, class TstateGrad>
struct StateMapBaseTraits<StateGradWithMatMap<TNum, TstateGrad>>
{
public:
  using StateType = typename StateMapBaseTraits<TstateGrad>::StateType;

public:
  using StateForSimType = typename StateMapBaseTraits<TstateGrad>::StateForSimType;

public:
  using NumType = typename StateMapBaseTraits<TstateGrad>::NumType;

public:
  using StateCfType = typename StateMapBaseTraits<TstateGrad>::StateCfType;

public:
  using StateNmType = typename StateMapBaseTraits<TstateGrad>::StateNmType;

public:
  using StateVirtType = typename StateMapBaseTraits<TstateGrad>::StateVirtType;

public:
  using StateNmNumType = typename StateMapBaseTraits<TstateGrad>::StateNmNumType;

public:
  using StateWithGradNmType = typename StateMapBaseTraits<TstateGrad>::StateWithGradNmType;

public:
  using StateWithGradCfType = typename StateMapBaseTraits<TstateGrad>::StateWithGradCfType;

public:
  using StateWithGradNmNumType = typename StateMapBaseTraits<TstateGrad>::StateWithGradNmNumType;
};
// template<class TNum, template<class> class TStateNm, template<class> class TStateCf>
// struct StateMapBaseTraits< StateMapBase<TNum, TStateNm, TStateCf> > {
//     public   : using StateType              = StateMapBase<TNum, TStateNm<TNum>, TStateCf<TNum>>;
//     public   : using StateForSimType        = StateMapBase<TNum, TStateNm<TNum>, TStateCf<TNum>>;
//     public   : using NumType                = TNum;
//     public   : using StateCfType            = TStateCf<TNum>;
//     public   : using StateNmType            = TStateNm;
//     public   : using StateVirtType          = StateMapBaseVirt<TNum>;
//     public   : using StateNmNumType         = typename TStateNm::StateMapBaseCRTP::MatrixTypeCRTP;
//     public   : using StateWithGradNmType    = EmptyGradType;
//     public   : using StateWithGradCfType    = EmptyGradType;
//     public   : using StateWithGradNmNumType = EmptyGradType;
// };

}  // namespace <anonymous>

namespace /*<anonymous>*/
{
template <class TStateWithGradMapType>
using StateMapTraits = StateMapBaseTraits<typename TStateWithGradMapType::StateMapBaseType>;

}  // namespace <anonymous>

template <class TNum, class TState, class TStateGrad>
class StateWithGradMapSimBaseInternal : public StateMapTuple<TNum, TState, TStateGrad>
{
public:
  using StateMapTuple<TNum, TState, TStateGrad>::StateMapTuple;

public:
  auto& state()
  {
    return this->template sub<0>();
  }

public:
  const auto& state() const
  {
    return this->template sub<0>();
  }

public:
  auto& stateGrad()
  {
    return this->template sub<1>();
  }

public:
  const auto& stateGrad() const
  {
    return this->template sub<1>();
  }
};

template <class TStateWithGradMapType>
class StateWithGradMapSimBase
    : public StateMapTuple<
          typename StateMapTraits<TStateWithGradMapType>::NumType,
          StateWithGradMapSimBaseInternal<typename StateMapTraits<TStateWithGradMapType>::NumType,
                                          typename StateMapTraits<TStateWithGradMapType>::StateNmType,
                                          typename StateMapTraits<TStateWithGradMapType>::StateNmGradType>,
          StateWithGradMapSimBaseInternal<typename StateMapTraits<TStateWithGradMapType>::NumType,
                                          typename StateMapTraits<TStateWithGradMapType>::StateCfType,
                                          typename StateMapTraits<TStateWithGradMapType>::StateCfGradType>>
{
public:
  using StateMapBaseType = typename StateMapTraits<TStateWithGradMapType>::StateType;

private:
  using NumType = typename StateMapTraits<TStateWithGradMapType>::NumType;

private:
  using BaseClassType =
      StateMapTuple<typename StateMapTraits<TStateWithGradMapType>::NumType,
                    StateWithGradMapSimBaseInternal<typename StateMapTraits<TStateWithGradMapType>::NumType,
                                                    typename StateMapTraits<TStateWithGradMapType>::StateNmType,
                                                    typename StateMapTraits<TStateWithGradMapType>::StateNmGradType>,
                    StateWithGradMapSimBaseInternal<typename StateMapTraits<TStateWithGradMapType>::NumType,
                                                    typename StateMapTraits<TStateWithGradMapType>::StateCfType,
                                                    typename StateMapTraits<TStateWithGradMapType>::StateCfGradType>>;

public:
  StateWithGradMapSimBase() : BaseClassType::StateMapTuple()
  {
  }

public:
  StateWithGradMapSimBase(const StateWithGradMapSimBase& _rhs) : BaseClassType(_rhs)
  {
  }

public:
  StateWithGradMapSimBase& operator=(const StateWithGradMapSimBase& _rhs)
  {
    BaseClassType::operator=(_rhs);
    return *this;
  }

public:
  StateWithGradMapSimBase(StateWithGradMapSimBase&&) = default;

public:
  StateWithGradMapSimBase& operator=(StateWithGradMapSimBase&&) = default;

public:
  auto& stateWithGradNm()
  {
    return this->template sub<0>();
  }

public:
  const auto& stateWithGradNm() const
  {
    return this->template sub<0>();
  }

public:
  auto& stateNm()
  {
    return this->stateWithGradNm().template sub<0>();
  }

public:
  const auto& stateNm() const
  {
    return this->stateWithGradNm().template sub<0>();
  }

public:
  auto& stateGradNm()
  {
    return this->stateWithGradNm().template sub<1>();
  }

public:
  const auto& stateGradNm() const
  {
    return this->stateWithGradNm().template sub<1>();
  }

public:
  auto& stateWithGradCf()
  {
    return this->template sub<1>();
  }

public:
  const auto& stateWithGradCf() const
  {
    return this->template sub<1>();
  }

public:
  auto& stateCf()
  {
    return this->stateWithGradCf().template sub<0>();
  }

public:
  const auto& stateCf() const
  {
    return this->stateWithGradCf().template sub<0>();
  }

public:
  auto& stateGradCf()
  {
    return this->stateWithGradCf().template sub<1>();
  }

public:
  const auto& stateGradCf() const
  {
    return this->stateWithGradCf().template sub<1>();
  }
};

template <class TNum, template <class, class> class TStateNm, template <class, class> class TStateCf,
          template <class, class> class TOptVarStruct>
class StateWithGradMapBase
    : public StateMapTuple<TNum, StateMapBase<TNum, TStateNm<TNum, TNum>, TStateCf<TNum, TNum>>,
                           StateGradMapBase<TNum, StateGradWithMatMap<TNum, TStateNm<TNum, TOptVarStruct<TNum, TNum>>>,
                                            StateGradWithMatMap<TNum, TStateCf<TNum, TOptVarStruct<TNum, TNum>>>>>
{
public:
  using StateMapBaseType = StateWithGradMapBase<TNum, TStateNm, TStateCf, TOptVarStruct>;

private:
  using BaseClassType =
      StateMapTuple<TNum, StateMapBase<TNum, TStateNm<TNum, TNum>, TStateCf<TNum, TNum>>,
                    StateGradMapBase<TNum, StateGradWithMatMap<TNum, TStateNm<TNum, TOptVarStruct<TNum, TNum>>>,
                                     StateGradWithMatMap<TNum, TStateCf<TNum, TOptVarStruct<TNum, TNum>>>>>;

public:
  StateWithGradMapBase() : BaseClassType::StateMapTuple()
  {
  }

public:
  StateWithGradMapBase(const StateWithGradMapBase& _rhs) : BaseClassType(_rhs)
  {
  }

public:
  StateWithGradMapBase& operator=(const StateWithGradMapBase& _rhs)
  {
    BaseClassType::operator=(_rhs);
    return *this;
  }

public:
  StateWithGradMapBase(StateWithGradMapBase&&) = default;

public:
  StateWithGradMapBase& operator=(StateWithGradMapBase&&) = default;

public:
  auto& stateNm()
  {
    return this->state().template sub<0>();
  }

public:
  const auto& stateNm() const
  {
    return this->state().template sub<0>();
  }

public:
  auto& stateCf()
  {
    return this->state().template sub<1>();
  }

public:
  const auto& stateCf() const
  {
    return this->state().template sub<1>();
  }

public:
  auto& stateGradNm()
  {
    return this->stateGrad().template sub<0>();
  }

public:
  const auto& stateGradNm() const
  {
    return this->stateGrad().template sub<0>();
  }

public:
  auto& stateGradCf()
  {
    return this->stateGrad().template sub<1>();
  }

public:
  const auto& stateGradCf() const
  {
    return this->stateGrad().template sub<1>();
  }

public:
  auto& state()
  {
    return this->template sub<0>();
  }

public:
  const auto& state() const
  {
    return this->template sub<0>();
  }

public:
  auto& stateGrad()
  {
    return this->template sub<1>();
  }

public:
  const auto& stateGrad() const
  {
    return this->template sub<1>();
  }
};

template <class TNum, template <class, class> class TStateNm, template <class, class> class TStateCf,
          template <class, class> class TOptVarStruct>
using StateWithGradMap = StateWithGradMapBase<TNum, TStateNm, TStateCf, TOptVarStruct>;

namespace /*<anonymous>*/
{
template <class TNum, template <class, class> class TStateNm, template <class, class> class TStateCf,
          template <class, class> class TOptVarStruct>
struct StateMapBaseTraits<StateWithGradMapBase<TNum, TStateNm, TStateCf, TOptVarStruct>>
{
public:
  using StateType = StateWithGradMapBase<TNum, TStateNm, TStateCf, TOptVarStruct>;

public:
  using StateForSimType = StateWithGradMapSimBase<StateType>;

public:
  using NumType = TNum;

public:
  using StateCfType = TStateCf<TNum, TNum>;

public:
  using StateNmType = TStateNm<TNum, TNum>;

public:
  using StateCfGradType = StateGradWithMatMap<TNum, TStateCf<TNum, TOptVarStruct<TNum, TNum>>>;

public:
  using StateNmGradType = StateGradWithMatMap<TNum, TStateNm<TNum, TOptVarStruct<TNum, TNum>>>;

public:
  using StateGradType = StateGradMapBase<TNum, StateGradWithMatMap<TNum, TStateNm<TNum, TOptVarStruct<TNum, TNum>>>,
                                         StateGradWithMatMap<TNum, TStateCf<TNum, TOptVarStruct<TNum, TNum>>>>;

public:
  using StateVirtType = StateMapBaseVirt<TNum>;

public:
  using StateNmNumType = typename TStateNm<TNum, TNum>::StateMapBaseCRTP::MatrixTypeCRTP;

public:
  using StateWithGradNmType = StateWithGradMapSimBaseInternal<TNum, StateNmType, StateNmGradType>;

public:
  using StateWithGradCfType = StateWithGradMapSimBaseInternal<TNum, StateCfType, StateCfGradType>;

public:
  using StateWithGradNmNumType = typename StateWithGradNmType::StateMapBaseCRTP::MatrixTypeCRTP;
};

}  // namespace <anonymous>

template <class TNumType>
using StateSimBaseVirtMap = StateSimBaseVirt<TNumType, StateMapBaseVirt<TNumType>>;
}

#endif  // STATE_SIM_HPP
