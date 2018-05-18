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

#ifndef EDGE8_HPP
#define EDGE8_HPP

#include <tuw_control/state_sim/state_sim.hpp>
#include <tuw_control/param_func_new/param_func_spline/param_func_spline0_dist.hpp>
#include <tuw_control/utils.h>

namespace tuw
{
namespace Edge8
{
template <class TNumType, class TLeafType>
class StateNm : public StateMapArray<TNumType, TLeafType, 3>
{
public:
  using StateMapArray<TNumType, TLeafType, 3>::StateMapArray;

public:
  auto& x()
  {
    return this->template sub<0>();
  }

public:
  const auto& x() const
  {
    return this->template sub<0>();
  }

public:
  auto& y()
  {
    return this->template sub<1>();
  }

public:
  const auto& y() const
  {
    return this->template sub<1>();
  }

public:
  auto& theta()
  {
    return this->template sub<2>();
  }

public:
  const auto& theta() const
  {
    return this->template sub<2>();
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

template <class TNumType, class TLeafType>
class StateNmWithL : public StateMapArray<TNumType, TLeafType, 4>
{
public:
  using StateMapArray<TNumType, TLeafType, 4>::StateMapArray;

public:
  auto& L()
  {
    return this->template sub<0>();
  }

public:
  const auto& L() const
  {
    return this->template sub<0>();
  }

public:
  auto& x()
  {
    return this->template sub<1>();
  }

public:
  const auto& x() const
  {
    return this->template sub<1>();
  }

public:
  auto& y()
  {
    return this->template sub<2>();
  }

public:
  const auto& y() const
  {
    return this->template sub<2>();
  }

public:
  auto& theta()
  {
    return this->template sub<3>();
  }

public:
  const auto& theta() const
  {
    return this->template sub<3>();
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

template <class TNumType, class TLeafType>
class StateCf : public StateMapArray<TNumType, TLeafType, 6>
{
public:
  using StateMapArray<TNumType, TLeafType, 6>::StateMapArray;

public:
  auto& v()
  {
    return this->template sub<0>();
  }

public:
  const auto& v() const
  {
    return this->template sub<0>();
  }

public:
  auto& phi()
  {
    return this->template sub<1>();
  }

public:
  const auto& phi() const
  {
    return this->template sub<1>();
  }

public:
  auto& vDot()
  {
    return this->template sub<2>();
  }

public:
  const auto& vDot() const
  {
    return this->template sub<2>();
  }

public:
  auto& phiDot()
  {
    return this->template sub<3>();
  }

public:
  const auto& phiDot() const
  {
    return this->template sub<3>();
  }

public:
  auto& t()
  {
    return this->template sub<4>();
  }

public:
  const auto& t() const
  {
    return this->template sub<4>();
  }

public:
  auto& s()
  {
    return this->template sub<5>();
  }

public:
  const auto& s() const
  {
    return this->template sub<5>();
  }
};

static constexpr const size_t optParamBlockSize = 4;

template <class TNumType, typename TLeafType>
class OptVarStructE8 : public StateMapArray<TNumType, StateMapVector<TNumType, TLeafType>, optParamBlockSize>
{
public:
  using StateMapArray<TNumType, StateMapVector<TNumType, TLeafType>, optParamBlockSize>::StateMapArray;

public:
  auto& optParamV()
  {
    return this->template sub<1>();
  }  // parameters influencing linear velocity
public:
  const auto& optParamV() const
  {
    return this->template sub<1>();
  }

public:
  auto& optParamTV()
  {
    return this->template sub<3>();
  }  // parameters influencing temporal location of previous parameters
public:
  const auto& optParamTV() const
  {
    return this->template sub<3>();
  }

public:
  auto& optParamP()
  {
    return this->template sub<0>();
  }  // parameters influencing angular velocity
public:
  const auto& optParamP() const
  {
    return this->template sub<0>();
  }

public:
  auto& optParamTP()
  {
    return this->template sub<2>();
  }  // parameters influencing temporal location of previous parameters
public:
  const auto& optParamTP() const
  {
    return this->template sub<2>();
  }
};

template <class TNumType>
using StateE8 = StateMap<TNumType, StateNm, StateCf>;
template <class TNumType>
using StateWithLE8 = StateMap<TNumType, StateNmWithL, StateCf>;
template <class TNumType>
using StateWithGradE8 = StateWithGradMap<TNumType, StateNm, StateCf, OptVarStructE8>;
template <class TNumType>
using StateWithLWithGradE8 = StateWithGradMap<TNumType, StateNmWithL, StateCf, OptVarStructE8>;

template <class TNumType, class TCfDataType>
struct ParamType
{
  ParamFuncsSpline0Dist<TNumType, 2, 2> paramFuncs;
  StateE8<TNumType> state0;
  TCfDataType cfData;
  enum class ParamFuncVars
  {
    V,
    Phi
  };

  TNumType lB;
};

template <class TNumType, class MapDataType, class TStateType, template <class> class TDiscretizationType,
          class... TFuncsType>
class StateSimE8Base
    : public StateSimBase<StateSimE8Base<TNumType, MapDataType, TStateType, TDiscretizationType, TFuncsType...>,
                          ParamType<TNumType, MapDataType>, TStateType, TDiscretizationType, TFuncsType...>
{
  using PFV = typename ParamType<TNumType, MapDataType>::ParamFuncVars;

public:
  void adjustXSizeImpl(auto& _XNm, auto& _XCf)
  {
    this->paramStruct->paramFuncs.precompute();
  }

private:
  void setXCfNmStep(auto& _XCf, const TNumType& _arc, const PfEaG& _eAG)
  {
    if ((arcCfNmStepCache_ == _arc) && (_eAG == PfEaG::NEAR_LAST))
    {
      return;
    }
    arcCfNmStepCache_ = _arc;
    auto& paramFuncs = this->paramStruct->paramFuncs;
    paramFuncs.setEvalArc(_arc, _eAG);

    _XCf.v() = paramFuncs.computeFuncVal(asInt(PFV::V));
    _XCf.phi() = paramFuncs.computeFuncVal(asInt(PFV::Phi));
  }

public:
  void setXCfImpl(auto& _XCf, const TNumType& _arc, const PfEaG& _eAG)
  {
    setXCfNmStep(_XCf, _arc, _eAG);
    auto& paramFuncs = this->paramStruct->paramFuncs;
    _XCf.vDot() = paramFuncs.computeFuncDiff1(asInt(PFV::V));
    _XCf.phiDot() = paramFuncs.computeFuncDiff1(asInt(PFV::Phi));
    _XCf.t() = _arc;
    _XCf.s() = paramFuncs.computeS();
  }

public:
  void setXCfDotImpl(auto& _XCfDot, const auto& _XCf, const TNumType& _arc, const PfEaG& _eAG) const
  {
    _XCfDot.v() = _XCf.vDot();
    _XCfDot.phi() = _XCf.phiDot();
    _XCfDot.vDot() = 0;
    _XCfDot.phiDot() = 0;
    _XCfDot.t() = 1.;
    _XCfDot.s() = fabs(_XCf.v());
  }

public:
  void setXNm0Impl(auto& _XNm0)
  {
    for (int i = 0; i < _XNm0.data().size(); ++i)
    {
      _XNm0.data()(i) = 0;
    }
    _XNm0.x() = this->paramStruct->state0.stateNm().x();
    _XNm0.y() = this->paramStruct->state0.stateNm().y();
    _XNm0.theta() = this->paramStruct->state0.stateNm().theta();
  }

public:
  void setXNmDotImpl(auto& _XNmDot, auto& _stateCf, const auto& _stateNm, const TNumType& _arc, const PfEaG& _eAG)
  {
    setXCfNmStep(_stateCf, _arc, _eAG);
    if ((arcNmDotCache_ == _arc) && (_eAG == PfEaG::NEAR_LAST))
    {
    }
    else
    {
      tanPhi_ = tan(_stateCf.phi());
      arcNmDotCache_ = _arc;
    }
    _XNmDot.x() = _stateCf.v() * cos(_stateNm.theta());
    _XNmDot.y() = _stateCf.v() * sin(_stateNm.theta());
    _XNmDot.theta() = _stateCf.v() * tanPhi_ / this->paramStruct->lB;
  }

  ///-----------------------------------Gradient information-----------------------------------///

public:
  void setGradXNm0Impl(auto& _gradXNm0, const auto& _XNm0)
  {
    for (int i = 0; i < _gradXNm0.data().size(); ++i)
    {
      _gradXNm0.data()(i) = 0;
    }
  }

public:
  void adjustGradXSizeImpl(auto& _gradXNm, auto& _gradXCf)
  {
    auto& paramFuncs = this->paramStruct->paramFuncs;
    int ctrlPtOptNr = paramFuncs.funcCtrlPtSize(0) - 1;
    if (_gradXNm.sub(0).sub(0).data().size() != ctrlPtOptNr)
    {
      for (size_t i = 0; i < _gradXNm.subSize(); ++i)
      {
        for (size_t j = 0; j < _gradXNm.sub(i).subSize(); ++j)
        {
          if (j != 2)
          {
            _gradXNm.sub(i).sub(j).subResize(ctrlPtOptNr);
          }
          else
          {
            _gradXNm.sub(i).sub(j).subResize(ctrlPtOptNr - 1);
          }
        }
      }
      for (size_t i = 0; i < _gradXCf.subSize(); ++i)
      {
        for (size_t j = 0; j < _gradXCf.sub(i).subSize(); ++j)
        {
          if (j != 2)
          {
            _gradXCf.sub(i).sub(j).subResize(ctrlPtOptNr);
          }
          else
          {
            _gradXCf.sub(i).sub(j).subResize(ctrlPtOptNr - 1);
          }
        }
      }
    }
  }

public:
  void setGradXCfImpl(auto& _gradXCf, const auto& _XCf, const TNumType& _arc, const PfEaG& _eAG)
  {
    setGradXCfNmStep(_gradXCf, _XCf, _arc, _eAG);

    auto& paramFuncs = this->paramStruct->paramFuncs;

    auto& dSdParamV = _gradXCf.s().optParamV();
    for (size_t i = 0; i < dSdParamV.subSize(); ++i)
    {
      auto& dSdParamVI = dSdParamV.sub(i);

      if (i + 1 < dSdParamV.subSize())
      {
        const TNumType& evalArcAbove = paramFuncs.ctrlPtVal(0, i + 2, CtrlPtDim::ARC);
        const TNumType& evalArcBelow = paramFuncs.ctrlPtVal(0, i + 1, CtrlPtDim::ARC);
        if (_arc > evalArcBelow)
        {
          const TNumType arcIntEnd = fmin(_arc, evalArcAbove);
          const TNumType deltaEvalArc = evalArcAbove - evalArcBelow;
          dSdParamVI = +(arcIntEnd - evalArcBelow) * (evalArcBelow - 2. * _arc + arcIntEnd) / (2. * deltaEvalArc);
        }
      }
      const TNumType& evalArcAbove = paramFuncs.ctrlPtVal(0, i + 1, CtrlPtDim::ARC);
      const TNumType& evalArcBelow = paramFuncs.ctrlPtVal(0, i, CtrlPtDim::ARC);
      if (_arc > evalArcBelow)
      {
        const TNumType arcIntEnd = fmin(_arc, evalArcAbove);
        const TNumType deltaEvalArc = evalArcAbove - evalArcBelow;
        dSdParamVI += -(arcIntEnd - evalArcBelow) * (evalArcBelow - 2. * _arc + arcIntEnd) / (2. * deltaEvalArc);
      }
    }

    auto& dSdParamT = _gradXCf.s().optParamTV();
    auto& dAVdParamV = _gradXCf.vDot().optParamV();
    auto& dAPdParamP = _gradXCf.phiDot().optParamP();
    auto& dAVdParamT = _gradXCf.vDot().optParamTV();
    for (size_t i = 0; i < dSdParamT.subSize(); ++i)
    {
      auto& dSdParamTI = dSdParamT.sub(i);
      auto& dAVdParamVI = dAVdParamV.sub(i);
      auto& dAPdParamPI = dAPdParamP.sub(i);
      auto& dAVdParamTI = dAVdParamT.sub(i);
      if (i + 1 < paramFuncs.funcCtrlPtSize(0))
      {
        const TNumType& evalArcAbove = paramFuncs.ctrlPtVal(0, i + 1, CtrlPtDim::ARC);
        const TNumType& evalArcBelow = paramFuncs.ctrlPtVal(0, i, CtrlPtDim::ARC);
        const TNumType& evalArcAboveP = paramFuncs.ctrlPtVal(1, i + 1, CtrlPtDim::ARC);
        const TNumType& evalArcBelowP = paramFuncs.ctrlPtVal(1, i, CtrlPtDim::ARC);
        if ((_arc <= evalArcAbove) && (_arc > evalArcBelow))
        {
          const TNumType& vP = paramFuncs.ctrlPtVal(0, i + 1, CtrlPtDim::VAL);
          const TNumType& vM = paramFuncs.ctrlPtVal(0, i + 0, CtrlPtDim::VAL);
          const TNumType arcIntEnd = fmin(_arc, evalArcAbove);
          const TNumType deltaEvalArc = evalArcAbove - evalArcBelow;
          const TNumType deltaEvalArcSqr = deltaEvalArc * deltaEvalArc;
          const TNumType deltaArcIntBound = arcIntEnd - evalArcBelow;
          dAVdParamTI = -(vP - vM) / deltaEvalArcSqr;
          dAVdParamVI = +1. / deltaEvalArc;
          dSdParamTI = +deltaArcIntBound * (vP - vM) * (evalArcBelow - 2. * _arc + arcIntEnd) / (2. * deltaEvalArcSqr);
        }
        else if ((_arc > evalArcBelow) && (i + 1 < dSdParamT.subSize()))
        {
          const TNumType& evalArcAbove2 = paramFuncs.ctrlPtVal(0, i + 2, CtrlPtDim::ARC);
          const TNumType deltaEvalArc = evalArcAbove2 - evalArcAbove;
          const TNumType deltaEvalArcSqr = deltaEvalArc * deltaEvalArc;
          if (_arc <= evalArcAbove2)
          {
            dAVdParamVI = -1. / deltaEvalArc;
            const TNumType& vP = paramFuncs.ctrlPtVal(0, i + 2, CtrlPtDim::VAL);
            const TNumType& vM = paramFuncs.ctrlPtVal(0, i + 1, CtrlPtDim::VAL);
            dAVdParamTI = +(vP - vM) / deltaEvalArcSqr;
          }
          if (_arc < evalArcAbove2)
          {
            const TNumType& vP = paramFuncs.ctrlPtVal(0, i + 2, CtrlPtDim::VAL);
            const TNumType& vM = paramFuncs.ctrlPtVal(0, i + 1, CtrlPtDim::VAL);
            const TNumType& vMM = paramFuncs.ctrlPtVal(0, i + 0, CtrlPtDim::VAL);
            const TNumType arcIntEnd = fmin(_arc, evalArcAbove2);
            dSdParamTI = -(+(vM - vMM) * evalArcAbove2 * evalArcAbove2 +
                           (vP - vMM) * (evalArcAbove * evalArcAbove - 2. * evalArcAbove * evalArcAbove2) +
                           (vP - vM) * (arcIntEnd * arcIntEnd + 2. * _arc * (evalArcAbove2 - arcIntEnd))) /
                         (2. * deltaEvalArcSqr);
          }
          else
          {
            const TNumType& vP = paramFuncs.ctrlPtVal(0, i + 2, CtrlPtDim::VAL);
            const TNumType& vM = paramFuncs.ctrlPtVal(0, i + 0, CtrlPtDim::VAL);
            dSdParamTI = -(vP - vM) / 2.;
          }
        }
        else if (_arc > evalArcBelow)
        {
          const TNumType& vP = paramFuncs.ctrlPtVal(0, i + 1, CtrlPtDim::VAL);
          const TNumType& vM = paramFuncs.ctrlPtVal(0, i + 0, CtrlPtDim::VAL);
          dSdParamTI = -(vP - vM) / 2.;
        }

        if ((_arc <= evalArcAboveP) && (_arc > evalArcBelowP))
        {
          dAPdParamPI = +1. / (evalArcAboveP - evalArcBelowP);
        }
        else if ((_arc > evalArcBelowP) && (i + 1 < dSdParamT.subSize()))
        {
          const TNumType& evalArcAbove2P = paramFuncs.ctrlPtVal(1, i + 2, CtrlPtDim::ARC);
          const TNumType deltaEvalArcP = evalArcAbove2P - evalArcAboveP;
          if (_arc <= evalArcAbove2P)
          {
            dAPdParamPI = -1. / deltaEvalArcP;
          }
        }
      }
    }

    auto& dAPdParamT = _gradXCf.phiDot().optParamTP();
    for (size_t i = 0; i < dAPdParamT.subSize(); ++i)
    {
      auto& dAPdParamTI = dAPdParamT.sub(i);
      if (i + 1 < paramFuncs.funcCtrlPtSize(1))
      {
        const TNumType& evalArcAbove = paramFuncs.ctrlPtVal(1, i + 1, CtrlPtDim::ARC);
        const TNumType& evalArcBelow = paramFuncs.ctrlPtVal(1, i, CtrlPtDim::ARC);
        if ((_arc <= evalArcAbove) && (_arc > evalArcBelow))
        {
          const TNumType& wP = paramFuncs.ctrlPtVal(1, i + 1, CtrlPtDim::VAL);
          const TNumType& wM = paramFuncs.ctrlPtVal(1, i + 0, CtrlPtDim::VAL);
          const TNumType deltaEvalArc = evalArcAbove - evalArcBelow;
          const TNumType deltaEvalArcSqr = deltaEvalArc * deltaEvalArc;
          dAPdParamTI = -(wP - wM) / deltaEvalArcSqr;
        }
        else if ((_arc > evalArcBelow) && (i + 1 < dAPdParamT.subSize() + 1))
        {  ///@todo subSize()+1 is correct???
          const TNumType& evalArcAbove2 = paramFuncs.ctrlPtVal(1, i + 2, CtrlPtDim::ARC);
          const TNumType deltaEvalArc = evalArcAbove2 - evalArcAbove;
          const TNumType deltaEvalArcSqr = deltaEvalArc * deltaEvalArc;
          if (_arc <= evalArcAbove2)
          {
            const TNumType& wP = paramFuncs.ctrlPtVal(1, i + 2, CtrlPtDim::VAL);
            const TNumType& wM = paramFuncs.ctrlPtVal(1, i + 1, CtrlPtDim::VAL);
            dAPdParamTI = +(wP - wM) / deltaEvalArcSqr;
          }
        }
      }
    }
  }

private:
  void setGradXCfNmStep(auto& _gradXCf, const auto& _XCf, const TNumType& _arc, const PfEaG& _eAG)
  {
    if ((arcGradCache_ == _arc) && (_eAG == PfEaG::NEAR_LAST))
    {
      return;
    }
    arcGradCache_ = _arc;
    auto& paramFuncs = this->paramStruct->paramFuncs;

    _gradXCf.data().setZero();
    auto& dVdParamV = _gradXCf.v().optParamV();
    for (size_t i = 0; i < dVdParamV.subSize(); ++i)
    {
      auto& dVdParamVI = dVdParamV.sub(i);

      if (i + 1 < dVdParamV.subSize())
      {
        const TNumType& evalArcAbove = paramFuncs.ctrlPtVal(0, i + 2, CtrlPtDim::ARC);
        const TNumType& evalArcBelow = paramFuncs.ctrlPtVal(0, i + 1, CtrlPtDim::ARC);
        if (_arc > evalArcBelow)
        {
          const TNumType arcIntEnd = fmin(_arc, evalArcAbove);
          const TNumType deltaEvalArc = evalArcAbove - evalArcBelow;
          dVdParamVI = -(arcIntEnd - evalArcBelow) / deltaEvalArc;
        }
      }
      const TNumType& evalArcAbove = paramFuncs.ctrlPtVal(0, i + 1, CtrlPtDim::ARC);
      const TNumType& evalArcBelow = paramFuncs.ctrlPtVal(0, i, CtrlPtDim::ARC);
      if (_arc > evalArcBelow)
      {
        const TNumType arcIntEnd = fmin(_arc, evalArcAbove);
        const TNumType deltaEvalArc = evalArcAbove - evalArcBelow;
        dVdParamVI += +(arcIntEnd - evalArcBelow) / deltaEvalArc;
      }
    }

    auto& dPdParamP = _gradXCf.phi().optParamP();
    for (size_t i = 0; i < dPdParamP.subSize(); ++i)
    {
      auto& dPdParamPI = dPdParamP.sub(i);
      if (i + 1 < dPdParamP.subSize())
      {
        const TNumType& evalArcAbove = paramFuncs.ctrlPtVal(1, i + 2, CtrlPtDim::ARC);
        const TNumType& evalArcBelow = paramFuncs.ctrlPtVal(1, i + 1, CtrlPtDim::ARC);
        if (_arc > evalArcBelow)
        {
          const TNumType arcIntEnd = fmin(_arc, evalArcAbove);
          const TNumType deltaEvalArc = evalArcAbove - evalArcBelow;
          dPdParamPI = -(arcIntEnd - evalArcBelow) / deltaEvalArc;
        }
      }
      const TNumType& evalArcAbove = paramFuncs.ctrlPtVal(1, i + 1, CtrlPtDim::ARC);
      const TNumType& evalArcBelow = paramFuncs.ctrlPtVal(1, i, CtrlPtDim::ARC);
      if (_arc > evalArcBelow)
      {
        const TNumType arcIntEnd = fmin(_arc, evalArcAbove);
        const TNumType deltaEvalArc = evalArcAbove - evalArcBelow;
        dPdParamPI += +(arcIntEnd - evalArcBelow) / deltaEvalArc;
      }
    }

    auto& dVdParamT = _gradXCf.v().optParamTV();
    for (size_t i = 0; i < dVdParamT.subSize(); ++i)
    {
      auto& dVdParamTI = dVdParamT.sub(i);
      if (i + 1 < paramFuncs.funcCtrlPtSize(0))
      {
        const TNumType& evalArcAbove = paramFuncs.ctrlPtVal(0, i + 1, CtrlPtDim::ARC);
        const TNumType& evalArcBelow = paramFuncs.ctrlPtVal(0, i, CtrlPtDim::ARC);
        if ((_arc <= evalArcAbove) && (_arc > evalArcBelow))
        {
          const TNumType& vP = paramFuncs.ctrlPtVal(0, i + 1, CtrlPtDim::VAL);
          const TNumType& vM = paramFuncs.ctrlPtVal(0, i + 0, CtrlPtDim::VAL);
          const TNumType arcIntEnd = fmin(_arc, evalArcAbove);
          const TNumType deltaEvalArc = evalArcAbove - evalArcBelow;
          const TNumType deltaEvalArcSqr = deltaEvalArc * deltaEvalArc;
          const TNumType deltaArcIntBound = arcIntEnd - evalArcBelow;
          dVdParamTI = -deltaArcIntBound * (vP - vM) / deltaEvalArcSqr;
        }
        else if ((_arc > evalArcBelow) && (i + 1 < dVdParamT.subSize()))
        {
          const TNumType& evalArcAbove2 = paramFuncs.ctrlPtVal(0, i + 2, CtrlPtDim::ARC);
          const TNumType deltaEvalArc = evalArcAbove2 - evalArcAbove;
          const TNumType deltaEvalArcSqr = deltaEvalArc * deltaEvalArc;
          if (_arc < evalArcAbove2)
          {
            const TNumType& vP = paramFuncs.ctrlPtVal(0, i + 2, CtrlPtDim::VAL);
            const TNumType& vM = paramFuncs.ctrlPtVal(0, i + 1, CtrlPtDim::VAL);
            const TNumType arcIntEnd = fmin(_arc, evalArcAbove2);

            const TNumType deltaArcIntBound = evalArcAbove2 - arcIntEnd;
            dVdParamTI = -deltaArcIntBound * (vP - vM) / deltaEvalArcSqr;
          }
        }
      }
    }

    auto& dPdParamT = _gradXCf.phi().optParamTP();
    for (size_t i = 0; i < dPdParamT.subSize(); ++i)
    {
      auto& dPdParamTI = dPdParamT.sub(i);
      if (i + 1 < paramFuncs.funcCtrlPtSize(1))
      {
        const TNumType& evalArcAbove = paramFuncs.ctrlPtVal(1, i + 1, CtrlPtDim::ARC);
        const TNumType& evalArcBelow = paramFuncs.ctrlPtVal(1, i, CtrlPtDim::ARC);
        if ((_arc <= evalArcAbove) && (_arc > evalArcBelow))
        {
          const TNumType& wP = paramFuncs.ctrlPtVal(1, i + 1, CtrlPtDim::VAL);
          const TNumType& wM = paramFuncs.ctrlPtVal(1, i + 0, CtrlPtDim::VAL);
          const TNumType arcIntEnd = fmin(_arc, evalArcAbove);
          const TNumType deltaEvalArc = evalArcAbove - evalArcBelow;
          const TNumType deltaEvalArcSqr = deltaEvalArc * deltaEvalArc;
          const TNumType deltaArcIntBound = arcIntEnd - evalArcBelow;
          dPdParamTI = -deltaArcIntBound * (wP - wM) / deltaEvalArcSqr;
        }
        else if ((_arc > evalArcBelow) && (i + 1 < dPdParamT.subSize() + 1))
        {  ///@todo subSize()+1 is correct???
          const TNumType& evalArcAbove2 = paramFuncs.ctrlPtVal(1, i + 2, CtrlPtDim::ARC);
          const TNumType deltaEvalArc = evalArcAbove2 - evalArcAbove;
          const TNumType deltaEvalArcSqr = deltaEvalArc * deltaEvalArc;
          if (_arc < evalArcAbove2)
          {
            const TNumType& wP = paramFuncs.ctrlPtVal(1, i + 2, CtrlPtDim::VAL);
            const TNumType& wM = paramFuncs.ctrlPtVal(1, i + 1, CtrlPtDim::VAL);
            const TNumType arcIntEnd = fmin(_arc, evalArcAbove2);

            const TNumType deltaArcIntBound = evalArcAbove2 - arcIntEnd;
            dPdParamTI = -deltaArcIntBound * (wP - wM) / deltaEvalArcSqr;
          }
        }
      }
    }
  }

public:
  void setGradXNmDotImpl(auto& _gradXNmDot, auto& _XGradXCf, const auto& _XGradXNm, const TNumType& _arc,
                         const PfEaG& _eAG)
  {
    if ((arcGradNmStepCache_ == _arc) && (_eAG == PfEaG::NEAR_LAST))
    {
      return;
    }
    arcGradNmStepCache_ = _arc;

    auto& gradXCf = _XGradXCf.stateGrad();
    const auto& XCf = _XGradXCf.state();
    setGradXCfNmStep(gradXCf, XCf, _arc, _eAG);

    // combining dfdx * GradX + dfdu * dudp
    static Eigen::Matrix<TNumType, 2, 1> dfduX;
    static Eigen::Matrix<TNumType, 2, 1> dfdTh;
    const TNumType vPhiSqrForm = XCf.v() * (1 + tanPhi_ * tanPhi_);

    cosTheta_ = cos(_XGradXNm.state().theta());
    sinTheta_ = sin(_XGradXNm.state().theta());

    dfduX(0) = cosTheta_;
    dfduX(1) = sinTheta_;

    dfdTh(0) = -XCf.v() * sinTheta_;
    dfdTh(1) = +XCf.v() * cosTheta_;

    auto& dXdParamV = _gradXNmDot.x().optParamV().data();
    auto& dYdParamV = _gradXNmDot.y().optParamV().data();
    auto& dThdParamV = _gradXNmDot.theta().optParamV().data();
    auto& dXdParamP = _gradXNmDot.x().optParamP().data();
    auto& dYdParamP = _gradXNmDot.y().optParamP().data();
    auto& dThdParamP = _gradXNmDot.theta().optParamP().data();

    auto& dXdParamTV = _gradXNmDot.x().optParamTV().data();
    auto& dYdParamTV = _gradXNmDot.y().optParamTV().data();
    auto& dThdParamTV = _gradXNmDot.theta().optParamTV().data();

    auto& dXdParamTP = _gradXNmDot.x().optParamTP().data();
    auto& dYdParamTP = _gradXNmDot.y().optParamTP().data();
    auto& dThdParamTP = _gradXNmDot.theta().optParamTP().data();

    const auto& stateGradCfParamVIV = gradXCf.v().optParamV();
    const auto& stateGradCfParamWIPhi = gradXCf.phi().optParamP();

    const auto& stateGradCfParamTVIV = gradXCf.v().optParamTV();
    const auto& stateGradCfParamTPIV = gradXCf.v().optParamTP();
    const auto& stateGradCfParamTVIPhi = gradXCf.phi().optParamTV();
    const auto& stateGradCfParamTPIPhi = gradXCf.phi().optParamTP();

    auto& gradXNm = _XGradXNm.stateGrad();
    const auto& stateGradNmParamVITh = gradXNm.theta().optParamV();
    const auto& stateGradNmParamPITh = gradXNm.theta().optParamP();
    const auto& stateGradNmParamTVITh = gradXNm.theta().optParamTV();
    const auto& stateGradNmParamTPITh = gradXNm.theta().optParamTP();

    for (int i = 0; i < dXdParamV.size(); ++i)
    {
      const auto& stateGradCfParamVIVI = stateGradCfParamVIV.sub(i);
      const auto& stateGradCfParamWIPhiI = stateGradCfParamWIPhi.sub(i);

      const auto& stateGradNmParamVIThI = stateGradNmParamVITh.sub(i);
      const auto& stateGradNmParamPIThI = stateGradNmParamPITh.sub(i);

      const auto& stateGradCfParamTVIVI = stateGradCfParamTVIV.sub(i);
      const auto& stateGradCfParamTVIPhiI = stateGradCfParamTVIPhi.sub(i);
      const auto& stateGradNmParamTVIThI = stateGradNmParamTVITh.sub(i);

      dXdParamV(i) = dfduX(0) * stateGradCfParamVIVI + dfdTh(0) * stateGradNmParamVIThI;
      dYdParamV(i) = dfduX(1) * stateGradCfParamVIVI + dfdTh(1) * stateGradNmParamVIThI;

      dXdParamP(i) = +dfdTh(0) * stateGradNmParamPIThI;
      dYdParamP(i) = +dfdTh(1) * stateGradNmParamPIThI;

      dXdParamTV(i) = dfduX(0) * stateGradCfParamTVIVI + dfdTh(0) * stateGradNmParamTVIThI;
      dYdParamTV(i) = dfduX(1) * stateGradCfParamTVIVI + dfdTh(1) * stateGradNmParamTVIThI;

      dThdParamV(i) = (stateGradCfParamVIVI * tanPhi_) / this->paramStruct->lB;

      dThdParamP(i) = (vPhiSqrForm * stateGradCfParamWIPhiI) / this->paramStruct->lB;

      dThdParamTV(i) =
          (vPhiSqrForm * stateGradCfParamTVIPhiI + stateGradCfParamTVIVI * tanPhi_) / this->paramStruct->lB;
    }
    for (int i = 0; i < dXdParamV.size() - 1; ++i)
    {
      const auto& stateGradCfParamTPIPhiI = stateGradCfParamTPIPhi.sub(i);
      const auto& stateGradCfParamTPIVI = stateGradCfParamTPIV.sub(i);
      const auto& stateGradNmParamTPIThI = stateGradNmParamTPITh.sub(i);

      dXdParamTP(i) = dfduX(0) * stateGradCfParamTPIVI + dfdTh(0) * stateGradNmParamTPIThI;
      dYdParamTP(i) = dfduX(1) * stateGradCfParamTPIVI + dfdTh(1) * stateGradNmParamTPIThI;

      dThdParamTP(i) =
          (vPhiSqrForm * stateGradCfParamTPIPhiI + stateGradCfParamTPIVI * tanPhi_) / this->paramStruct->lB;
    }
  }

  // internal helper variables
protected:
  TNumType tanPhi_;

protected:
  TNumType cosTheta_;

protected:
  TNumType sinTheta_;

protected:
  TNumType arcCfNmStepCache_;

protected:
  TNumType arcNmDotCache_;

protected:
  TNumType arcGradCache_;

protected:
  TNumType arcGradNmStepCache_;
};
//---------------------------------------------------------------------Optimization parameters

template <class TNumType, class TParamStructType>
struct OptVarMapE8
{
  static void setOptVar(TParamStructType& _paramStruct, const Eigen::Matrix<TNumType, -1, 1>& _optVarExt)
  {
    auto& paramFuncs = _paramStruct.paramFuncs;
    size_t idxOptVec = 0;
    for (int i = paramFuncs.funcsSize() - 1; i >= 0; --i)
    {
      for (size_t j = 1; j < paramFuncs.funcCtrlPtSize(i); ++j)
      {
        paramFuncs.ctrlPtVal(i, j, CtrlPtDim::VAL) = _optVarExt(idxOptVec++);
      }
    }
    for (size_t j = 1; j < paramFuncs.funcsArcSize(1) - 1; ++j)
    {
      paramFuncs.funcsArc(1, j) = _optVarExt(idxOptVec++);
    }
    for (size_t j = 1; j < paramFuncs.funcsArcSize(0); ++j)
    {
      paramFuncs.funcsArc(0, j) = _optVarExt(idxOptVec++);
    }
  }
  static void getOptVar(Eigen::Matrix<TNumType, -1, 1>& _optVarExt, const TParamStructType& _paramStruct)
  {
    auto& paramFuncs = _paramStruct.paramFuncs;
    int newSize = paramFuncs.funcsSize() * (paramFuncs.funcCtrlPtSize(0) - 1) + (paramFuncs.funcsArcSize(0) - 1) +
                  (paramFuncs.funcsArcSize(1) - 2);
    if (newSize != _optVarExt.size())
    {
      _optVarExt.resize(newSize);
    }
    size_t idxOptVec = 0;
    for (int i = paramFuncs.funcsSize() - 1; i >= 0; --i)
    {
      for (size_t j = 1; j < paramFuncs.funcCtrlPtSize(i); ++j)
      {
        _optVarExt(idxOptVec++) = paramFuncs.ctrlPtVal(i, j, CtrlPtDim::VAL);
      }
    }
    for (size_t j = 1; j < paramFuncs.funcsArcSize(1) - 1; ++j)
    {
      _optVarExt(idxOptVec++) = paramFuncs.funcsArc(1, j);
    }
    for (size_t j = 1; j < paramFuncs.funcsArcSize(0); ++j)
    {
      _optVarExt(idxOptVec++) = paramFuncs.funcsArc(0, j);
    }
  }
};

template <class TNumType, class TMapDataType, template <class> class TDiscretizationType>
class StateSimVW : public StateSimE8Base<TNumType, TMapDataType, StateE8<TNumType>, TDiscretizationType>
{
};

template <class TNumType, class TMapDataType, template <class> class TDiscretizationType, class... TCostFuncType>
class StateWithLSimVW
    : public StateSimE8Base<TNumType, TMapDataType, StateWithLE8<TNumType>, TDiscretizationType, TCostFuncType...>
{
};

template <class TNumType, class TMapDataType, template <class> class TDiscretizationType>
class StateWithGradSimVW : public StateSimE8Base<TNumType, TMapDataType, StateWithGradE8<TNumType>, TDiscretizationType>
{
};

template <class TNumType, class TMapDataType, template <class> class TDiscretizationType, class... TCostFuncType>
class StateWithLWithGradSimVW : public StateSimE8Base<TNumType, TMapDataType, StateWithLWithGradE8<TNumType>,
                                                      TDiscretizationType, TCostFuncType...>
{
};
}
}

#endif  // EDGE8_HPP
