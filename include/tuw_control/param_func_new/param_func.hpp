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

#ifndef PARAM_FUNC_HPP
#define PARAM_FUNC_HPP

#include <memory>
#include <vector>
#include <map>
#include <tuw_control/utils.h>

namespace tuw
{
// enums
///@brief Required type of computation relative to the parametric function.
enum class FuncEvalMode
{
  DIFF1,  ///<function first derivative
  DIFF2,  ///<function second derivative
  INT1,   ///<function single integral
  INT2,   ///<function double integral
  FUNC,   ///<function value (always assumed to have the highest integer value of the enum)
  ENUM_SIZE
};
///@brief Control point variable dimension.
enum class CtrlPtDim
{
  VAL,  ///<control point value
  ARC   ///<control point arc parameter
};
///@brief Flags if any guarantees about evaluation arc relative to last evaluation arc are present.
enum class EvalArcGuarantee
{
  NONE,
  NEAR_LAST,  ///<close to previous evaluation arc
  AT_BEGIN,   ///<this evaluation arc is at the arc parametrization begin
  AT_END      ///<this evaluation arc is at the arc parametrization end
};

///@brief Maximum number of computation modes (except the parametric function itself @ref FUNC). Currently 4 other
///choices supported. @see FuncEvalMode.
static constexpr const std::size_t nrModesMax_ = asInt(FuncEvalMode::ENUM_SIZE) - 1;
using FuncEvalModesFlags = std::array<bool, nrModesMax_>;

// structs
///@brief Containts parametric function initialization data.
struct ParamFuncsStructure
{
  ParamFuncsStructure() : ctrlPtsSize(0), ctrlPtsArcRefIdx(0), evalReq()
  {
    evalReq.fill(false);
  }
  std::size_t ctrlPtsSize;  ///<number of parametric function control points   ( @ref ctrlPtsSize > max(BSpline order,
                            ///2) ! )
  std::size_t ctrlPtsArcRefIdx;  ///<arc parameter structure index used by the parametric function (if two functions f1,
                                 ///f2 share the same arc index, @ref ctrlPtsSize(f1) has to be equal to @ref
                                 ///ctrlPtsSize(f2)!
  FuncEvalModesFlags evalReq;    ///<flags for required function computation modes
};
///@brief Control point variable.
template <typename TNumType>
struct FuncCtrlPt
{
  FuncCtrlPt(const TNumType& _val, TNumType& _arc) : val(_val), arc(_arc)
  {
  }
  TNumType val;   ///<control point value
  TNumType& arc;  ///<control point arc parameter
};

namespace
{
template <class TDerived>
struct ParamFuncsBaseCRTPTraits;
}

template <typename TDerived>
class ParamFuncsBaseCRTP
{
protected:
  using eAG = EvalArcGuarantee;

protected:
  using NumType = typename ParamFuncsBaseCRTPTraits<TDerived>::NumType;

protected:
  using FuncCtrlPtType = FuncCtrlPt<NumType>;

  // special class member functions
public:
  ParamFuncsBaseCRTP() = default;

public:
  ~ParamFuncsBaseCRTP() = default;

public:
  ParamFuncsBaseCRTP(const ParamFuncsBaseCRTP&) = default;

public:
  ParamFuncsBaseCRTP& operator=(const ParamFuncsBaseCRTP&) = default;

public:
  ParamFuncsBaseCRTP(ParamFuncsBaseCRTP&&) = default;

public:
  ParamFuncsBaseCRTP& operator=(ParamFuncsBaseCRTP&&) = default;

  /**
   * @brief Initializes the control structure.
   * @param _paramFuncsStructure parametric functions structure
   *
   * Note: @ref _paramFuncsStructure[].ctrlPtsArcRefIdx has to be consistent (first function has to have value 0. Next
   *functions can only reference previous referenced indexes
   * or increse maximum reference index by 1). Some examples:
   *
   * _paramFuncsStructure[].ctrlPtsArcRefIdx = [0,0,0] ok! means one arc parameter set for all 3 functions;
   *
   * _paramFuncsStructure[].ctrlPtsArcRefIdx = [0,1,0] ok! means one arc parameter set for function 0 and 2 and another
   *arc parameter set for function 1;
   *
   * _paramFuncsStructure[].ctrlPtsArcRefIdx = [0,1,2] ok! means separate arc parameter set for each function;
   *
   * _paramFuncsStructure[].ctrlPtsArcRefIdx = [0,2,1] inconsistent! indexes not consistent step-increasing;
   *
   **/
public:
  template <typename TParamFucsStructVecArr>
  void init(const TParamFucsStructVecArr& _paramFuncsStructure)
  {
    thisDerived().initImplCRTP(_paramFuncsStructure);
  }
  ///@brief Number of parametric functions.
public:
  std::size_t funcsSize() const
  {
    return thisDerived().funcsSizeImplCRTP();
  }
  ///@brief Number of arc parametrizations.
public:
  std::size_t funcsArcSize() const
  {
    return thisDerived().funcsArcSizeImplCRTP();
  }
  ///@brief Number of control points of parametrization @ref \_i.
public:
  std::size_t funcsArcSize(const std::size_t& _i) const
  {
    return thisDerived().funcsArcSizeImplCRTP(_i);
  }
  ///@brief Number of control points for a parametric function.
public:
  std::size_t funcCtrlPtSize(const std::size_t& _i) const
  {
    return thisDerived().funcCtrlPtSizeImplCRTP(_i);
  }
  ///@brief Access to the initial value of the arc parametrization.
public:
  NumType& funcsArcBegin()
  {
    return thisDerived().funcsArcBeginImplCRTP();
  }
  ///@brief Const access to the initial value of the arc parametrization.
public:
  const NumType& funcsArcBegin() const
  {
    return thisDerived().funcsArcBeginImplCRTP();
  }
  ///@brief Access to the final value of the arc parametrization.
public:
  NumType& funcsArcEnd()
  {
    return thisDerived().funcsArcEndImplCRTP();
  }
  ///@brief Const access to the finall value of the arc parametrization.
public:
  const NumType& funcsArcEnd() const
  {
    return thisDerived().funcsArcEndImplCRTP();
  }
  ///@brief Const access to the active evaluation point arc parametrization.
public:
  const NumType& funcsArcEval() const
  {
    return thisDerived().funcsArcEvalImplCRTP();
  }
  ///@brief Access to the arc parameter vector at index @ref \_i, control point @ref \_j.
public:
  NumType& funcsArc(const std::size_t& _i, const std::size_t& _j)
  {
    return thisDerived().funcsArcImplCRTP(_i, _j);
  }
  ///@brief Const access the arc parameter vector at index @ref \_i, control point @ref \_j.
public:
  const NumType& funcsArc(const std::size_t& _i, const std::size_t& _j) const
  {
    return thisDerived().funcsArcImplCRTP(_i, _j);
  }
  ///@brief Access of a parametric function control point.
public:
  FuncCtrlPtType& ctrlPt(const std::size_t& _funcIdx, const std::size_t& _funcCtrlPtIdx)
  {
    return thisDerived().ctrlPtImplCRTP(_funcIdx, _funcCtrlPtIdx);
  }
  ///@brief Const access of a parametric function control point.
public:
  const FuncCtrlPtType& ctrlPt(const std::size_t& _funcIdx, const std::size_t& _funcCtrlPtIdx) const
  {
    return thisDerived().ctrlPtImplCRTP(_funcIdx, _funcCtrlPtIdx);
  }
  ///@brief Access of a parametric function control point dimension.
public:
  NumType& ctrlPtVal(const std::size_t& _funcIdx, const std::size_t& _funcCtrlPtIdx,
                     const CtrlPtDim& _ctrlPtDim = CtrlPtDim::VAL)
  {
    return thisDerived().ctrlPtValImplCRTP(_funcIdx, _funcCtrlPtIdx, _ctrlPtDim);
  }
  ///@brief Const of a parametric function control point dimension.
public:
  const NumType& ctrlPtVal(const std::size_t& _funcIdx, const std::size_t& _funcCtrlPtIdx,
                           const CtrlPtDim& _ctrlPtDim = CtrlPtDim::VAL) const
  {
    return thisDerived().ctrlPtValImplCRTP(_funcIdx, _funcCtrlPtIdx, _ctrlPtDim);
  }

public:
  void shiftStartCtrlPt(const NumType& _dt)
  {
    thisDerived().shiftStartCtrlPtImplCRTP(_dt);
  }

  ///@brief Precomputes cached data.
public:
  void precompute()
  {
    thisDerived().precomputeImplCRTP();
  }
  ///@brief Sets parametric function evaluation arc.
public:
  void setEvalArc(const NumType& _arcEval, const eAG& _eAG = eAG::NONE)
  {
    thisDerived().setEvalArcImplCRTP(_arcEval, _eAG);
  }
  ///@brief Computes value of parametric function with index @ref \_funcIdx at parametric arc set by @ref setEvalArc.
public:
  NumType computeFuncVal(const std::size_t& _funcIdx) const
  {
    return thisDerived().computeFuncValImplCRTP(_funcIdx);
  }
  ///@brief Computes 1st derivative of parametric function with index @ref \_funcIdx at parametric arc set by @ref
  ///setEvalArc.
public:
  NumType computeFuncDiff1(const std::size_t& _funcIdx) const
  {
    return thisDerived().computeFuncDiff1ImplCRTP(_funcIdx);
  }
  ///@brief Computes 2nd derivative of parametric function with index @ref \_funcIdx at parametric arc set by @ref
  ///setEvalArc.
public:
  NumType computeFuncDiff2(const std::size_t& _funcIdx) const
  {
    return thisDerived().computeFuncDiff2ImplCRTP(_funcIdx);
  }
  ///@brief Computes integral of parametric function with index @ref \_funcIdx on interval [@ref funcsArcBegin\_, @ref
  ///funcsArcEnd\_].
public:
  NumType computeFuncInt1(const std::size_t& _funcIdx) const
  {
    return thisDerived().computeFuncInt1ImplCRTP(_funcIdx);
  }
  ///@brief Computes double integral of parametric function with index @ref \_funcIdx on interval [@ref funcsArcBegin\_,
  ///@ref funcsArcEnd\_].
public:
  NumType computeFuncInt2(const std::size_t& _funcIdx) const
  {
    return thisDerived().computeFuncInt2ImplCRTP(_funcIdx);
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
template <typename TNumType>
class ParamFuncsBaseVirt
{
protected:
  using eAG = EvalArcGuarantee;

protected:
  using FuncCtrlPtType = FuncCtrlPt<TNumType>;

  // special class member functions
public:
  ParamFuncsBaseVirt() = default;

public:
  virtual ~ParamFuncsBaseVirt() = default;

public:
  ParamFuncsBaseVirt(const ParamFuncsBaseVirt&) = default;

public:
  ParamFuncsBaseVirt& operator=(const ParamFuncsBaseVirt&) = default;

public:
  ParamFuncsBaseVirt(ParamFuncsBaseVirt&&) = default;

public:
  ParamFuncsBaseVirt& operator=(ParamFuncsBaseVirt&&) = default;

  /**
   * @brief Initializes the control structure.
   * @param _paramFuncsStructure parametric functions structure
   *
   * Note: @ref _paramFuncsStructure[].ctrlPtsArcRefIdx has to be consistent (first function has to have value 0. Next
   *functions can only reference previous referenced indexes
   * or increse maximum reference index by 1). Some examples:
   *
   * _paramFuncsStructure[].ctrlPtsArcRefIdx = [0,0,0] ok! means one arc parameter set for all 3 functions;
   *
   * _paramFuncsStructure[].ctrlPtsArcRefIdx = [0,1,0] ok! means one arc parameter set for function 0 and 2 and another
   *arc parameter set for function 1;
   *
   * _paramFuncsStructure[].ctrlPtsArcRefIdx = [0,1,2] ok! means separate arc parameter set for each function;
   *
   * _paramFuncsStructure[].ctrlPtsArcRefIdx = [0,2,1] inconsistent! indexes not consistent step-increasing;
   *
   **/
public:
  void init(const std::vector<ParamFuncsStructure>& _paramFuncsStructure)
  {
    initImplVirt(_paramFuncsStructure);
  }
  ///@brief Number of parametric functions.
public:
  std::size_t funcsSize() const
  {
    return funcsSizeImplVirt();
  }
  ///@brief Number of arc parametrizations.
public:
  std::size_t funcsArcSize() const
  {
    return funcsArcSizeImplVirt();
  }
  ///@brief Number of control points of parametrization @ref \_i.
public:
  std::size_t funcsArcSize(const std::size_t& _i) const
  {
    return funcsArcSizeImplVirt(_i);
  }
  ///@brief Number of control points for a parametric function.
public:
  std::size_t funcCtrlPtSize(const std::size_t& _i) const
  {
    return funcCtrlPtSizeImplVirt(_i);
  }
  ///@brief Access to the initial value of the arc parametrization.
public:
  TNumType& funcsArcBegin()
  {
    return funcsArcBeginImplVirt();
  }
  ///@brief Const access to the initial value of the arc parametrization.
public:
  const TNumType& funcsArcBegin() const
  {
    return funcsArcBeginImplVirt();
  }
  ///@brief Access to the final value of the arc parametrization.
public:
  TNumType& funcsArcEnd()
  {
    return funcsArcEndImplVirt();
  }
  ///@brief Const access to the finall value of the arc parametrization.
public:
  const TNumType& funcsArcEnd() const
  {
    return funcsArcEndImplVirt();
  }
  ///@brief Const access to the active evaluation point arc parametrization.
public:
  const TNumType& funcsArcEval() const
  {
    return funcsArcEvalImplVirt();
  }
  ///@brief Access to the arc parameter vector at index @ref \_i, control point @ref \_j.
public:
  TNumType& funcsArc(const std::size_t& _i, const std::size_t& _j)
  {
    return funcsArcImplVirt(_i, _j);
  }
  ///@brief Const access the arc parameter vector at index @ref \_i, control point @ref \_j.
public:
  const TNumType& funcsArc(const std::size_t& _i, const std::size_t& _j) const
  {
    return funcsArcImplVirt(_i, _j);
  }
  ///@brief Access of a parametric function control point.
public:
  FuncCtrlPtType& ctrlPt(const std::size_t& _funcIdx, const std::size_t& _funcCtrlPtIdx)
  {
    return ctrlPtImplVirt(_funcIdx, _funcCtrlPtIdx);
  }
  ///@brief Const access of a parametric function control point.
public:
  const FuncCtrlPtType& ctrlPt(const std::size_t& _funcIdx, const std::size_t& _funcCtrlPtIdx) const
  {
    return ctrlPtImplVirt(_funcIdx, _funcCtrlPtIdx);
  }
  ///@brief Access of a parametric function control point dimension.
public:
  TNumType& ctrlPtVal(const std::size_t& _funcIdx, const std::size_t& _funcCtrlPtIdx,
                      const CtrlPtDim& _ctrlPtDim = CtrlPtDim::VAL)
  {
    return ctrlPtValImplVirt(_funcIdx, _funcCtrlPtIdx, _ctrlPtDim);
  }
  ///@brief Const of a parametric function control point dimension.
public:
  const TNumType& ctrlPtVal(const std::size_t& _funcIdx, const std::size_t& _funcCtrlPtIdx,
                            const CtrlPtDim& _ctrlPtDim = CtrlPtDim::VAL) const
  {
    return ctrlPtValImplVirt(_funcIdx, _funcCtrlPtIdx, _ctrlPtDim);
  }

public:
  void shiftStartCtrlPt(const TNumType& _dt)
  {
    shiftStartCtrlPtImplVirt(_dt);
  }

  ///@brief Precomputes cached data.
public:
  void precompute()
  {
    precomputeImplVirt();
  }
  ///@brief Sets parametric function evaluation arc.
public:
  void setEvalArc(const TNumType& _arcEval, const eAG& _eAG = eAG::NONE)
  {
    setEvalArcImplVirt(_arcEval, _eAG);
  }
  ///@brief Computes value of parametric function with index @ref \_funcIdx at parametric arc set by @ref setEvalArc.
public:
  TNumType computeFuncVal(const std::size_t& _funcIdx) const
  {
    return computeFuncValImplVirt(_funcIdx);
  }
  ///@brief Computes 1st derivative of parametric function with index @ref \_funcIdx at parametric arc set by @ref
  ///setEvalArc.
public:
  TNumType computeFuncDiff1(const std::size_t& _funcIdx) const
  {
    return computeFuncDiff1ImplVirt(_funcIdx);
  }
  ///@brief Computes 2nd derivative of parametric function with index @ref \_funcIdx at parametric arc set by @ref
  ///setEvalArc.
public:
  TNumType computeFuncDiff2(const std::size_t& _funcIdx) const
  {
    return computeFuncDiff2ImplVirt(_funcIdx);
  }
  ///@brief Computes integral of parametric function with index @ref \_funcIdx on interval [@ref funcsArcBegin\_, @ref
  ///funcsArcEnd\_].
public:
  TNumType computeFuncInt1(const std::size_t& _funcIdx) const
  {
    return computeFuncInt1ImplVirt(_funcIdx);
  }
  ///@brief Computes double integral of parametric function with index @ref \_funcIdx on interval [@ref funcsArcBegin\_,
  ///@ref funcsArcEnd\_].
public:
  TNumType computeFuncInt2(const std::size_t& _funcIdx) const
  {
    return computeFuncInt2ImplVirt(_funcIdx);
  }

  // pure virtual functions
private:
  virtual void initImplVirt(const std::vector<ParamFuncsStructure>& _paramFuncsStructure) = 0;

private:
  virtual std::size_t funcsSizeImplVirt() const = 0;

private:
  virtual std::size_t funcsArcSizeImplVirt() const = 0;

private:
  virtual std::size_t funcsArcSizeImplVirt(const std::size_t& _i) const = 0;

private:
  virtual std::size_t funcCtrlPtSizeImplVirt(const std::size_t& _i) const = 0;

private:
  virtual TNumType& funcsArcBeginImplVirt() = 0;

private:
  virtual const TNumType& funcsArcBeginImplVirt() const = 0;

private:
  virtual TNumType& funcsArcEndImplVirt() = 0;

private:
  virtual const TNumType& funcsArcEndImplVirt() const = 0;

private:
  virtual const TNumType& funcsArcEvalImplVirt() const = 0;

private:
  virtual TNumType& funcsArcImplVirt(const std::size_t& _i, const std::size_t& _j) = 0;

private:
  virtual const TNumType& funcsArcImplVirt(const std::size_t& _i, const std::size_t& _j) const = 0;

private:
  virtual FuncCtrlPtType& ctrlPtImplVirt(const std::size_t& _funcIdx, const std::size_t& _funcCtrlPtIdx) = 0;

private:
  virtual const FuncCtrlPtType& ctrlPtImplVirt(const std::size_t& _funcIdx,
                                               const std::size_t& _funcCtrlPtIdx) const = 0;

private:
  virtual TNumType& ctrlPtValImplVirt(const std::size_t& _funcIdx, const std::size_t& _funcCtrlPtIdx,
                                      const CtrlPtDim& _ctrlPtDim) = 0;

private:
  virtual const TNumType& ctrlPtValImplVirt(const std::size_t& _funcIdx, const std::size_t& _funcCtrlPtIdx,
                                            const CtrlPtDim& _ctrlPtDim) const = 0;

private:
  virtual void shiftStartCtrlPtImplVirt(const TNumType& _dt) = 0;

private:
  virtual void precomputeImplVirt() = 0;

private:
  virtual void setEvalArcImplVirt(const TNumType& _arcEval, const eAG& _eAG) = 0;

private:
  virtual TNumType computeFuncValImplVirt(const std::size_t& _funcIdx) const = 0;

private:
  virtual TNumType computeFuncDiff1ImplVirt(const std::size_t& _funcIdx) const = 0;

private:
  virtual TNumType computeFuncDiff2ImplVirt(const std::size_t& _funcIdx) const = 0;

private:
  virtual TNumType computeFuncInt1ImplVirt(const std::size_t& _funcIdx) const = 0;

private:
  virtual TNumType computeFuncInt2ImplVirt(const std::size_t& _funcIdx) const = 0;
};

template <typename TNumType>
struct ParamFuncsBaseArcVarsDyn
{
  ///@brief Stores the control points arc parameters for all the parametrized functions.
protected:
  std::vector<std::vector<TNumType>> funcCtrlPtArc_;
};
template <typename TNumType, size_t TArcLatticeSize>
struct ParamFuncsBaseArcVarsStatic
{
  ///@brief Stores the control points arc parameters for all the parametrized functions.
protected:
  std::array<std::vector<TNumType>, TArcLatticeSize> funcCtrlPtArc_;
};

template <typename TNumType>
struct ParamFuncsBaseFuncVarsDyn
{
  ///@brief Flags for required function computation modes.
protected:
  std::vector<FuncEvalModesFlags> funcEvalReq_;
  ///@brief Stores the control points for all the parametrized functions.
protected:
  std::vector<std::vector<FuncCtrlPt<TNumType>>> funcCtrlPt_;
  ///@brief Initialization structure "store" variable
protected:
  std::vector<ParamFuncsStructure> paramFuncsStructure_;
  ///@brief Maps the parametrized functions to their afferent arc parametrizations.
protected:
  std::vector<std::size_t> func2Arc_;
};
template <typename TNumType, size_t TFuncSize>
struct ParamFuncsBaseFuncVarsStatic
{
  ///@brief Flags for required function computation modes.
protected:
  std::array<FuncEvalModesFlags, TFuncSize> funcEvalReq_;
  ///@brief Stores the control points for all the parametrized functions.
protected:
  std::array<std::vector<FuncCtrlPt<TNumType>>, TFuncSize> funcCtrlPt_;
  ///@brief Initialization structure "store" variable
protected:
  std::array<ParamFuncsStructure, TFuncSize> paramFuncsStructure_;
  ///@brief Maps the parametrized functions to their afferent arc parametrizations.
protected:
  std::array<std::size_t, TFuncSize> func2Arc_;
};

template <typename TDerived, typename TNumType, int TFuncSize, int TArcLatticeSize>
class ParamFuncsBase : public ParamFuncsBaseCRTP<ParamFuncsBase<TDerived, TNumType, TFuncSize, TArcLatticeSize>>,
                       public ParamFuncsBaseVirt<TNumType>,
                       public std::conditional<TFuncSize == -1, ParamFuncsBaseFuncVarsDyn<TNumType>,
                                               ParamFuncsBaseFuncVarsStatic<TNumType, TFuncSize>>::type,
                       public std::conditional<TArcLatticeSize == -1, ParamFuncsBaseArcVarsDyn<TNumType>,
                                               ParamFuncsBaseArcVarsStatic<TNumType, TArcLatticeSize>>::type
{
public:
  using NumType = TNumType;

public:
  using ParamFuncsBaseType = ParamFuncsBase<TDerived, TNumType, TFuncSize, TArcLatticeSize>;

public:
  static constexpr const int FuncSize = TFuncSize;

private:
  static constexpr const bool IsFuncDyn = TFuncSize == -1;

private:
  static constexpr const bool IsArcDyn = TArcLatticeSize == -1;

protected:
  using FuncCtrlPtType = FuncCtrlPt<TNumType>;

  // special class member functions
public:
  ParamFuncsBase() : funcsArcEval_(-1)
  {
  }

public:
  virtual ~ParamFuncsBase() = default;

public:
  ParamFuncsBase(const ParamFuncsBase& _other)
  {
    using PfCpD = CtrlPtDim;
    initBase(_other.paramFuncsStructure_);
    for (size_t i = 0; i < funcsArcSize(); ++i)
    {
      for (size_t j = 0; j < funcsArcSize(i); ++j)
      {
        funcsArc(i, j) = _other.funcsArc(i, j);
      }
    }
    for (size_t i = 0; i < funcsSize(); ++i)
    {
      for (size_t j = 0; j < funcCtrlPtSize(i); ++j)
      {
        ctrlPtVal(i, j, PfCpD::VAL) = _other.ctrlPtVal(i, j, PfCpD::VAL);
      }
    }
  }

public:
  ParamFuncsBase& operator=(const ParamFuncsBase& _other)
  {
    if (this == &_other)
    {
      return *this;
    }

    using PfCpD = CtrlPtDim;
    init(_other.paramFuncsStructure_);
    for (size_t i = 0; i < funcsArcSize(); ++i)
    {
      for (size_t j = 0; j < funcsArcSize(i); ++j)
      {
        funcsArc(i, j) = _other.funcsArc(i, j);
      }
    }
    for (size_t i = 0; i < funcsSize(); ++i)
    {
      for (size_t j = 0; j < funcCtrlPtSize(i); ++j)
      {
        ctrlPtVal(i, j, PfCpD::VAL) = _other.ctrlPtVal(i, j, PfCpD::VAL);
      }
    }
    return *this;
  }

public:
  ParamFuncsBase(ParamFuncsBase&&) = default;

public:
  ParamFuncsBase& operator=(ParamFuncsBase&&) = default;

  using ParamFuncsBaseCRTP<ParamFuncsBaseType>::init;
  using ParamFuncsBaseCRTP<ParamFuncsBaseType>::funcsSize;
  using ParamFuncsBaseCRTP<ParamFuncsBaseType>::funcsArcSize;
  using ParamFuncsBaseCRTP<ParamFuncsBaseType>::funcCtrlPtSize;
  using ParamFuncsBaseCRTP<ParamFuncsBaseType>::funcsArcBegin;
  using ParamFuncsBaseCRTP<ParamFuncsBaseType>::funcsArcEnd;
  using ParamFuncsBaseCRTP<ParamFuncsBaseType>::funcsArcEval;
  using ParamFuncsBaseCRTP<ParamFuncsBaseType>::funcsArc;
  using ParamFuncsBaseCRTP<ParamFuncsBaseType>::ctrlPt;
  using ParamFuncsBaseCRTP<ParamFuncsBaseType>::ctrlPtVal;
  using ParamFuncsBaseCRTP<ParamFuncsBaseType>::shiftStartCtrlPt;
  using ParamFuncsBaseCRTP<ParamFuncsBaseType>::precompute;
  using ParamFuncsBaseCRTP<ParamFuncsBaseType>::setEvalArc;
  using ParamFuncsBaseCRTP<ParamFuncsBaseType>::computeFuncVal;
  using ParamFuncsBaseCRTP<ParamFuncsBaseType>::computeFuncDiff1;
  using ParamFuncsBaseCRTP<ParamFuncsBaseType>::computeFuncDiff2;
  using ParamFuncsBaseCRTP<ParamFuncsBaseType>::computeFuncInt1;
  using ParamFuncsBaseCRTP<ParamFuncsBaseType>::computeFuncInt2;
  ///@brief Called at end of @ref init. To be used by extended classes.
protected:
  void initImpl()
  {
    thisDerived().initImplImpl();
  }

private:
  void precomputeImplVirt() override final
  {
    thisDerived().precomputeImpl();
  }

private:
  void setEvalArcImplVirt(const TNumType& _arcEval,
                          const typename ParamFuncsBaseCRTP<ParamFuncsBaseType>::eAG& _eAG) override final
  {
    thisDerived().setEvalArcImpl(_arcEval, _eAG);
  }

private:
  TNumType computeFuncValImplVirt(const std::size_t& _funcIdx) const override final
  {
    return thisDerived().computeFuncValImpl(_funcIdx);
  }

private:
  TNumType computeFuncDiff1ImplVirt(const std::size_t& _funcIdx) const override final
  {
    return thisDerived().computeFuncDiff1Impl(_funcIdx);
  }

private:
  TNumType computeFuncDiff2ImplVirt(const std::size_t& _funcIdx) const override final
  {
    return thisDerived().computeFuncDiff2Impl(_funcIdx);
  }

private:
  TNumType computeFuncInt1ImplVirt(const std::size_t& _funcIdx) const override final
  {
    return thisDerived().computeFuncInt1Impl(_funcIdx);
  }

private:
  TNumType computeFuncInt2ImplVirt(const std::size_t& _funcIdx) const override final
  {
    return thisDerived().computeFuncInt2Impl(_funcIdx);
  }

private:
  void precomputeImplCRTP()
  {
    thisDerived().precomputeImpl();
  }

private:
  void setEvalArcImplCRTP(const TNumType& _arcEval, const typename ParamFuncsBaseCRTP<ParamFuncsBaseType>::eAG& _eAG)
  {
    thisDerived().setEvalArcImpl(_arcEval, _eAG);
  }

private:
  TNumType computeFuncValImplCRTP(const std::size_t& _funcIdx) const
  {
    return thisDerived().computeFuncValImpl(_funcIdx);
  }

private:
  TNumType computeFuncDiff1ImplCRTP(const std::size_t& _funcIdx) const
  {
    return thisDerived().computeFuncDiff1Impl(_funcIdx);
  }

private:
  TNumType computeFuncDiff2ImplCRTP(const std::size_t& _funcIdx) const
  {
    return thisDerived().computeFuncDiff2Impl(_funcIdx);
  }

private:
  TNumType computeFuncInt1ImplCRTP(const std::size_t& _funcIdx) const
  {
    return thisDerived().computeFuncInt1Impl(_funcIdx);
  }

private:
  TNumType computeFuncInt2ImplCRTP(const std::size_t& _funcIdx) const
  {
    return thisDerived().computeFuncInt2Impl(_funcIdx);
  }

private:
  template <bool FuncDyn = IsFuncDyn, typename std::enable_if<FuncDyn>::type* = nullptr>
  void initImplVirtDispatch(const std::vector<ParamFuncsStructure>& _paramFuncsStructure)
  {
    initImplCRTP(_paramFuncsStructure);
  }

private:
  template <bool FuncDyn = IsFuncDyn, typename std::enable_if<!FuncDyn>::type* = nullptr>
  void initImplVirtDispatch(const std::vector<ParamFuncsStructure>& _paramFuncsStructure)
  {
    if (_paramFuncsStructure.size() != TFuncSize)
    {
      throw std::runtime_error("Error in ParamFuncs::init: number of functions in init structure does not match "
                               "(static) number of param funcs");
    }
    std::array<ParamFuncsStructure, TFuncSize> pfsStatic;
    for (size_t i = 0; i < pfsStatic.size(); ++i)
    {
      pfsStatic[i] = _paramFuncsStructure[i];
    }
    initImplCRTP(pfsStatic);
  }

private:
  void initImplVirt(const std::vector<ParamFuncsStructure>& _paramFuncsStructure) override final
  {
    initImplVirtDispatch(_paramFuncsStructure);
  }

private:
  std::size_t funcsSizeImplVirt() const override final
  {
    return funcsSizeImplCRTP();
  }

private:
  std::size_t funcsArcSizeImplVirt() const override final
  {
    return funcsArcSizeImplCRTP();
  }

private:
  std::size_t funcsArcSizeImplVirt(const std::size_t& _i) const override final
  {
    return funcsArcSizeImplCRTP(_i);
  }

private:
  std::size_t funcCtrlPtSizeImplVirt(const std::size_t& _i) const override final
  {
    return funcCtrlPtSizeImplCRTP(_i);
  }

private:
  TNumType& funcsArcBeginImplVirt() override final
  {
    return funcsArcBeginImplCRTP();
  }

private:
  const TNumType& funcsArcBeginImplVirt() const override final
  {
    return funcsArcBeginImplCRTP();
  }

private:
  TNumType& funcsArcEndImplVirt() override final
  {
    return funcsArcEndImplCRTP();
  }

private:
  const TNumType& funcsArcEndImplVirt() const override final
  {
    return funcsArcEndImplCRTP();
  }

private:
  const TNumType& funcsArcEvalImplVirt() const override final
  {
    return funcsArcEvalImplCRTP();
  }

private:
  TNumType& funcsArcImplVirt(const std::size_t& _i, const std::size_t& _j) override final
  {
    return funcsArcImplCRTP(_i, _j);
  }

private:
  const TNumType& funcsArcImplVirt(const std::size_t& _i, const std::size_t& _j) const override final
  {
    return funcsArcImplCRTP(_i, _j);
  }

private:
  FuncCtrlPtType& ctrlPtImplVirt(const std::size_t& _funcIdx, const std::size_t& _funcCtrlPtIdx) override final
  {
    return ctrlPtImplCRTP(_funcIdx, _funcCtrlPtIdx);
  }

private:
  const FuncCtrlPtType& ctrlPtImplVirt(const std::size_t& _funcIdx,
                                       const std::size_t& _funcCtrlPtIdx) const override final
  {
    return ctrlPtImplCRTP(_funcIdx, _funcCtrlPtIdx);
  }

private:
  TNumType& ctrlPtValImplVirt(const std::size_t& _funcIdx, const std::size_t& _funcCtrlPtIdx,
                              const CtrlPtDim& _ctrlPtDim) override final
  {
    return ctrlPtValImplCRTP(_funcIdx, _funcCtrlPtIdx, _ctrlPtDim);
  }

private:
  const TNumType& ctrlPtValImplVirt(const std::size_t& _funcIdx, const std::size_t& _funcCtrlPtIdx,
                                    const CtrlPtDim& _ctrlPtDim) const override final
  {
    return ctrlPtValImplCRTP(_funcIdx, _funcCtrlPtIdx, _ctrlPtDim);
  }

private:
  void shiftStartCtrlPtImplVirt(const TNumType& _dt) override final
  {
    shiftStartCtrlPtImplCRTP(_dt);
  }

private:
  template <bool FuncDyn = (IsFuncDyn), bool ArcDyn = IsArcDyn,
            typename std::enable_if<(FuncDyn) && (ArcDyn)>::type* = nullptr>
  void initBase(const std::vector<ParamFuncsStructure>& _paramFuncsStructure)
  {
    this->paramFuncsStructure_ = _paramFuncsStructure;
    const size_t funcsSize = _paramFuncsStructure.size();
    if (funcsSize < 1)
    {
      throw std::runtime_error("Error in ParamFuncs::checkParamFuncsStructure: initialization with 0 number of "
                               "functions!");
    }
    this->funcCtrlPt_.resize(funcsSize);
    for (auto& funcCtrlPtrI : this->funcCtrlPt_)
    {
      funcCtrlPtrI.clear();
    }
    this->func2Arc_.resize(funcsSize);
    this->funcEvalReq_.resize(funcsSize);
    this->funcCtrlPtArc_.clear();
    std::vector<size_t> funcSameArcCtrlPtSize;
    for (size_t i = 0; i < funcsSize; ++i)
    {
      const auto& paramFuncsStructureI = _paramFuncsStructure[i];
      const size_t& funcCtrlPtsSize = paramFuncsStructureI.ctrlPtsSize;
      if (funcCtrlPtsSize < 2)
      {
        throw std::runtime_error("Error in ParamFuncs::checkParamFuncsStructure: function has less than 2 control "
                                 "points!");
      }
      auto& func2ArcI = this->func2Arc_[i];
      func2ArcI = paramFuncsStructureI.ctrlPtsArcRefIdx;
      if (func2ArcI >= this->funcCtrlPtArc_.size())
      {
        if (func2ArcI == this->funcCtrlPtArc_.size())
        {
          this->funcCtrlPtArc_.emplace_back(std::vector<TNumType>(funcCtrlPtsSize - 2, 0));
          funcSameArcCtrlPtSize.emplace_back(funcCtrlPtsSize);
        }
        else
        {
          throw std::runtime_error("Error in ParamFuncs::init: a ctrlPtsArcRefIdx is not consistent (new Idx not unit "
                                   "incremental)");
        }
      }
      if (funcCtrlPtsSize != funcSameArcCtrlPtSize[func2ArcI])
      {
        throw "Error in ParamFuncs::init: inconsistent number of control points for functions with same arc "
              "parametrization reference";
      }
      auto& funcCtrlPtI = this->funcCtrlPt_[i];
      funcCtrlPtI.reserve(funcCtrlPtsSize);
      funcCtrlPtI.emplace_back(FuncCtrlPtType(0, funcsArcBegin_));
      for (size_t j = 1; j < funcCtrlPtsSize - 1; ++j)
      {
        funcCtrlPtI.emplace_back(FuncCtrlPtType(0, this->funcCtrlPtArc_[func2ArcI][j - 1]));
      }
      funcCtrlPtI.emplace_back(FuncCtrlPtType(0, funcsArcEnd_));

      this->funcEvalReq_[i] = paramFuncsStructureI.evalReq;
    }
  }

private:
  template <bool FuncDyn = (IsFuncDyn), bool ArcDyn = IsArcDyn,
            typename std::enable_if<(!FuncDyn) && (ArcDyn)>::type* = nullptr>
  void initBase(const std::array<ParamFuncsStructure, TFuncSize>& _paramFuncsStructure)
  {
    this->paramFuncsStructure_ = _paramFuncsStructure;
    for (auto& funcCtrlPtrI : this->funcCtrlPt_)
    {
      funcCtrlPtrI.clear();
    }
    this->funcCtrlPtArc_.clear();
    std::vector<size_t> funcSameArcCtrlPtSize;
    for (size_t i = 0; i < (size_t)TFuncSize; ++i)
    {
      const auto& paramFuncsStructureI = _paramFuncsStructure[i];
      const size_t& funcCtrlPtsSize = paramFuncsStructureI.ctrlPtsSize;
      if (funcCtrlPtsSize < 2)
      {
        throw std::runtime_error("Error in ParamFuncs::checkParamFuncsStructure: function has less than 2 control "
                                 "points!");
      }
      auto& func2ArcI = this->func2Arc_[i];
      func2ArcI = paramFuncsStructureI.ctrlPtsArcRefIdx;
      if (func2ArcI >= this->funcCtrlPtArc_.size())
      {
        if (func2ArcI == this->funcCtrlPtArc_.size())
        {
          this->funcCtrlPtArc_.emplace_back(std::vector<TNumType>(funcCtrlPtsSize - 2, 0));
          funcSameArcCtrlPtSize.emplace_back(funcCtrlPtsSize);
        }
        else
        {
          throw std::runtime_error("Error in ParamFuncs::init: a ctrlPtsArcRefIdx is not consistent (new Idx not unit "
                                   "incremental)");
        }
      }
      if (funcCtrlPtsSize != funcSameArcCtrlPtSize[func2ArcI])
      {
        throw "Error in ParamFuncs::init: inconsistent number of control points for functions with same arc "
              "parametrization reference";
      }
      auto& funcCtrlPtI = this->funcCtrlPt_[i];
      funcCtrlPtI.reserve(funcCtrlPtsSize);
      funcCtrlPtI.emplace_back(FuncCtrlPtType(0, funcsArcBegin_));
      for (size_t j = 1; j < funcCtrlPtsSize - 1; ++j)
      {
        funcCtrlPtI.emplace_back(FuncCtrlPtType(0, this->funcCtrlPtArc_[func2ArcI][j - 1]));
      }
      funcCtrlPtI.emplace_back(FuncCtrlPtType(0, funcsArcEnd_));

      this->funcEvalReq_[i] = paramFuncsStructureI.evalReq;
    }
  }

private:
  template <bool FuncDyn = (IsFuncDyn), bool ArcDyn = IsArcDyn,
            typename std::enable_if<(FuncDyn) && (!ArcDyn)>::type* = nullptr>
  void initBase(const std::vector<ParamFuncsStructure>& _paramFuncsStructure)
  {
    this->paramFuncsStructure_ = _paramFuncsStructure;
    const size_t funcsSize = _paramFuncsStructure.size();
    if (funcsSize < 1)
    {
      throw std::runtime_error("Error in ParamFuncs::checkParamFuncsStructure: initialization with 0 number of "
                               "functions!");
    }
    this->funcCtrlPt_.resize(funcsSize);
    for (auto& funcCtrlPtrI : this->funcCtrlPt_)
    {
      funcCtrlPtrI.clear();
    }
    this->func2Arc_.resize(funcsSize);
    this->funcEvalReq_.resize(funcsSize);
    std::array<size_t, TFuncSize> funcSameArcCtrlPtSize;
    std::vector<bool> initFuncCtrlPtArc(this->funcCtrlPtArc_.size(), false);
    for (size_t i = 0; i < funcsSize; ++i)
    {
      const auto& paramFuncsStructureI = _paramFuncsStructure[i];
      const size_t& funcCtrlPtsSize = paramFuncsStructureI.ctrlPtsSize;
      if (funcCtrlPtsSize < 2)
      {
        throw std::runtime_error("Error in ParamFuncs::checkParamFuncsStructure: function has less than 2 control "
                                 "points!");
      }
      auto& func2ArcI = this->func2Arc_[i];
      func2ArcI = paramFuncsStructureI.ctrlPtsArcRefIdx;
      if (func2ArcI < this->funcCtrlPtArc_.size())
      {
        if (initFuncCtrlPtArc[func2ArcI] == false)
        {
          this->funcCtrlPtArc_[func2ArcI] = std::vector<TNumType>(funcCtrlPtsSize - 2, 0);
          initFuncCtrlPtArc[func2ArcI] = true;
        }
        funcSameArcCtrlPtSize[func2ArcI] = funcCtrlPtsSize;
      }
      else
      {
        throw std::runtime_error("Error in ParamFuncs::init: a ctrlPtsArcRefIdx is not consistent (>= than the static "
                                 "arc lattice size)");
      }
      if (funcCtrlPtsSize != funcSameArcCtrlPtSize[func2ArcI])
      {
        throw "Error in ParamFuncs::init: inconsistent number of control points for functions with same arc "
              "parametrization reference";
      }
      auto& funcCtrlPtI = this->funcCtrlPt_[i];
      funcCtrlPtI.reserve(funcCtrlPtsSize);
      funcCtrlPtI.emplace_back(FuncCtrlPtType(0, funcsArcBegin_));
      for (size_t j = 1; j < funcCtrlPtsSize - 1; ++j)
      {
        funcCtrlPtI.emplace_back(FuncCtrlPtType(0, this->funcCtrlPtArc_[func2ArcI][j - 1]));
      }
      funcCtrlPtI.emplace_back(FuncCtrlPtType(0, funcsArcEnd_));

      this->funcEvalReq_[i] = paramFuncsStructureI.evalReq;
    }
  }

private:
  template <bool FuncDyn = (IsFuncDyn), bool ArcDyn = IsArcDyn,
            typename std::enable_if<(!FuncDyn) && (!ArcDyn)>::type* = nullptr>
  void initBase(const std::array<ParamFuncsStructure, TFuncSize>& _paramFuncsStructure)
  {
    this->paramFuncsStructure_ = _paramFuncsStructure;
    for (auto& funcCtrlPtrI : this->funcCtrlPt_)
    {
      funcCtrlPtrI.clear();
    }
    std::array<size_t, TFuncSize> funcSameArcCtrlPtSize;
    std::vector<bool> initFuncCtrlPtArc(this->funcCtrlPtArc_.size(), false);
    for (size_t i = 0; i < (size_t)TFuncSize; ++i)
    {
      const auto& paramFuncsStructureI = _paramFuncsStructure[i];
      const size_t& funcCtrlPtsSize = paramFuncsStructureI.ctrlPtsSize;
      if (funcCtrlPtsSize < 2)
      {
        throw std::runtime_error("Error in ParamFuncs::checkParamFuncsStructure: function has less than 2 control "
                                 "points!");
      }
      auto& func2ArcI = this->func2Arc_[i];
      func2ArcI = paramFuncsStructureI.ctrlPtsArcRefIdx;
      if (func2ArcI < this->funcCtrlPtArc_.size())
      {
        if (initFuncCtrlPtArc[func2ArcI] == false)
        {
          this->funcCtrlPtArc_[func2ArcI] = std::vector<TNumType>(funcCtrlPtsSize - 2, 0);
          initFuncCtrlPtArc[func2ArcI] = true;
        }
        funcSameArcCtrlPtSize[func2ArcI] = funcCtrlPtsSize;
      }
      else
      {
        throw std::runtime_error("Error in ParamFuncs::init: a ctrlPtsArcRefIdx is not consistent (>= than the static "
                                 "arc lattice size)");
      }
      if (funcCtrlPtsSize != funcSameArcCtrlPtSize[func2ArcI])
      {
        throw std::runtime_error("Error in ParamFuncs::init: inconsistent number of control points for functions with "
                                 "same arc parametrization reference");
      }
      auto& funcCtrlPtI = this->funcCtrlPt_[i];
      funcCtrlPtI.reserve(funcCtrlPtsSize);
      funcCtrlPtI.emplace_back(FuncCtrlPtType(0, funcsArcBegin_));
      for (size_t j = 1; j < funcCtrlPtsSize - 1; ++j)
      {
        funcCtrlPtI.emplace_back(FuncCtrlPtType(0, this->funcCtrlPtArc_[func2ArcI][j - 1]));
      }
      funcCtrlPtI.emplace_back(FuncCtrlPtType(0, funcsArcEnd_));

      this->funcEvalReq_[i] = paramFuncsStructureI.evalReq;
    }
  }

private:
  template <typename TParamFucsStructVecArr>
  void initImplCRTP(const TParamFucsStructVecArr& _paramFuncsStructure)
  {
    initBase(_paramFuncsStructure);
    initImpl();
  }

private:
  std::size_t funcsSizeImplCRTP() const
  {
    return this->funcCtrlPt_.size();
  }

private:
  std::size_t funcsArcSizeImplCRTP() const
  {
    return this->funcCtrlPtArc_.size();
  }

private:
  std::size_t funcsArcSizeImplCRTP(const std::size_t& _i) const
  {
    return this->funcCtrlPtArc_[_i].size() + 2;
  }

private:
  std::size_t funcCtrlPtSizeImplCRTP(const std::size_t& _i) const
  {
    return this->funcCtrlPt_[_i].size();
  }

private:
  TNumType& funcsArcBeginImplCRTP()
  {
    return funcsArcBegin_;
  }

private:
  const TNumType& funcsArcBeginImplCRTP() const
  {
    return funcsArcBegin_;
  }

private:
  TNumType& funcsArcEndImplCRTP()
  {
    return funcsArcEnd_;
  }

private:
  const TNumType& funcsArcEndImplCRTP() const
  {
    return funcsArcEnd_;
  }

private:
  const TNumType& funcsArcEvalImplCRTP() const
  {
    return funcsArcEval_;
  }

private:
  TNumType& funcsArcImplCRTP(const std::size_t& _i, const std::size_t& _j)
  {
    if (_j == 0)
    {
      return funcsArcBegin_;
    }
    if (_j > this->funcCtrlPtArc_[_i].size())
    {
      return funcsArcEnd_;
    }
    return this->funcCtrlPtArc_[_i][_j - 1];
  }

private:
  const TNumType& funcsArcImplCRTP(const std::size_t& _i, const std::size_t& _j) const
  {
    if (_j == 0)
    {
      return funcsArcBegin_;
    }
    if (_j > this->funcCtrlPtArc_[_i].size())
    {
      return funcsArcEnd_;
    }
    return this->funcCtrlPtArc_[_i][_j - 1];
  }

private:
  FuncCtrlPtType& ctrlPtImplCRTP(const std::size_t& _funcIdx, const std::size_t& _funcCtrlPtIdx)
  {
    return this->funcCtrlPt_[_funcIdx][_funcCtrlPtIdx];
  }

private:
  const FuncCtrlPtType& ctrlPtImplCRTP(const std::size_t& _funcIdx, const std::size_t& _funcCtrlPtIdx) const
  {
    return this->funcCtrlPt_[_funcIdx][_funcCtrlPtIdx];
  }

private:
  TNumType& ctrlPtValImplCRTP(const std::size_t& _funcIdx, const std::size_t& _funcCtrlPtIdx,
                              const CtrlPtDim& _ctrlPtDim)
  {
    switch (_ctrlPtDim)
    {
      case CtrlPtDim::VAL:
        return this->funcCtrlPt_[_funcIdx][_funcCtrlPtIdx].val;
        break;
      case CtrlPtDim::ARC:
        return this->funcCtrlPt_[_funcIdx][_funcCtrlPtIdx].arc;
        break;
      default:
        throw "CtrlPtDim unrecognised in function ParamFuncs::ctrlPtVal !";
        break;
    }
  }

private:
  const TNumType& ctrlPtValImplCRTP(const std::size_t& _funcIdx, const std::size_t& _funcCtrlPtIdx,
                                    const CtrlPtDim& _ctrlPtDim) const
  {
    switch (_ctrlPtDim)
    {
      case CtrlPtDim::VAL:
        return this->funcCtrlPt_[_funcIdx][_funcCtrlPtIdx].val;
        break;
      case CtrlPtDim::ARC:
        return this->funcCtrlPt_[_funcIdx][_funcCtrlPtIdx].arc;
        break;
      default:
        throw "CtrlPtDim unrecognised in function ParamFuncs::ctrlPtVal !";
        break;
    }
  }

private:
  void shiftStartCtrlPtImplCRTP(const TNumType& _dt)
  {
    std::vector<TNumType> ctrlPtValPref(funcsSize(), 0);
    const TNumType dtBound = fmin(fmax(funcsArcBegin_, _dt), funcsArcEnd_ - funcsArcBegin_);
    const TNumType evalArc = funcsArcBegin_ + dtBound;
    setEvalArc(evalArc);
    for (size_t i = 0; i < funcsSize(); ++i)
    {
      ctrlPtValPref[i] = computeFuncVal(i);
    }
    for (size_t i = 0; i < funcsSize(); ++i)
    {
      for (size_t j = 0; j < funcCtrlPtSize(i); ++j)
      {
        if (ctrlPtVal(i, j, CtrlPtDim::ARC) < evalArc)
        {
          ctrlPtVal(i, j, CtrlPtDim::VAL) = ctrlPtValPref[i];
        }
        else
        {
          break;
        }
      }
    }
    for (size_t k = 0; k < funcsArcSize(); ++k)
    {
      for (size_t j = 0; j < this->funcCtrlPtArc_[k].size(); ++j)
      {
        this->funcCtrlPtArc_[k][j] = fmax(funcsArcBegin_, this->funcCtrlPtArc_[k][j] - dtBound);
      }
    }
    funcsArcEnd_ -= dtBound;
    precompute();
    setEvalArc(funcsArcBegin_);
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

  ///@brief Arc parametrization at the beginning of the functions domain definitions (common for all functions).
protected:
  TNumType funcsArcBegin_;
  ///@brief Arc parametrization at the end of the functions domain definitions (common for all functions).
protected:
  TNumType funcsArcEnd_;
  ///@brief Arc parametrization at the evaluation point (set by @ref setEvalArc)/
protected:
  TNumType funcsArcEval_;

  template <typename TDerived2>
  friend class ParamFuncsBaseCRTP;
  template <typename TNum2>
  friend class ParamFuncsBaseVirt;
};
template <typename TDerived, typename TNumType, int TFuncSize, int TArcLatticeSize>
constexpr const int ParamFuncsBase<TDerived, TNumType, TFuncSize, TArcLatticeSize>::FuncSize;

namespace
{
template <typename TDerived, typename TNumType, int TFuncSize, int TArcLatticeSize>
struct ParamFuncsBaseCRTPTraits<ParamFuncsBase<TDerived, TNumType, TFuncSize, TArcLatticeSize>>
{
  using NumType = TNumType;
};

}  // namespace <anonymous>
}

#endif  // PARAM_FUNC_HPP
