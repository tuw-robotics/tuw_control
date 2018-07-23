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

#ifndef PARAM_FUNC_H
#define PARAM_FUNC_H

#include <memory>
#include <vector>
#include <map>
#include <tuw_control/utils.h>

namespace tuw
{
/*!@class ParamFuncs
 * @brief Storage and manipulation of parametric functions collection
 *
 * @todo Making rules for unconsistent arcs (works only with nondecreasing function arcs) and document what the
 *functions assume before and stuff.
 * @todo Making rules for initial conditions (either tBegin should be allways 0 or something)
 * @todo cout the control points and their arc parametrizations
 *
 */
class ParamFuncs;
using ParamFuncsUPtr = std::unique_ptr<ParamFuncs>;
using ParamFuncsConstUPtr = std::unique_ptr<ParamFuncs const>;
using ParamFuncsSPtr = std::shared_ptr<ParamFuncs>;
using ParamFuncsConstSPtr = std::shared_ptr<ParamFuncs const>;
class ParamFuncs
{
  // enums
  ///@brief Required type of computation relative to the parametric function.
public:
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
public:
  enum class CtrlPtDim
  {
    VAL,  ///<control point value
    ARC   ///<control point arc parameter
  };
  ///@brief Flags if any guarantees about evaluation arc relative to last evaluation arc are present.
public:
  enum class EvalArcGuarantee
  {
    NONE,
    AFTER_LAST,   ///<previous evaluation arc <= this evaluation arc
    BEFORE_LAST,  ///<previous evaluation arc >= this evaluation arc
    AT_BEGIN,     ///<this evaluation arc is at the arc parametrization begin
    AT_END        ///<this evaluation arc is at the arc parametrization end
  };

  ///@brief Maximum number of computation modes (except the parametric function itself @ref FUNC). Currently 4 other
  ///choices supported. @see FuncEvalMode.
public:
  static constexpr const std::size_t nrModesMax_ = asInt(FuncEvalMode::ENUM_SIZE) - 1;

public:
  using FuncEvalModesFlags = std::array<bool, nrModesMax_>;

  // structs
  ///@brief Containts parametric function initialization data.
public:
  struct ParamFuncsStructure
  {
    ParamFuncsStructure() : ctrlPtsSize(0), ctrlPtsArcRefIdx(0), evalReq()
    {
      evalReq.fill(false);
    }
    std::size_t ctrlPtsSize;  ///<number of parametric function control points   ( @ref ctrlPtsSize > max(BSpline order,
                              ///2) ! )
    std::size_t ctrlPtsArcRefIdx;  ///<arc parameter structure index used by the parametric function (if two functions
                                   ///f1, f2 share the same arc index, @ref ctrlPtsSize(f1) has to be equal to @ref
                                   ///ctrlPtsSize(f2)!
    FuncEvalModesFlags evalReq;    ///<flags for required function computation modes
  };
  ///@brief Control point variable.
protected:
  struct FuncCtrlPt
  {
    FuncCtrlPt(const double& _val, double& _arc) : val(_val), arc(_arc)
    {
    }
    double val;   ///<control point value
    double& arc;  ///<control point arc parameter
  };

  // special class member functions
public:
  ParamFuncs() = default;

public:
  virtual ~ParamFuncs() = default;

public:
  ParamFuncs(const ParamFuncs&);  // = default;
public:
  ParamFuncs& operator=(const ParamFuncs&);  // = default;
public:
  ParamFuncs(ParamFuncs&&) = delete;

public:
  ParamFuncs& operator=(ParamFuncs&&) = delete;

  /**
   * @brief Initializes the control structure for the base class.
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
private:
  void initBase(const std::vector<tuw::ParamFuncs::ParamFuncsStructure>& _paramFuncsStructure);
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
  void init(const std::vector<tuw::ParamFuncs::ParamFuncsStructure>& _paramFuncsStructure);
  ///@brief Number of parametric functions.
public:
  std::size_t funcsSize() const;
  ///@brief Number of arc parametrizations.
public:
  std::size_t funcsArcSize() const;
  ///@brief Number of control points of parametrization @ref \_i.
public:
  std::size_t funcsArcSize(const std::size_t& _i) const;
  ///@brief Number of control points for a parametric function.
public:
  std::size_t funcCtrlPtSize(const std::size_t& _i) const;
  ///@brief Access to the initial value of the arc parametrization.
public:
  double& funcsArcBegin();
  ///@brief Const access to the initial value of the arc parametrization.
public:
  const double& funcsArcBegin() const;
  ///@brief Access to the final value of the arc parametrization.
public:
  double& funcsArcEnd();
  ///@brief Const access to the finall value of the arc parametrization.
public:
  const double& funcsArcEnd() const;
  ///@brief Const access to the active evaluation point arc parametrization.
public:
  const double& funcsArcEval() const;
  ///@brief Access to the arc parameter vector at index @ref \_i, control point @ref \_j.
public:
  double& funcsArc(const std::size_t& _i, const std::size_t& _j);
  ///@brief Const access the arc parameter vector at index @ref \_i, control point @ref \_j.
public:
  const double& funcsArc(const std::size_t& _i, const std::size_t& _j) const;
  ///@brief Access of a parametric function control point.
public:
  FuncCtrlPt& ctrlPt(const std::size_t& _funcIdx, const std::size_t& _funcCtrlPtIdx);
  ///@brief Const access of a parametric function control point.
public:
  const FuncCtrlPt& ctrlPt(const std::size_t& _funcIdx, const std::size_t& _funcCtrlPtIdx) const;
  ///@brief Access of a parametric function control point dimension.
public:
  double& ctrlPtVal(const std::size_t& _funcIdx, const std::size_t& _funcCtrlPtIdx,
                    const CtrlPtDim& _ctrlPtDim = CtrlPtDim::VAL);
  ///@brief Const of a parametric function control point dimension.
public:
  const double& ctrlPtVal(const std::size_t& _funcIdx, const std::size_t& _funcCtrlPtIdx,
                          const CtrlPtDim& _ctrlPtDim = CtrlPtDim::VAL) const;

public:
  void shiftStartCtrlPt(const double& _dt);

  ///@brief Flags for required function computation modes.
protected:
  std::vector<FuncEvalModesFlags> funcEvalReq_;
  ///@brief Stores the control points for all the parametrized functions.
protected:
  std::vector<std::vector<FuncCtrlPt> > funcCtrlPt_;
  ///@brief Stores the control points arc parameters for all the parametrized functions.
private:
  std::vector<std::vector<double> > funcCtrlPtArc_;
  ///@brief Maps the parametrized functions to their afferent arc parametrizations.
protected:
  std::vector<std::size_t> func2Arc_;
  ///@brief Arc parametrization at the beginning of the functions domain definitions (common for all functions).
protected:
  double funcsArcBegin_;
  ///@brief Arc parametrization at the end of the functions domain definitions (common for all functions).
protected:
  double funcsArcEnd_;
  ///@brief Arc parametrization at the evaluation point (set by @ref setEvalArc)/
protected:
  double funcsArcEval_;
  ///@brief Initialization structure "store" variable
protected:
  std::vector<ParamFuncsStructure> paramFuncsStructure_;

  // pure virtual functions
  ///@brief Called at end of @ref init. To be used by extended classes.
protected:
  virtual void initImpl() = 0;
  ///@brief Precomputes cached data.
public:
  virtual void precompute() = 0;
  ///@brief Sets parametric function evaluation arc.
public:
  virtual void setEvalArc(const double& _funcsArcEval,
                          const EvalArcGuarantee& _evalArcGuarantee = EvalArcGuarantee::NONE) = 0;
  ///@brief Computes value of parametric function with index @ref \_funcIdx at parametric arc set by @ref setEvalArc.
public:
  virtual double computeFuncVal(const std::size_t& _funcIdx) const = 0;
  ///@brief Computes 1st derivative of parametric function with index @ref \_funcIdx at parametric arc set by @ref
  ///setEvalArc.
public:
  virtual double computeFuncDiff1(const std::size_t& _funcIdx) const = 0;
  ///@brief Computes 2nd derivative of parametric function with index @ref \_funcIdx at parametric arc set by @ref
  ///setEvalArc.
public:
  virtual double computeFuncDiff2(const std::size_t& _funcIdx) const = 0;
  ///@brief Computes integral of parametric function with index @ref \_funcIdx on interval [@ref funcsArcBegin\_, @ref
  ///funcsArcEnd\_].
public:
  virtual double computeFuncInt1(const std::size_t& _funcIdx) const = 0;
  ///@brief Computes double integral of parametric function with index @ref \_funcIdx on interval [@ref funcsArcBegin\_,
  ///@ref funcsArcEnd\_].
public:
  virtual double computeFuncInt2(const std::size_t& _funcIdx) const = 0;
};
}

#endif  // PARAM_FUNC_H
