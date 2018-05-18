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

#ifndef PARAM_FUNC_DIST_H
#define PARAM_FUNC_DIST_H

#include <memory>
#include <vector>
#include <map>
#include <tuw_control/utils.h>
#include <tuw_control/param_func/param_func.h>

namespace tuw
{
/*!@class ParamFuncsDist
 * @brief Extends manipulation of parametric functions collection with closed-form arc length (distance) computation
 *
 */
class ParamFuncsDist;
using ParamFuncsDistPtr = std::shared_ptr<ParamFuncsDist>;
using ParamFuncsDistConstPtr = std::shared_ptr<ParamFuncsDist const>;
class ParamFuncsDist : public ParamFuncs
{
  // enums
  ///@brief Required type of traveled distance computation relative to the parametric function.
public:
  enum class TraveledDistCfMode
  {
    NONE,  ///<no closed-form distance computation mode
    V,     ///<agent base center linear velocity is parametric function
    AV     ///<agent base center linear acceleration is parametric function
  };

  // special class member functions
public:
  ParamFuncsDist() = default;

public:
  virtual ~ParamFuncsDist() = default;

public:
  ParamFuncsDist(const ParamFuncsDist& _other) = default;

public:
  ParamFuncsDist& operator=(const ParamFuncsDist& _other) = default;

public:
  ParamFuncsDist(ParamFuncsDist&&) = delete;

public:
  ParamFuncsDist& operator=(ParamFuncsDist&&) = delete;

  // pure virtual functions
  ///@brief Initializer of the Closed form distance computation mode
public:
  virtual void setDistCfMode(TraveledDistCfMode _distCfMode, const std::vector<std::size_t>& _distRelFuncIdx) = 0;
  ///@brief Moves to evaluation arc at which the traveled distance @ref _funcsDistEval is achieved.
public:
  virtual void
  setEvalDist(const double& _funcsDistEval,
              const ParamFuncs::EvalArcGuarantee& _evalArcGuarantee = ParamFuncs::EvalArcGuarantee::NONE) = 0;
  /// Solves the equation \f$ \int_{0}^{evalArc\_}{ |v(\mathbf{p}, t)| } dt  = \_s \f$ for @ref \_deltaS (evalArc\_:
  /// time, _s: traveled distance, v: body linear velocity,\f$ \mathbf{p} \f$: parametrized control points).
public:
  virtual double computeS() const = 0;
  /// Solves the equation \f$ \int_{0}^{\_deltaT}{ |v(\mathbf{p}, t)| } dt  = \_s \f$ for @ref \_s (evalArc\_: time, _s:
  /// traveled distance, v: body linear velocity,\f$ \mathbf{p} \f$: parametrized control points).
public:
  virtual double computeT(
      const double& _s, const ParamFuncs::EvalArcGuarantee& _evalArcGuarantee = ParamFuncs::EvalArcGuarantee::NONE) = 0;
  /** @brief Computes arc parametrization lattice given a distance-parametrized lattice.
   *  @param _sLattice Distance-parametrized input lattice. It is assumed that the vector is monotonically increasing.
   *  @param _tLattice Arc-parametrized output lattice.
   */
public:
  virtual void computeS2TLattice(const std::vector<double>& _sLattice, std::vector<double>& _tLattice) = 0;
  /** @brief Computes arc parametrization lattice given an inital arc and distance parametrized sampling interval.
   *
   *  The function computes the temporal lattice starting with s(_arc0) and ending with the maximum value of the arc
   *parametrization.
   *
   *  @param _sLattice Distance-parametrized input lattice. It is assumed that the vector values are monotonically
   *increasing.
   *  @param _tLattice Arc-parametrized output lattice.
   */
public:
  virtual void computeS2TLattice(const double& _arc0, const double& _ds, std::vector<double>& _tLattice) = 0;
};
}

#endif  // PARAM_FUNC_DIST_H
