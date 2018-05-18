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

#ifndef STATE_FEEDBACK_1OUTPUT_HPP
#define STATE_FEEDBACK_1OUTPUT_HPP

#include <float.h>
#include <memory>

#include <tuw_control/filters/state_feedback.hpp>
#include <tuw_control/filters/integrator.hpp>

namespace tuw
{
/*!@class StateFeedback1Output
 * @brief Partial implementation of the @ref StateFeedback interface for 1-dimensional output linear state-feedback
 *filter types.
 * @tparam InputStateType Class defining the current state of the afferent system
 * @tparam ParamType      Class defining the filter parameters
 * The class represents a generic state-feedback filter (with integrator term) for 1-dimensional output states:
 *
 * Given an observed state vector \f$ \mathbf{x_{obs}} = [xo_0, xo_1, xo_2, ..., xo_n]^T \f$, a desired state vector \f$
 *\mathbf{x_{des}} = [xd_0, xd_1, xd_2,..., xd_m]^T \f$, and a specified output order \f$ 0 \leq ord_u \leq n+1 \f$
 *(@ref outputOrder_), the filter computes \f$u\f$ as follows:
 *
 * \f$ u = - [k_I, \mathbf{k_x}] \cdot [ e_I, \mathbf{x_{obs}}_{0..dim_{\Delta \mathbf{x}}} -
 *\mathbf{x_{des}}_{0..dim_{\Delta \mathbf{x}}}], \quad dim_{\Delta \mathbf{x}} = min(ord_u+1, m) \f$
 *
 * The integrator performs (numerically stable) integration on the error \f$ e_I = \int_{0}^{t}{e_{x_0}}dt, \ e_{x_0} =
 *xo_0 - xd_0 \f$. It includes anti-windup and thus integration is not performed in a cycle if the computed output \f$ u
 *\f$ is outside the box-constraint @ref intSaturateVal_. Note that integration can be disabled either by setting @ref
 *intSaturateVal_ or \f$ x_I \f$ to \f$ 0 \f$.
 *
 * If \f$ dim_{\Delta \mathbf{x}} = ord_u+1 \f$, \f$ \mathbf{x_{obs}}_{dim_{\Delta \mathbf{x}}} \f$ is set to \f$ 0 \f$
 *and \f$ \mathbf{k_x}_{dim_{\Delta \mathbf{x}}} \f$ is set to \f$ 1 \f$. This implies that the last state becomes
 *feed-forward (as long as it is available and at the same order as the output order).
 *
 * Variables that have to be manipulated by class extensions: @ref outputOrder_, @ref intSaturateVal_, @ref kInt_, @ref
 *kX_, @ref reloadParamInternal_.
 */
template <typename InputStateType, typename ParamsType>
class StateFeedback1Output : public StateFeedback<InputStateType, InputStateType, double, ParamsType>, public Integrator
{
  // special class member functions
public:
  StateFeedback1Output(std::shared_ptr<ParamsType> _params)
    : StateFeedback<InputStateType, InputStateType, double, ParamsType>(_params), reloadParamInternal_(false)
  {
    this->output_ = std::make_shared<double>();
  }

public:
  virtual ~StateFeedback1Output() = default;

public:
  StateFeedback1Output(const StateFeedback1Output&) = default;

public:
  StateFeedback1Output& operator=(const StateFeedback1Output&) = default;

public:
  StateFeedback1Output(StateFeedback1Output&&) = default;

public:
  StateFeedback1Output& operator=(StateFeedback1Output&&) = default;

public:
  std::shared_ptr<double>& compute(std::shared_ptr<InputStateType>& _xObs, std::shared_ptr<InputStateType>& _xDes,
                                   const double& _t) override
  {
    desSize_ = _xDes->valueSize();
    if (reloadParamInternal_)
    {
      reloadParamInternal();
    }

    for (size_t i = 0; i < outputOrder_; ++i)
    {
      xDiffVec_(i) = _xObs->value(i) - _xDes->value(i);
    }
    for (size_t i = outputOrder_; i < xDiffSize_; ++i)
    {
      xDiffVec_(i) = -_xDes->value(i);
    }

    *this->output_ = -kX_.dot(xDiffVec_) - kInt_ * intOutput();
    if (fabs(*this->output_) < intSaturateVal_)
    {
      integrate(xDiffVec_(0) * (_t - t_));
    }
    t_ = _t;
    return this->output_;
  }
  /** @brief Performs class specific reconfiguration on parameters change. */
private:
  void reloadParamInternal()
  {
    reloadParamInternal_ = false;
    xDiffSize_ = std::min(outputOrder_ + 1, desSize_);
    xDiffVec_.resize(xDiffSize_);
    kX_.conservativeResize(xDiffSize_);
    if (outputOrder_ != xDiffSize_)
    {
      kX_(kX_.rows() - 1) = 1;
    }
    if (kInt_ == 0)
    {
      Integrator::reset(0);
      t_ = 0;
    }
  }

protected:
  size_t outputOrder_;  ///< Order of the output variable in the defined state
protected:
  double intSaturateVal_;  ///< Box constraint. When output outside of it, error integration is not performed
protected:
  double kInt_;  ///< gain of the integrated error
protected:
  Eigen::VectorXd kX_;  ///< %State error gains
protected:
  bool reloadParamInternal_;  ///< Triggers base class reconfiguration. To be set to true on any parameter change

private:
  size_t desSize_;

private:
  size_t xDiffSize_;

private:
  size_t outputOrderRelXDes_;

private:
  double t_;

private:
  Eigen::VectorXd xDiffVec_;
};
}

#endif  // STATE_FEEDBACK_1OUTPUT_HPP