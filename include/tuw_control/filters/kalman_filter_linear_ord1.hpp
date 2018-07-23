/***************************************************************************
 *   Software License Agreement (BSD License)                              *
 *   Copyright (C) 2015 by Horatiu George Todoran <todorangrg@gmail.com>   *
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
#ifndef KALMAN_FILTER_LINEAR_ORD1_H
#define KALMAN_FILTER_LINEAR_ORD1_H

#include <tuw_control/filters/kalman_filter.hpp>

namespace tuw
{
/*!@class KalmanFilterLinOrd1
 * @brief Partial implementation of @ref KalmanFilterPredictInterface for 1st order multivariate (linear) integrator
 * systems.
 * The state variables are expected to be ordered in a 2 * XDim vector where the first XDim variables are the order 0
 * %states and the 2nd XDim variables are their corresponding derivatives.
 */
template <typename NumType, size_t XDim, size_t UDim, typename ParamType>
class KalmanFilterLinOrd1 : public KalmanFilterPredictInterface<NumType, 2 * XDim, UDim, ParamType>
{
public:
  KalmanFilterLinOrd1(ParamType& _params)
    : KalmanFilterPredictInterface<NumType, 2 * XDim, UDim, ParamType>(_params), Ta_(0), recalculate_(false)
  {
    this->Phi_.setZero();
    for (size_t i = 0; i < 2 * XDim; ++i)
    {
      this->Phi_(i, i) = 1;
    }
    this->f_.setZero();
    this->Q_.setZero();
    this->Sigma_.setZero();
    computeSigmaInit();
  }

public:
  virtual void computeSigmaInit() override
  {
    for (size_t i = 0; i < 2 * XDim; ++i)
    {
      this->Sigma_(i, i) = 1000000;
    }
  }

public:
  virtual void precompute(const double& _Ta) override
  {
    if (_Ta == Ta_)
    {
      recalculate_ = true;
    }
    else
    {
      recalculate_ = true;
      Ta_ = _Ta;
      TaSqr_ = Ta_ * Ta_;
      TaCub_ = TaSqr_ * Ta_;
    }
  }

public:
  void computePhi() override
  {
    if (recalculate_)
    {
      for (size_t i = 0; i < XDim; ++i)
      {
        this->Phi_(i, i + XDim) = Ta_;
      }
    }
  }

public:
  void computef(const Eigen::Matrix<NumType, UDim, 1>& _u) override
  {
    for (size_t i = 0; i < XDim; ++i)
    {
      this->f_(i) = this->x_(i) + this->x_(i + XDim) * Ta_;
      this->f_(i + XDim) = this->x_(i + XDim);
    }
  }

public:
  void computeQ() override
  {
    if (recalculate_ || recalculateQ_)
    {
      for (size_t i = 0; i < XDim; ++i)
      {
        this->Q_(i, i) = TaCub_ * nn_(i);
        this->Q_(i, i + XDim) = this->Q_(i + XDim, i) = TaSqr_ * nn_(i);
        this->Q_(i + XDim, i + XDim) = Ta_ * nn_(i);
      }
      recalculateQ_ = false;
    }
  }
  /** @brief Sets a noise variance parameter.
   *  @param _i Index of the state correspoing variance
   *  @param _val Variance value
   */
public:
  void setNN(const size_t& _i, const double& _val)
  {
    nn_(_i) = _val;
    recalculateQ_ = true;
  }

private:
  Eigen::Matrix<NumType, XDim, 1> nn_;  ///< container storing state noise variance values.
private:
  double Ta_, TaSqr_, TaCub_;

private:
  bool recalculate_;

private:
  bool recalculateQ_;
};
}

#endif  // KALMAN_FILTER_LINEAR_ORD1_H