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

#ifndef STATE_FEEDBACK_HPP
#define STATE_FEEDBACK_HPP

#include <float.h>
#include <memory>

#include <tuw_control/state/state.h>

namespace tuw
{
/*!@class StateFeedback
 * @brief Interface for a filter that outputs a desired %state given an observed %state and a desired %state.
 * @tparam InputObsStateType Class defining the current observed state of the afferent system
 * @tparam InputDesStateType Class defining the current desired state of the afferent system
 * @tparam OutputStateType Class defining the computed state of the filter
 * @tparam ParamType       Class defining the filter parameters
 */
template <typename InputObsStateType, typename InputDesStateType, typename OutputStateType, typename ParamType>
class StateFeedback
{
  // special class member functions
public:
  StateFeedback(std::shared_ptr<ParamType> _params) : params_(_params)
  {
  }

public:
  virtual ~StateFeedback() = default;

public:
  StateFeedback(const StateFeedback&) = default;

public:
  StateFeedback& operator=(const StateFeedback&) = default;

public:
  StateFeedback(StateFeedback&&) = default;

public:
  StateFeedback& operator=(StateFeedback&&) = default;

  // pure virtual functions
  /** @brief Computes the output %state at the specified time instant given the observed %state and the desired state.
   *  @param _xObs Observed %state
   *  @param _xDes Desired %state
   *  @param _t Temporal evaluation point
   *  @return computed output %state
   */
public:
  virtual std::shared_ptr<OutputStateType>& compute(std::shared_ptr<InputObsStateType>& _xObs,
                                                    std::shared_ptr<InputDesStateType>& _xDes, const double& _t) = 0;
  /** @brief Reloads class parameters.
   *  To be called when parameters that influence the class variables are changed.
   */
public:
  virtual void reloadParam() = 0;
  /** @brief Access to the last computed output %state.
   */
public:
  std::shared_ptr<OutputStateType>& output()
  {
    return output_;
  }
  /** @brief Access to the last parameters object pointer.
   */
public:
  std::shared_ptr<ParamType>& params()
  {
    return params_;
  }

protected:
  std::shared_ptr<ParamType> params_;  ///< Pointer to the class parameters object
protected:
  std::shared_ptr<OutputStateType> output_;  ///< Last computet output %state
};
}

#endif  // STATE_FEEDBACK_HPP