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

#ifndef STATE_FEEDBACK_BACKSTEP_HPP
#define STATE_FEEDBACK_BACKSTEP_HPP

#include <float.h>
#include <memory>

#include <tuw_control/filters/state_feedback.hpp>
#include <tuw_control/filters/state_mapping.hpp>

namespace tuw {

/*!@class StateMapping
 *
 * 
 */

// template <typename InputStateType, typename OutputStateType, typename ParamType, typename StateFeedback1Type, typename StateFeedback2Type, typename StateMappingType>
// class StateFeedbackBackStep : public StateFeedback<InputStateType, OutputStateType, ParamType>{
//     
//     //special class member functions
//     public   : StateFeedbackBackStep           (ParamType& _params) : StateFeedback<InputStateType, OutputStateType, ParamType>(_params) {}
//     public   : virtual ~StateFeedbackBackStep  ()                             = default;
//     public   : StateFeedbackBackStep           (const StateFeedbackBackStep&) = default;
//     public   : StateFeedbackBackStep& operator=(const StateFeedbackBackStep&) = default;
//     public   : StateFeedbackBackStep           (StateFeedbackBackStep&&)      = default;
//     public   : StateFeedbackBackStep& operator=(StateFeedbackBackStep&&)      = default;
//     
//     //pure virtual functions
//     
//     protected: StateFeedback1Type stateFeedback1_;
//     protected: StateFeedback2Type stateFeedback2_;
//     protected: StateMappingType   stateMapping_;
// };



}

#endif // STATE_MAPPING_HPP