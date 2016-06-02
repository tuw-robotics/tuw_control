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

#ifndef FEED_FWD_CONTROL_H
#define FEED_FWD_CONTROL_H

#include <array>
#include <memory>


namespace tuw {

template<  typename SystemStateType, typename FeedForwardStateType, typename BodyStateTrajectoryType, typename SystemParamsType  >
class FeedFwdCtrl;

//Shared Pointer
template<  typename SystemStateType, typename FeedForwardStateType, typename BodyStateTrajectoryType, typename SystemParamsType  >
using FeedFwdCtrlSPtr = std::shared_ptr< FeedFwdCtrl<SystemStateType, FeedForwardStateType, BodyStateTrajectoryType, SystemParamsType> >;
//Unique Pointer
template<  typename SystemStateType, typename FeedForwardStateType, typename BodyStateTrajectoryType, typename SystemParamsType  >
using FeedFwdCtrlUPtr = std::unique_ptr< FeedFwdCtrl<SystemStateType, FeedForwardStateType, BodyStateTrajectoryType, SystemParamsType> >;
    
template< typename SystemStateType, typename FeedForwardStateType, typename BodyStateTrajectoryType, typename SystemParamsType >
class FeedFwdCtrl {
    
    //special class member functions
    public   : FeedFwdCtrl           ()                   = default;
    public   : virtual ~FeedFwdCtrl  ()                   = default;
    public   : FeedFwdCtrl           (const FeedFwdCtrl&) = default;
    public   : FeedFwdCtrl& operator=(const FeedFwdCtrl&) = default;
    public   : FeedFwdCtrl           (FeedFwdCtrl&&)      = default;
    public   : FeedFwdCtrl& operator=(FeedFwdCtrl&&)      = default;
    
    /** @brief Initializes the controller with the system static parameters.
      * @param _systemParams Static system parameters.
      */
    public   : virtual void init                                     ( const SystemParamsType&  _systemParams   ) = 0;
    /** @brief Computes the feed-forward input of the system.
      * @param _systemState  State of the system (possibly estimated).
      * @param _stateTraj    Parametrized trajectory of some state variables (the control trajectory).
      * @param _t            Time stamp of the trajectory evaluation.
      * @param _feedFwdState Computed feed-forward desired state.
      * @return Computed feed-forward desired state.
      */
    public   : virtual FeedForwardStateType& computeFeedForwardInput ( const SystemStateType&   _systemState, 
	                                                               BodyStateTrajectoryType& _stateTraj, 
								       const double&            _t, 
								       FeedForwardStateType&    _feedFwdState   ) = 0;
};

}

#endif // FEED_FWD_CONTROL_H

