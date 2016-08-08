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

namespace tuw {

/*!@class StateFeedback1Output
 *
 * 
 */
template <typename InputStateType>
class StateFeedback1Output : public StateFeedback<InputStateType, InputStateType, double, State > {
    
    //special class member functions
    public   : StateFeedback1Output           (std::shared_ptr<State> _params) : StateFeedback<InputStateType, InputStateType, double, State >(_params) { 
	this->output_ = std::make_shared<double>(); reloadParamInternal_ = false; 
    }
    public   : virtual ~StateFeedback1Output  ()                            = default;
    public   : StateFeedback1Output           (const StateFeedback1Output&) = default;
    public   : StateFeedback1Output& operator=(const StateFeedback1Output&) = default;
    public   : StateFeedback1Output           (StateFeedback1Output&&)      = default;
    public   : StateFeedback1Output& operator=(StateFeedback1Output&&)      = default;
    
    //pure virtual functions
    public   : virtual std::shared_ptr<double>& compute ( std::shared_ptr<InputStateType>& _xObs, std::shared_ptr<InputStateType>& _xDes, const double& _t ) override {
	State::minus( *_xObs, *_xDes, xDiffVec_ );
	if ( reloadParamInternal_ ) { reloadParamInternal(); }
	
	*this->output_    = - ( k_.dot(xDiffVec_)/*.sum()*/ );
	return this->output_;
    }
    public   : void reloadParam () override {
	reloadParamInternal_ = true;
    }
    public   : Eigen::VectorXd& xDiff () { return xDiffVec_; }
    private  : void reloadParamInternal () {
	reloadParamInternal_ = false;
	k_.resize(xDiffVec_.rows());
	for(size_t i = 0; i < (size_t)xDiffVec_.size(); ++i) { k_(i) = this->params_->value(i); }
    }
    private  : bool reloadParamInternal_;
    private  : Eigen::VectorXd xDiffVec_;
    private  : Eigen::VectorXd k_;
};



}

#endif // STATE_FEEDBACK_1OUTPUT_HPP