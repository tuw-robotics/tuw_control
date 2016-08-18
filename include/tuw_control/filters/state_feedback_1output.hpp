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

namespace tuw {

/*!@class StateFeedback1Output
 *
 * 
 */
template <typename InputStateType, typename ParamsType>
class StateFeedback1Output : public StateFeedback<InputStateType, InputStateType, double, ParamsType >, public Integrator {
    
    //special class member functions
    public   : StateFeedback1Output           (std::shared_ptr<ParamsType> _params) : StateFeedback<InputStateType, InputStateType, double, ParamsType >(_params), reloadParamInternal_(false) { 
	this->output_ = std::make_shared<double>(); 
    }
    public   : virtual ~StateFeedback1Output  ()                            = default;
    public   : StateFeedback1Output           (const StateFeedback1Output&) = default;
    public   : StateFeedback1Output& operator=(const StateFeedback1Output&) = default;
    public   : StateFeedback1Output           (StateFeedback1Output&&)      = default;
    public   : StateFeedback1Output& operator=(StateFeedback1Output&&)      = default;
    
    public   : std::shared_ptr<double>& compute ( std::shared_ptr<InputStateType>& _xObs, std::shared_ptr<InputStateType>& _xDes, const double& _t ) override {
	desSize_ = _xDes->valueSize();
	if ( reloadParamInternal_ ) { reloadParamInternal(); }
	
	for( size_t i =            0; i < outputOrder_; ++i ) { xDiffVec_(i) = _xObs->value(i) - _xDes->value(i); }
	for( size_t i = outputOrder_; i <   xDiffSize_; ++i ) { xDiffVec_(i) =                 - _xDes->value(i); }
	
	*this->output_    = - k_.dot(xDiffVec_) - kInt_* intOutput();
	if( fabs(*this->output_) < intSaturateVal_ ) { integrate( xDiffVec_(0) * (_t - t_) ); }
	t_ = _t;
	return this->output_;
    }
    private  : void reloadParamInternal () {
	reloadParamInternal_ = false;
	xDiffSize_ = std::min(outputOrder_ + 1, desSize_);
	xDiffVec_.resize(xDiffSize_);
	k_       .conservativeResize(xDiffSize_);
	if( outputOrder_ != xDiffSize_ ) { k_( k_.rows()-1 ) = 1; }
	if ( kInt_ == 0 ) { Integrator::reset(0); t_ = 0; }
    }
    
    protected: size_t outputOrder_;
    protected: double intSaturateVal_;
    protected: double kInt_;
    protected: Eigen::VectorXd k_;
    protected: bool   reloadParamInternal_;
    
    private  : size_t desSize_;
    private  : size_t xDiffSize_;
    private  : size_t outputOrderRelXDes_;
    private  : double t_;
    private  : Eigen::VectorXd xDiffVec_;
};



}

#endif // STATE_FEEDBACK_1OUTPUT_HPP