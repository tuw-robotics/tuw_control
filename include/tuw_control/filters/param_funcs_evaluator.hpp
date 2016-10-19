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

#ifndef IWS_PARAM_FUNCS_EVALUATOR_H
#define IWS_PARAM_FUNCS_EVALUATOR_H

#include <tuw_control/utils.h>
#include <tuw_control/filters/param_funcs_to_state.hpp>

namespace tuw {

/*!@class ParamFuncsEvaluator
 * @brief Class manipulating a set of parametric-functions-to-state objects based on requested parametric functions input type
 * @tparam ObservedStateType Class containing the observed state
 * @tparam OutputStateType Class containing the output state
 * @tparam ParamType Class containing the parameter structure
 * @tparam ParamFuncs2StateTypes Parameter pack of the supported parametric functions evaluators
 */
template<typename ObservedStateType, typename OutputStateType, typename ParamType, typename... ParamFuncs2StateTypes>
class ParamFuncsEvaluator {
    
    //special class member functions
    public   : ParamFuncsEvaluator (std::shared_ptr<ParamType> _params) : paramFuncs2State_(std::make_tuple(ParamFuncs2StateTypes(_params)...) ), processingParamFuncs_( false ) {}
    
    /** @brief Performs a parametric functions evaluator computation step (calling the parametric functions evaluator specific for the templated @ref ParamFuncPtrType)
     *  @tparam ParamFuncPtrType Type of parametric functions structure in use
     *  @param _xObs Observed chassis + arc parametrization state
     *  @param _t Temporal evaluation point
     */
    public   : template<typename ParamFuncPtrType> std::shared_ptr<OutputStateType>& compute ( std::shared_ptr<ObservedStateType>& _xObs, ParamFuncPtrType& _paramFuncs, const double& _t ) {
	auto& paramFuncs2State = std::get< Get_Tuple_Index<ParamFuncPtrType, std::tuple<std::shared_ptr<typename ParamFuncs2StateTypes::ParamFuncType>...> >::value >(paramFuncs2State_);
	auto& stateChDes       =  paramFuncs2State.compute( _xObs,    _paramFuncs, _t);
	if( paramFuncs2State.finished() ) { paramFuncs2State.reset(); processingParamFuncs_ = false; }
	return stateChDes;
    }
    /** @brief Loads the initial state into the specific parametric functions evaluators.
     *  @tparam ParamFuncsType Parametric function structure type
     *  @param _state0 Initial state
     */
    public   : template<typename ParamFuncsType> void loadParamFuncsState0( const std::vector<double>& _state0, const double& _timeShift = 0 ) {
	auto& paramFuncsI = std::get<ParamFuncsType>(paramFuncs2State_);
	paramFuncsI.state0ParamFuncs_ = _state0;
	paramFuncsI.timeShift_        = _timeShift;
	paramFuncsI.reset();
    }
    /** @brief Resets all parametric functions evaluators.
     */
    public   : void reset       () { for_each_tuple(paramFuncs2State_, [](ParamFuncs2StateBase& _paramFuncs2StateI){ _paramFuncs2StateI.reset();       } ); }
    /** @brief Calls reloadParam on all parametric functions evaluators.
     */
    public   : void reloadParam () { for_each_tuple(paramFuncs2State_, [](ParamFuncs2StateBase& _paramFuncs2StateI){ _paramFuncs2StateI.reloadParam(); } ); }
    
    public   :std::tuple<ParamFuncs2StateTypes...> paramFuncs2State_;///< Tuple storing the parametric functions evaluators
    public   : bool processingParamFuncs_;///< Flags if any evaluator is active
};

}

#endif // IWS_PARAM_FUNCS_EVALUATOR_H