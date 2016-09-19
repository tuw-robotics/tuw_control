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

#ifndef PARAM_FUNCS_2_STATE_HPP
#define PARAM_FUNCS_2_STATE_HPP

#include <float.h>
#include <memory>

#include <tuw_control/state/state.h>

namespace tuw {

/*!@class ParamFuncs2State
 * @brief Interface for a filter that outputs a desired %state given an observed %state and a parametric functions structure.
 * @tparam InputStateType Class defining the current (observed) state of the afferent system
 * @tparam ParamFuncsType Class defining the parametric functions structure to be evaluated (e.g. multivariate splines)
 * @tparam OutputStateType Class defining the computed state of the filter
 * @tparam ParamType       Class defining the filter parameters
 */
template <typename InputStateType, typename ParamFuncsType, typename OutputStateType, typename ParamType>
class ParamFuncs2State {
    
    //special class member functions
    public   : ParamFuncs2State           (std::shared_ptr<ParamType> _params) : params_(_params) {}
    public   : virtual ~ParamFuncs2State  ()                        = default;
    public   : ParamFuncs2State           (const ParamFuncs2State&) = default;
    public   : ParamFuncs2State& operator=(const ParamFuncs2State&) = default;
    public   : ParamFuncs2State           (ParamFuncs2State&&)      = default;
    public   : ParamFuncs2State& operator=(ParamFuncs2State&&)      = default;
    
    public   : using ParamFuncType = ParamFuncsType;///< Parametric functions class type
    
    //pure virtual functions
    /** @brief Computes the desired %state at the specified time instant given the observed %state and a parametric functions structure.
     *  @param _x Observed %state
     *  @param _funcs Parametric functions structure
     *  @param _t Temporal evaluation point
     *  @return computed output %state
     */
    public   : virtual std::shared_ptr<OutputStateType>& compute ( std::shared_ptr<InputStateType>& _x, std::shared_ptr<ParamFuncsType>& _funcs, const double& _t ) = 0;
    /** @brief Reloads class parameters.
     *  To be called when parameters that influence the class variables are changed.
     */
    public   : virtual void reloadParam ()  = 0;
    /** @brief Returns wether the parametric functions evaluation has reached the end of the parametric functions domain definition.
     *  @return True if evaluation is outside/on the border of the domain definition of the parametric functions.
     */
    public   : virtual bool finished    () const = 0;
    /** @brief Resets class structures/variables.
     */
    public   : virtual void reset       () = 0;
    /** @brief Access to the last computed output %state.
     */
    public   : std::shared_ptr<OutputStateType>& output  () { return output_; }
    
    protected: std::shared_ptr<ParamType>        params_;///< Pointer to the class parameters object
    protected: std::shared_ptr<OutputStateType>  output_;///< Last computet output %state
};



}

#endif // PARAM_FUNCS_2_STATE_HPP