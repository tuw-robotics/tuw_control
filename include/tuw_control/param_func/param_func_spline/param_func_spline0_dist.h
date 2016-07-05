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

#ifndef PARAM_FUNC_SPLINE0_DIST_H
#define PARAM_FUNC_SPLINE0_DIST_H

#include <memory>
#include <vector>

#include <tuw_control/param_func/param_func_dist.h>

namespace tuw {


    
/*!@class ParamFuncsSpline0Dist 
 * 
 * @todo Test distance computation for AV input. 
 * @todo Should s0 = 0 (the way it is now) or add another function that sets it?
 * @todo cout the control points and their arc parametrizations (override with dist also)
 */
    
class ParamFuncsSpline0Dist;
using ParamFuncsSpline0DistPtr      = std::shared_ptr<ParamFuncsSpline0Dist>;
using ParamFuncsSpline0DistConstPtr = std::shared_ptr<ParamFuncsSpline0Dist const>;
class ParamFuncsSpline0Dist : public ParamFuncsDist {

    //--------------------------------------- ParamFunc implementation ---------------------------------------//
    ///@brief Structure referencing a control point and storing cached relevant function evaluation data (derivatives, integrals etc.).
    protected : struct FuncCacheData {
	FuncCacheData ( const FuncCtrlPt& _ctrlPt ) : val(_ctrlPt.val), arc(_ctrlPt.arc) {}
	const double& val;
	const double& arc;
	std::array<double, nrModesMax_> cache;
    };

    //special class member functions
    public   : ParamFuncsSpline0Dist           ()                             = default;
    public   : virtual ~ParamFuncsSpline0Dist  ()                             = default;
    public   : ParamFuncsSpline0Dist           (const ParamFuncsSpline0Dist&) = default;
    public   : ParamFuncsSpline0Dist& operator=(const ParamFuncsSpline0Dist&) = default;
    public   : ParamFuncsSpline0Dist           (ParamFuncsSpline0Dist&&)      = default;
    public   : ParamFuncsSpline0Dist& operator=(ParamFuncsSpline0Dist&&)      = default;
    
    //(re-)implemented virtual functions
    protected: virtual void   initImpl         () override;
    ///@brief Precomputes cached data. To be called after ANY control point modifications.
    public   : virtual void   precompute       () override;
    public   :         void   setEvalArc       ( const double& _funcsArcEval, const EvalArcGuarantee& _evalArcGuarantee = EvalArcGuarantee::NONE ) override;
    public   : virtual double computeFuncVal   ( const std::size_t& _funcIdx ) const override;
    public   : virtual double computeFuncDiff1 ( const std::size_t& _funcIdx ) const override;
    public   : virtual double computeFuncDiff2 ( const std::size_t& _funcIdx ) const override;
    public   : virtual double computeFuncInt1  ( const std::size_t& _funcIdx ) const override;
    public   : virtual double computeFuncInt2  ( const std::size_t& _funcIdx ) const override;
    
    ///@brief Computes the integral of the parametric function @ref _\funcIdx on interval [@ref funcEvalArcCacheIdxOld\_[func2Arc_[_funcIdx]] - 1, @ref \_arcEnd]
    private  : double computeFuncDeltaInt1 ( const std::size_t& _funcIdx, const double& _arcEnd ) const;
    ///@brief Computes the double integral integral of the parametric function @ref _\funcIdx on interval [@ref funcEvalArcCacheIdxOld\_[func2Arc_[_funcIdx]] - 1, @ref \_arcEnd]
    private  : double computeFuncDeltaInt2 ( const std::size_t& _funcIdx, const double& _arcEnd ) const;
    
    ///@brief Contains the index of the \htmlonly<u>next</u>\endhtmlonly  control point relative to the last evaluation point.
    protected: std::vector< std::size_t                  > funcEvalArcCacheIdxUnder_;
    ///@brief Cached values of the used function evaluation modes.
    protected: std::vector< std::vector< FuncCacheData > > funcEvalCache_;
    ///@brief Maps the arc parametrizations to the functions that use them.
    private  : std::vector< std::vector<std::size_t> > arc2func_;
    
    
    //--------------------------------------- ParamFuncDist implementation ---------------------------------------//
    
    //(re-)implemented virtual functions
    public   : void setDistCfMode ( TraveledDistCfMode _distCfMode, const std::vector<std::size_t>& _distRelFuncIdx ) override;
    public   : double computeS   () const override;
    public   : double computeT ( const double& _s, const EvalArcGuarantee& _evalArcGuarantee = EvalArcGuarantee::NONE  ) override;
    public   : void   setEvalDist ( const double& _funcsDistEval, const EvalArcGuarantee& _evalArcGuarantee = EvalArcGuarantee::NONE ) override;
    public   : void   computeS2TLattice ( const std::vector<double>& _sLattice, std::vector<double>& _tLattice ) override;
    public   : void   computeS2TLattice ( const double& _arc0, const double& _ds, std::vector<double>& _tLattice ) override;
    
    ///@brief Helper function that computes deltaS (from the @ref evalArc\_ position) operating on one function control point interval (used by @ref TraveledDistCfMode::V and @ref TraveledDistCfMode::AV modes).
    public   : static double computeDeltaS_V_AV ( const double& _dt, const double& _v0, const double& _av );
    ///@brief Helper function that computes deltaT (from the @ref evalArc\_ position) operating on one function control point interval (used by @ref TraveledDistCfMode::V and @ref TraveledDistCfMode::AV modes).
    public   : static double computeDeltaT_V_AV ( const double& _ds, const double& _v0, const double& _av );
    
    ///@brief Precomputes distance invervals.
    private  : void     precomputeDist ();
    ///@brief Internal implementation of computing the arc parametrization given a distance @ref _s.
    private  : double   computeTImpl   ( const double& _s, const EvalArcGuarantee& _evalArcGuarantee );
    ///@brief Computes distance on a piecewise linear function describing the center linear velocity.
    private  : double   computeS_V     () const;
    ///@brief Computes distance on a piecewise linear function describing the center linear acceleration.
    private  : double   computeS_AV    () const;
    ///@brief Computes arc parametrization on a piecewise linear function describing the center linear velocity at a variation @ref _ds.
    private  : double   computeT_V     ( const double& _ds ) const;
    ///@brief Computes arc parametrization on a piecewise linear function describing the center linear acceleration at a variation @ref _ds.
    private  : double   computeT_AV    ( const double& _ds ) const;
    
    ///@brief Indexes of parametric functions used for computing the traveled distance.
    private  : std::vector<std::size_t> distLinkedFuncIdx_;
    ///@brief Cached values of traveled distance computation.
    private  : std::vector<double     > distEvalCache_;
    ///@brief Closed form distance computation mode.
    private  : TraveledDistCfMode distCfMode_;
    ///@brief Index of the parametrized function that relates to distance computation.
    private  : std::size_t distLinkedArcIdx_;
    
    private  : using ComputeSFuncPtr = double (ParamFuncsSpline0Dist::*)( ) const;
    private  : using ComputeDs2DtPtr = double (ParamFuncsSpline0Dist::*)( const double& ) const;
    private  : ComputeSFuncPtr computeSFuncPtr_;
    private  : ComputeDs2DtPtr computeDs2DtPtr_;
};

}

#endif // PARAM_FUNC_SPLINE0_DIST_H
