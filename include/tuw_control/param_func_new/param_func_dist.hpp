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

#ifndef PARAM_FUNC_DIST_HPP
#define PARAM_FUNC_DIST_HPP

#include <memory>
#include <vector>
#include <map>
#include <tuw_control/utils.h>
#include <tuw_control/param_func_new/param_func.hpp>

namespace tuw {

    
/*!@class ParamFuncsDist 
 * @brief Extends manipulation of parametric functions collection with closed-form arc length (distance) computation
 * 
 */
class ParamFuncsDist;
using ParamFuncsDistPtr      = std::shared_ptr<ParamFuncsDist>;
using ParamFuncsDistConstPtr = std::shared_ptr<ParamFuncsDist const>;

//enums
///@brief Required type of traveled distance computation relative to the parametric function.
enum class TraveledDistCfMode { 
    NONE, ///<no closed-form distance computation mode
    V   ,///<agent base center linear velocity is parametric function
    AV   ///<agent base center linear acceleration is parametric function
}; 

namespace {
 
template<class TDerived>
struct ParamFuncsDistBaseCRTPTraits;
    
}

template<typename TDerived>
class ParamFuncsDistBaseCRTP {
    
    protected: using NumType = typename ParamFuncsDistBaseCRTPTraits<TDerived>::NumType;
    
    //special class member functions
    public   : ParamFuncsDistBaseCRTP           ()                              = default;
    public   : ~ParamFuncsDistBaseCRTP          ()                              = default;
    public   : ParamFuncsDistBaseCRTP           (const ParamFuncsDistBaseCRTP&) = default;
    public   : ParamFuncsDistBaseCRTP& operator=(const ParamFuncsDistBaseCRTP&) = default;
    public   : ParamFuncsDistBaseCRTP           (ParamFuncsDistBaseCRTP&&)      = delete;
    public   : ParamFuncsDistBaseCRTP& operator=(ParamFuncsDistBaseCRTP&&)      = delete;
    
    ///@brief Initializer of the Closed form distance computation mode
    public   : void setDistCfMode ( TraveledDistCfMode _distCfMode, const std::vector<std::size_t>& _distRelFuncIdx ) { thisDerived().setDistCfModeImplCRTP(_distCfMode, _distRelFuncIdx); }
    ///@brief Moves to evaluation arc at which the traveled distance @ref _funcsDistEval is achieved.
    public   : void   setEvalDist      ( const NumType& _funcsDistEval, const EvalArcGuarantee& _eAG = EvalArcGuarantee::NONE ) { thisDerived().setEvalDistImplCRTP(_funcsDistEval, _eAG); }
    ///Solves the equation \f$ \int_{0}^{evalArc\_}{ |v(\mathbf{p}, t)| } dt  = \_s \f$ for @ref \_deltaS (evalArc\_: time, _s: traveled distance, v: body linear velocity,\f$ \mathbf{p} \f$: parametrized control points).
    public   : NumType computeS () const { return thisDerived().computeSImplCRTP(); }
    ///Solves the equation \f$ \int_{0}^{\_deltaT}{ |v(\mathbf{p}, t)| } dt  = \_s \f$ for @ref \_s (evalArc\_: time, _s: traveled distance, v: body linear velocity,\f$ \mathbf{p} \f$: parametrized control points).
    public   : NumType computeT ( const NumType& _s, const EvalArcGuarantee& _eAG = EvalArcGuarantee::NONE ) { return thisDerived().computeTImplCRTP(_s, _eAG); }
    /** @brief Computes arc parametrization lattice given a distance-parametrized lattice.
     *  @param _sLattice Distance-parametrized input lattice. It is assumed that the vector is monotonically increasing.
     *  @param _tLattice Arc-parametrized output lattice.
     */
    public   : void   computeS2TLattice ( const std::vector<NumType>& _sLattice, std::vector<NumType>& _tLattice ) { thisDerived().computeS2TLatticeImplCRTP(_sLattice, _tLattice); }
    /** @brief Computes arc parametrization lattice given an inital arc and distance parametrized sampling interval.
     * 
     *  The function computes the temporal lattice starting with s(_arc0) and ending with the maximum value of the arc parametrization.
     * 
     *  @param _sLattice Distance-parametrized input lattice. It is assumed that the vector values are monotonically increasing.
     *  @param _tLattice Arc-parametrized output lattice.
     */
    public   : void   computeS2TLattice ( const NumType& _arc0, const NumType& _ds, std::vector<NumType>& _tLattice ) { thisDerived().computeS2TLatticeImplCRTP(_arc0, _ds, _tLattice); }
    
    private  :       TDerived& thisDerived()       { return static_cast<      TDerived&>(*this); }
    private  : const TDerived& thisDerived() const { return static_cast<const TDerived&>(*this); }
};

template<typename TNumType>
class ParamFuncsDistBaseVirt {
    
    //special class member functions
    public   : ParamFuncsDistBaseVirt           ()                              = default;
    public   : virtual ~ParamFuncsDistBaseVirt  ()                              = default;
    public   : ParamFuncsDistBaseVirt           (const ParamFuncsDistBaseVirt&) = default;
    public   : ParamFuncsDistBaseVirt& operator=(const ParamFuncsDistBaseVirt&) = default;
    public   : ParamFuncsDistBaseVirt           (ParamFuncsDistBaseVirt&&)      = delete;
    public   : ParamFuncsDistBaseVirt& operator=(ParamFuncsDistBaseVirt&&)      = delete;
    
    
    ///@brief Initializer of the Closed form distance computation mode
    public   : void    setDistCfMode ( TraveledDistCfMode _distCfMode, const std::vector<std::size_t>& _distRelFuncIdx ) { setDistCfModeImplVirt(_distCfMode, _distRelFuncIdx); }
    ///@brief Moves to evaluation arc at which the traveled distance @ref _funcsDistEval is achieved.
    public   : void     setEvalDist      ( const TNumType& _funcsDistEval, const EvalArcGuarantee& _eAG = EvalArcGuarantee::NONE ) { setEvalDistImplVirt(_funcsDistEval, _eAG); }
    ///Solves the equation \f$ \int_{0}^{evalArc\_}{ |v(\mathbf{p}, t)| } dt  = \_s \f$ for @ref \_deltaS (evalArc\_: time, _s: traveled distance, v: body linear velocity,\f$ \mathbf{p} \f$: parametrized control points).
    public   : TNumType computeS () const { return computeSImplVirt(); }
    ///Solves the equation \f$ \int_{0}^{\_deltaT}{ |v(\mathbf{p}, t)| } dt  = \_s \f$ for @ref \_s (evalArc\_: time, _s: traveled distance, v: body linear velocity,\f$ \mathbf{p} \f$: parametrized control points).
    public   : TNumType computeT ( const TNumType& _s, const EvalArcGuarantee& _eAG = EvalArcGuarantee::NONE ) { return computeTImplVirt(_s, _eAG); }
    /** @brief Computes arc parametrization lattice given a distance-parametrized lattice.
     *  @param _sLattice Distance-parametrized input lattice. It is assumed that the vector is monotonically increasing.
     *  @param _tLattice Arc-parametrized output lattice.
     */
    public   : void     computeS2TLattice ( const std::vector<TNumType>& _sLattice, std::vector<TNumType>& _tLattice ) { computeS2TLatticeImplVirt(_sLattice, _tLattice); }
    /** @brief Computes arc parametrization lattice given an inital arc and distance parametrized sampling interval.
     * 
     *  The function computes the temporal lattice starting with s(_arc0) and ending with the maximum value of the arc parametrization.
     * 
     *  @param _sLattice Distance-parametrized input lattice. It is assumed that the vector values are monotonically increasing.
     *  @param _tLattice Arc-parametrized output lattice.
     */
    public   : void     computeS2TLattice ( const TNumType& _arc0, const TNumType& _ds, std::vector<TNumType>& _tLattice ) { computeS2TLatticeImplVirt(_arc0, _ds, _tLattice); }
    
    
    //pure virtual functions
    private  : virtual void     setDistCfModeImplVirt     ( TraveledDistCfMode _distCfMode, const std::vector<std::size_t>& _distRelFuncIdx ) = 0;
    private  : virtual void     setEvalDistImplVirt       ( const TNumType& _funcsDistEval, const EvalArcGuarantee& _evalArcGuarantee = EvalArcGuarantee::NONE ) = 0;
    private  : virtual TNumType computeSImplVirt          () const = 0;
    private  : virtual TNumType computeTImplVirt          ( const TNumType& _s, const EvalArcGuarantee& _evalArcGuarantee = EvalArcGuarantee::NONE ) = 0;
    private  : virtual void     computeS2TLatticeImplVirt ( const std::vector<TNumType>& _sLattice, std::vector<TNumType>& _tLattice ) = 0;
    private  : virtual void     computeS2TLatticeImplVirt ( const TNumType& _arc0, const TNumType& _ds, std::vector<TNumType>& _tLattice ) = 0;
    
};

template<typename TDerived, typename TNumType>
class ParamFuncsDistBase : public ParamFuncsDistBaseCRTP<ParamFuncsDistBase<TDerived, TNumType>>, public ParamFuncsDistBaseVirt<TNumType> {
    public   : using ParramFuncsDistBaseType = ParamFuncsDistBase<TDerived, TNumType>;
    public   : using NumType = TNumType;
    
    //special class member functions
    public   : ParamFuncsDistBase           ()                          = default;
    public   : virtual ~ParamFuncsDistBase  ()                          = default;
    public   : ParamFuncsDistBase           (const ParamFuncsDistBase&) = default;
    public   : ParamFuncsDistBase& operator=(const ParamFuncsDistBase&) = default;
    public   : ParamFuncsDistBase           (ParamFuncsDistBase&&)      = delete;
    public   : ParamFuncsDistBase& operator=(ParamFuncsDistBase&&)      = delete;
    
    public   : using ParamFuncsDistBaseCRTP<ParramFuncsDistBaseType>::setDistCfMode;
    public   : using ParamFuncsDistBaseCRTP<ParramFuncsDistBaseType>::setEvalDist;
    public   : using ParamFuncsDistBaseCRTP<ParramFuncsDistBaseType>::computeS;
    public   : using ParamFuncsDistBaseCRTP<ParramFuncsDistBaseType>::computeT;
    public   : using ParamFuncsDistBaseCRTP<ParramFuncsDistBaseType>::computeS2TLattice;
    
    private  : void     setDistCfModeImplCRTP     ( TraveledDistCfMode _distCfMode, const std::vector<std::size_t>& _distRelFuncIdx ) { thisDerived().setDistCfModeImpl(_distCfMode, _distRelFuncIdx); }
    private  : void     setEvalDistImplCRTP       ( const TNumType& _funcsDistEval, const EvalArcGuarantee& _eAG )                    { thisDerived().setEvalDistImpl(_funcsDistEval, _eAG); }
    private  : TNumType computeSImplCRTP          ()                                                                            const { return thisDerived().computeSImpl(); }
    private  : TNumType computeTImplCRTP          ( const TNumType& _s, const EvalArcGuarantee& _eAG )                                { return thisDerived().computeTImpl(_s, _eAG); }
    private  : void     computeS2TLatticeImplCRTP ( const std::vector<TNumType>& _sLattice, std::vector<TNumType>& _tLattice )     { thisDerived().computeS2TLatticeImpl(_sLattice, _tLattice); }
    private  : void     computeS2TLatticeImplCRTP ( const TNumType& _arc0, const TNumType& _ds, std::vector<TNumType>& _tLattice ) { thisDerived().computeS2TLatticeImpl(_arc0, _ds, _tLattice); }
    
    private  : virtual void     setDistCfModeImplVirt     ( TraveledDistCfMode _distCfMode, const std::vector<std::size_t>& _distRelFuncIdx ) override final { thisDerived().setDistCfModeImpl(_distCfMode, _distRelFuncIdx); }
    private  : virtual void     setEvalDistImplVirt       ( const TNumType& _funcsDistEval, const EvalArcGuarantee& _eAG ) override final { thisDerived().setEvalDistImpl(_funcsDistEval, _eAG); }
    private  : virtual TNumType computeSImplVirt          () const override final { return thisDerived().computeSImpl(); }
    private  : virtual TNumType computeTImplVirt          ( const TNumType& _s, const EvalArcGuarantee& _eAG ) override final { return thisDerived().computeTImpl(_s, _eAG); }
    private  : virtual void     computeS2TLatticeImplVirt ( const std::vector<TNumType>& _sLattice, std::vector<TNumType>& _tLattice ) override final { 
	thisDerived().computeS2TLatticeImpl(_sLattice, _tLattice); 
    }
    private  : virtual void     computeS2TLatticeImplVirt ( const TNumType& _arc0, const TNumType& _ds, std::vector<TNumType>& _tLattice ) override final { 
	thisDerived().computeS2TLatticeImpl(_arc0, _ds, _tLattice);
    }
	
    private  :       TDerived& thisDerived()       { return static_cast<      TDerived&>(*this); }
    private  : const TDerived& thisDerived() const { return static_cast<const TDerived&>(*this); }
    
    template<typename TDerived2> friend class ParamFuncsDistBaseCRTP;
    template<typename TNum2    > friend class ParamFuncsDistBaseVirt;
};


namespace {

template<typename TDerived, typename TNumType>
struct ParamFuncsDistBaseCRTPTraits<ParamFuncsDistBase<TDerived, TNumType>> {
    using NumType = TNumType;
};

} //namespace <anonymous>


}

#endif // PARAM_FUNC_DIST_HPP
