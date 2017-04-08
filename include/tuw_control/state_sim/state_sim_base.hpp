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

#ifndef STATE_SIM_BASE_HPP
#define STATE_SIM_BASE_HPP

#include <tuw_control/param_func_new/param_func_dist.hpp>
#include <tuw_control/discretization/discretization_options.hpp>
#include <eigen3/Eigen/Eigen>

namespace tuw {

namespace /*<anonymous>*/ {
    
using PfEaG = EvalArcGuarantee;
    
template<class TDerived> struct StateMapSimBaseTraits;
template<class TDerived>
struct StateMapSimBaseTraits {
    public   : using StateType              = typename StateMapSimBaseTraits<typename TDerived::StateMapSimBaseType>::StateType;
    public   : using NumType                = typename StateMapSimBaseTraits<typename TDerived::StateMapSimBaseType>::NumType;
    public   : using StateCfType            = typename StateMapSimBaseTraits<typename TDerived::StateMapSimBaseType>::StateCfType;
    public   : using StateNmType            = typename StateMapSimBaseTraits<typename TDerived::StateMapSimBaseType>::StateNmType;
    public   : using StateVirtType          = typename StateMapSimBaseTraits<typename TDerived::StateMapSimBaseType>::StateVirtType;
    public   : using StateNmNumType         = typename StateMapSimBaseTraits<typename TDerived::StateMapSimBaseType>::StateNmNumType;
    public   : using StateWithGradNmType    = typename StateMapSimBaseTraits<typename TDerived::StateMapSimBaseType>::StateWithGradNmType;
    public   : using StateWithGradNmNumType = typename StateMapSimBaseTraits<typename TDerived::StateMapSimBaseType>::StateWithGradNmNumType;
};

template<class TDerived> struct StateSimBaseCRTPTraits;

class EmptyGradType{};

} //namespace <anonymous>

template<class TDerived>
class StateSimBaseCRTP {
    public   : using NumType      = typename StateSimBaseCRTPTraits<TDerived>::NumType;
    public   : using StateType    = typename StateSimBaseCRTPTraits<TDerived>::StateType;
    public   : using StateNmType  = typename StateSimBaseCRTPTraits<TDerived>::StateNmType;
    public   : static constexpr const bool hasStateGrad = !std::is_same<EmptyGradType, typename StateSimBaseCRTPTraits<TDerived>::StateWithGradNmType>::value;
    
    public   : StateSimBaseCRTP           ()                        = default;
    public   : ~StateSimBaseCRTP          ()                        = default;
    public   : StateSimBaseCRTP           (const StateSimBaseCRTP&) = default;
    public   : StateSimBaseCRTP& operator=(const StateSimBaseCRTP&) = default;
    public   : StateSimBaseCRTP           (StateSimBaseCRTP&&)      = default;
    public   : StateSimBaseCRTP& operator=(StateSimBaseCRTP&&)      = default;
    
    public   : template< bool stateGradientRepresentation = hasStateGrad, 
	                 typename std::enable_if< ( stateGradientRepresentation ) >::type* = nullptr > 
	       void                  advanceWithGrad  ( const double& _arc )                          { thisDerived().advanceWithGradImplCRTP      (_arc);       }
    public   : void                  advance          ( const double& _arc )                          { thisDerived().advanceImplCRTP              (_arc);       }
    public   : void                  toState0         ()                                              { thisDerived().toState0ImplCRTP             ();           }
    public   :       StateType&      state0           ()                                              { return thisDerived().state0ImplCRTP        ();           }
    public   : const StateType&      state0           ()                                        const { return thisDerived().state0ImplCRTP        ();           }
    public   :       StateType&      state            ()                                              { return thisDerived().stateImplCRTP         ();           }
    public   : const StateType&      state            ()                                        const { return thisDerived().stateImplCRTP         ();           }
    public   :       double          stateArc         ()                                              { return thisDerived().stateArcImplCRTP      ();           }
    public   :       double          stateDist        ()                                              { return thisDerived().stateDistImplCRTP     ();           }
    public   : void                  advanceSet0      ( const double& _tEnd, const double& _dt )      { thisDerived().advanceSet0ImplCRTP          (_tEnd, _dt); }
    public   :       ParamFuncsBaseVirt<NumType>*     paramFuncs       ()                                              { return thisDerived().paramFuncsImplCRTP    ();           }
    public   : const ParamFuncsBaseVirt<NumType>*     paramFuncs       ()                                        const { return thisDerived().paramFuncsImplCRTP    ();           }
    public   :       ParamFuncsDistBaseVirt<NumType>* paramFuncsDist   ()                                              { return thisDerived().paramFuncsDistImplCRTP();           }
    public   : const ParamFuncsDistBaseVirt<NumType>* paramFuncsDist   ()                                        const { return thisDerived().paramFuncsDistImplCRTP();           }
    
    private  :       TDerived& thisDerived()       { return static_cast<      TDerived&>(*this); }
    private  : const TDerived& thisDerived() const { return static_cast<const TDerived&>(*this); }
};
    
    
template<class TNumType, class StateVirtType>
class StateSimBaseVirt {
    //special class member functions
    public   : StateSimBaseVirt           ()                        = default;
    public   : ~StateSimBaseVirt          ()                        = default;
    public   : StateSimBaseVirt           (const StateSimBaseVirt&) = default;
    public   : StateSimBaseVirt& operator=(const StateSimBaseVirt&) = default;
    public   : StateSimBaseVirt           (StateSimBaseVirt&&)      = default;
    public   : StateSimBaseVirt& operator=(StateSimBaseVirt&&)      = default;
    
    public   : void                  advance          ( const TNumType& _arc )                             { advanceImplVirt              (_arc);       }
    public   : void                  advanceWithGrad  ( const TNumType& _arc )                             { advanceWithGradImplVirt      (_arc);       }
    public   : void                  toState0         ()                                                   { toState0ImplVirt             ();           }
    public   :       StateVirtType&  state0           ()                                                   { return state0ImplVirt        ();           }
    public   : const StateVirtType&  state0           ()                                             const { return state0ImplVirt        ();           }
    public   :       StateVirtType&  state            ()                                                   { return stateImplVirt         ();           }
    public   : const StateVirtType&  state            ()                                             const { return stateImplVirt         ();           }
    public   : TNumType              stateArc         ()                                                   { return stateArcImplVirt      ();           }
    public   : TNumType              stateDist        ()                                                   { return stateDistImplVirt     ();           }
    public   : void                  advanceSet0      ( const TNumType& _tEnd, const TNumType& _dt )       { advanceSet0ImplVirt          (_tEnd, _dt); }
    public   :       ParamFuncsBaseVirt<TNumType>*     paramFuncs       ()                                                   { return paramFuncsImplVirt    ();           }
    public   : const ParamFuncsBaseVirt<TNumType>*     paramFuncs       ()                                             const { return paramFuncsImplVirt    ();           }
    public   :       ParamFuncsDistBaseVirt<TNumType>* paramFuncsDist   ()                                                   { return paramFuncsDistImplVirt();           }
    public   : const ParamFuncsDistBaseVirt<TNumType>* paramFuncsDist   ()                                             const { return paramFuncsDistImplVirt();           }
    
    private  : virtual void                  advanceImplVirt          ( const TNumType& _arc )                             = 0;
    private  : virtual void                  advanceWithGradImplVirt  ( const TNumType& _arc )                             = 0;
    private  : virtual void                  toState0ImplVirt         ()                                                   = 0;
    private  : virtual       StateVirtType&  state0ImplVirt           ()                                                   = 0;
    private  : virtual const StateVirtType&  state0ImplVirt           ()                                             const = 0;
    private  : virtual       StateVirtType&  stateImplVirt            ()                                                   = 0;
    private  : virtual const StateVirtType&  stateImplVirt            ()                                             const = 0;
    private  : virtual TNumType              stateArcImplVirt         ()                                                   = 0;
    private  : virtual TNumType              stateDistImplVirt        ()                                                   = 0;
    private  : virtual void                  advanceSet0ImplVirt      ( const TNumType& _tEnd, const TNumType& _dt )       = 0;
    private  : virtual       ParamFuncsBaseVirt<TNumType>*     paramFuncsImplVirt       ()                                                   = 0;
    private  : virtual const ParamFuncsBaseVirt<TNumType>*     paramFuncsImplVirt       ()                                             const = 0;
    private  : virtual       ParamFuncsDistBaseVirt<TNumType>* paramFuncsDistImplVirt   ()                                                   = 0;
    private  : virtual const ParamFuncsDistBaseVirt<TNumType>* paramFuncsDistImplVirt   ()                                             const = 0;
};

template<class NumType, class StateWithGradNmNumType, template<class> class  TDiscretizationType>
class OdeStateSolverRealAlias {
    using OdeStateGradSolverType = explicit_generic_rk_impl<TDiscretizationType, 
							    StateWithGradNmNumType, 
							    NumType,        
							    StateWithGradNmNumType, 
							    NumType, 
							    odeint::vector_space_algebra>;
    private  : std::shared_ptr<OdeStateGradSolverType> rkGrad_;
    
    template<class TDerived2, class TStateType2, template<class> class  TDiscretizationType2> friend class StateSimBase;
};

class OdeStateSolverDummyAlias {
    using OdeStateGradSolverType = EmptyGradType;
    
    template<class TDerived2, class TStateType2, template<class> class  TDiscretizationType2> friend class StateSimBase;
};

template<class TDerived, class TStateType, template<class> class  TDiscretizationType>
class StateSimBase : public StateSimBaseCRTP<StateSimBase<TDerived, TStateType, TDiscretizationType>>, 
                     public StateSimBaseVirt<typename StateMapSimBaseTraits<typename TStateType::StateMapSimBaseType>::NumType, 
			                     typename StateMapSimBaseTraits<typename TStateType::StateMapSimBaseType>::StateVirtType>,
		     public std::conditional <!std::is_same<EmptyGradType, typename StateMapSimBaseTraits<typename TStateType::StateMapSimBaseType>::StateWithGradNmType>::value, 
			                      OdeStateSolverRealAlias<typename StateMapSimBaseTraits<typename TStateType::StateMapSimBaseType>::NumType,
					                              typename StateMapSimBaseTraits<typename TStateType::StateMapSimBaseType>::StateWithGradNmNumType,
								      TDiscretizationType>, 
					      OdeStateSolverDummyAlias >::type {
						  
    public   : using StateSimBaseType       = StateSimBase<TDerived, TStateType, TDiscretizationType>;
    public   : using NumType                = typename StateMapSimBaseTraits<typename TStateType::StateMapSimBaseType>::NumType;
    public   : using StateVirtType          = typename StateMapSimBaseTraits<typename TStateType::StateMapSimBaseType>::StateVirtType;
    public   : using StateType              = TStateType;
    public   : using StateNmType            = typename StateMapSimBaseTraits<typename TStateType::StateMapSimBaseType>::StateNmType;
    public   : using StateNmNumType         = typename StateMapSimBaseTraits<typename TStateType::StateMapSimBaseType>::StateNmNumType;
    public   : using StateWithGradNmType    = typename StateMapSimBaseTraits<typename TStateType::StateMapSimBaseType>::StateWithGradNmType;
    public   : using StateWithGradNmNumType = typename StateMapSimBaseTraits<typename TStateType::StateMapSimBaseType>::StateWithGradNmNumType;
    
    public   : static constexpr const bool hasStateGrad = !std::is_same<EmptyGradType, StateWithGradNmType>::value;
    
    public   : using OdeStateSolverType     = explicit_generic_rk_impl<TDiscretizationType, 
	                                                               StateNmNumType, 
								       NumType,        
								       StateNmNumType, 
								       NumType, 
								       odeint::vector_space_algebra>;
    
    
    
    //special class member functions
    public   : StateSimBase           ()                    = default;
    public   : ~StateSimBase          ()                    = default;
    public   : StateSimBase           (const StateSimBase&) = default;
    public   : StateSimBase& operator=(const StateSimBase&) = default;
    public   : StateSimBase           (StateSimBase&&)      = delete;
    public   : StateSimBase& operator=(StateSimBase&&)      = delete;
    
    public   : using StateSimBaseCRTP<StateSimBaseType>::advance;
    public   : using StateSimBaseCRTP<StateSimBaseType>::advanceWithGrad;
    public   : using StateSimBaseCRTP<StateSimBaseType>::toState0;
    public   : using StateSimBaseCRTP<StateSimBaseType>::state0;
    public   : using StateSimBaseCRTP<StateSimBaseType>::state;
    public   : using StateSimBaseCRTP<StateSimBaseType>::stateArc;
    public   : using StateSimBaseCRTP<StateSimBaseType>::stateDist;
    public   : using StateSimBaseCRTP<StateSimBaseType>::advanceSet0;
    public   : using StateSimBaseCRTP<StateSimBaseType>::paramFuncs;
    public   : using StateSimBaseCRTP<StateSimBaseType>::paramFuncsDist;
    
    private  : void                  advanceImplVirt          ( const NumType& _arc )                            override final { advanceImplCRTP(_arc); }
    private  : void                  advanceWithGradImplVirt  ( const NumType& _arc )                            override final { advanceWithGradImplVirtDispatch(_arc); }
    private  : void                  toState0ImplVirt         ()                                                 override final { toState0ImplCRTP(); }
    private  :       StateVirtType&  state0ImplVirt           ()                                                 override final { return state0ImplCRTP(); }
    private  : const StateVirtType&  state0ImplVirt           ()                                           const override final { return state0ImplCRTP(); }
    private  :       StateVirtType&  stateImplVirt            ()                                                 override final { return stateImplCRTP(); }
    private  : const StateVirtType&  stateImplVirt            ()                                           const override final { return stateImplCRTP(); }
    private  : NumType               stateArcImplVirt         ()                                                 override final { return stateArcImplCRTP(); }
    private  : NumType               stateDistImplVirt        ()                                                 override final { return stateDistImplCRTP(); }
    private  : void                  advanceSet0ImplVirt      ( const NumType& _tEnd, const NumType& _dt )       override final { advanceSet0ImplCRTP(_tEnd, _dt); }
    private  :       ParamFuncsBaseVirt<NumType>*     paramFuncsImplVirt       ()                                                 override final { return paramFuncsImplCRTP(); }
    private  : const ParamFuncsBaseVirt<NumType>*     paramFuncsImplVirt       ()                                           const override final { return paramFuncsImplCRTP(); }
    private  :       ParamFuncsDistBaseVirt<NumType>* paramFuncsDistImplVirt   ()                                                 override final { return paramFuncsDistImplCRTP(); }
    private  : const ParamFuncsDistBaseVirt<NumType>* paramFuncsDistImplVirt   ()                                           const override final { return paramFuncsDistImplCRTP(); }
    
    private   : template< bool stateGradientRepresentation = hasStateGrad, 
	                  typename std::enable_if< (  stateGradientRepresentation ) >::type* = nullptr > 
		void advanceWithGradImplVirtDispatch ( const NumType& _arc ) { advanceWithGradImplCRTP ( _arc ); }
    private   : template< bool stateGradientRepresentation = hasStateGrad, 
	                  typename std::enable_if< ( !stateGradientRepresentation ) >::type* = nullptr > 
		void advanceWithGradImplVirtDispatch ( const NumType& _arc ) { throw std::runtime_error("Cannot advance with gradient info (State class not suited)"); }
    
    public   :       StateType& state0ImplCRTP ()       { return state0_; }
    public   : const StateType& state0ImplCRTP () const { return state0_; }
    public   :       StateType& stateImplCRTP  ()       { return state_; }
    public   : const StateType& stateImplCRTP  () const { return state_; }
    public   : void advanceImplCRTP ( const NumType& _arc ) {
	rk_->do_step( [this](const StateNmNumType& _x, StateNmNumType& _dxdt, const NumType _t){
			setStateCfNmStep ( _t, PfEaG::/*NONE*/AFTER_LAST );
			state_.stateNm().data() = _x;
			_dxdt = stateNmDot().data();
		      }, 
		      state_.stateNm().data(), 
		      arcOld_, 
		      _arc-arcOld_ );
	arcOld_ = _arc;
	setStateCf ( _arc, PfEaG::/*NONE*/AFTER_LAST );
    }
    
    public   :  template< bool stateGradientRepresentation = hasStateGrad, 
	                  typename std::enable_if< ( stateGradientRepresentation ) >::type* = nullptr > 
		void advanceWithGradImplCRTP ( const NumType& _arc ) {
		    this->rkGrad_->do_step( [this](const StateWithGradNmNumType& _x, StateWithGradNmNumType& _dxdt, const NumType _t){
					    setStateCfWithGradNmStep ( _t, PfEaG::/*NONE*/AFTER_LAST );
					    state_.stateWithGradNm().data() = _x;
					    _dxdt = stateWithGradNmDot().data();
					}, 
					state_.stateWithGradNm().data(), 
					arcOld_, 
					_arc-arcOld_ );
		    arcOld_ = _arc;
		    setStateCfWithGrad ( _arc, PfEaG::/*NONE*/AFTER_LAST );
		}
    public   :  template< bool stateGradientRepresentation = hasStateGrad, 
	                  typename std::enable_if< ( !stateGradientRepresentation ) >::type* = nullptr > 
		void toState0ImplCRTP () {
		    state_ = state0_;
		    setStateCf (0, PfEaG::AT_BEGIN/*NONE*/);
		    rk_ = std::make_shared<OdeStateSolverType>(); rk_->adjust_size(state_.stateNm().data().size());
		    arcOld_ = 0;
		}
    public   :  template< bool stateGradientRepresentation = hasStateGrad, 
	                  typename std::enable_if< (  stateGradientRepresentation ) >::type* = nullptr > 
		void toState0ImplCRTP () {
		    state_ = state0_;
		    setStateCfWithGrad (0, PfEaG::AT_BEGIN/*NONE*/);
		    for(size_t i = 0; i < state_.stateGradNm().subSize(); ++i) { stateWithGradNmDotCache_.template sub<1>().setPBlockSize(i, state_.stateGradNm().dxdpBlockI(i).subSize() ); }
		    rk_           = std::make_shared<OdeStateSolverType>(); 
		    this->rkGrad_ = std::make_shared<typename OdeStateSolverRealAlias<NumType, StateWithGradNmNumType, TDiscretizationType>::OdeStateGradSolverType>(); 
		    
		    rk_          ->adjust_size(state_.stateNm        ().data().size());
		    this->rkGrad_->adjust_size(state_.stateWithGradNm().data().size());
		    arcOld_ = 0;
		}
    
    public   : void advanceSet0ImplCRTP ( const NumType& _tEnd, const NumType& _dt ) {
	toState0();
	NumType tSim = 0;
	while(tSim + _dt <= _tEnd){ tSim += _dt; advance(tSim); }
	advance(_tEnd);
	state0_.data() = state_.data();
    }
    
    public   :       ParamFuncsBaseVirt<NumType>*     paramFuncsImplCRTP       ()                              { return thisSimDerived().paramFuncsImpl(); }
    public   : const ParamFuncsBaseVirt<NumType>*     paramFuncsImplCRTP       ()                        const { return thisSimDerived().paramFuncsImpl(); }
    public   :       ParamFuncsDistBaseVirt<NumType>* paramFuncsDistImplCRTP   ()                              { return thisSimDerived().paramFuncsDistImpl(); }
    public   : const ParamFuncsDistBaseVirt<NumType>* paramFuncsDistImplCRTP   ()                        const { return thisSimDerived().paramFuncsDistImpl(); }
    
    public   : NumType               stateArcImplCRTP         ()                                               { return thisSimDerived().stateArcImpl(); }
    public   : NumType               stateDistImplCRTP        ()                                               { return thisSimDerived().stateDistImpl(); }
    public   : void                  setStateCf               ( const NumType& _arc, const PfEaG& _eAG )       { thisSimDerived().setStateCfImpl              (_arc, _eAG); }
    public   : void                  setStateCfNmStep         ( const NumType& _arc, const PfEaG& _eAG )       { thisSimDerived().setStateCfNmStepImpl        (_arc, _eAG); }
    public   : void                  setStateCfWithGrad       ( const NumType& _arc, const PfEaG& _eAG )       { thisSimDerived().setStateCfWithGradImpl      (_arc, _eAG); }
    public   : void                  setStateCfWithGradNmStep ( const NumType& _arc, const PfEaG& _eAG )       { thisSimDerived().setStateCfWithGradNmStepImpl(_arc, _eAG); }
    public   : StateNmType&          stateNmDot               ()                                               { return thisSimDerived().stateNmDotImpl(); }
    public   : template<  bool stateGradientRepresentation = hasStateGrad, 
	                  typename std::enable_if< ( stateGradientRepresentation ) >::type* = nullptr > 
	       StateWithGradNmType&  stateWithGradNmDot       ()                                               { return thisSimDerived().stateWithGradNmDotImpl(); }
    
    public   : NumType arcOld_;
    private  : std::shared_ptr<OdeStateSolverType    > rk_;
    public   : StateNmType         stateNmDotCache_;
    public   : StateWithGradNmType stateWithGradNmDotCache_;
    private  : StateType   state0_;
    private  : StateType   state_;
    
    private  :       TDerived& thisSimDerived()       { return static_cast<      TDerived&>(*this); }
    private  : const TDerived& thisSimDerived() const { return static_cast<const TDerived&>(*this); }
};



namespace /*<anonymous>*/ {
    
template<class TDerived, class TStateType, template<class> class TDiscretizationType>
struct StateSimBaseCRTPTraits<StateSimBase<TDerived, TStateType, TDiscretizationType>> {
    public   : using StateType               = TStateType;
    public   : using NumType                 = typename StateMapSimBaseTraits<typename TStateType::StateMapSimBaseType>::NumType;
    public   : using StateCfType             = typename StateMapSimBaseTraits<typename TStateType::StateMapSimBaseType>::StateCfType;
    public   : using StateNmType             = typename StateMapSimBaseTraits<typename TStateType::StateMapSimBaseType>::StateNmType;
    public   : using StateVirtType           = typename StateMapSimBaseTraits<typename TStateType::StateMapSimBaseType>::StateVirtType;
    public   : using StateNmNumType          = typename StateMapSimBaseTraits<typename TStateType::StateMapSimBaseType>::StateNmNumType;
    public   : using StateWithGradNmType     = typename StateMapSimBaseTraits<typename TStateType::StateMapSimBaseType>::StateWithGradNmType;
    public   : using StateWithGradNmNumType  = typename StateMapSimBaseTraits<typename TStateType::StateMapSimBaseType>::StateWithGradNmNumType;
};

} //namespace <anonymous>

}

#endif // STATE_SIM_BASE_HPP