/***************************************************************************
 *   Software License Agreement (BSD License)                              *
 *   Copyright (C) 2017 by Horatiu George Todoran <todorangrg@gmail.com>   *
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

#ifndef AGV_DIFF_DRIVE_V_W_HPP
#define AGV_DIFF_DRIVE_V_W_HPP

#include <tuw_control/state_sim/state_sim.hpp>
#include <tuw_control/param_func_new/param_func_spline/param_func_spline0_dist.hpp>
#include <tuw_control/utils.h>
#include <grid_map_ros/grid_map_ros.hpp>

namespace tuw {

    
namespace DiffDrive {
    
// Defining the system state


template<class TNumType, class TLeafType>
class StateNmLVW : public StateMapArray<TNumType, TLeafType, 5> {
    public   : using StateMapArray<TNumType, TLeafType, 5>::StateMapArray;
    public   : auto&       dThetaSqr()       { return this->template sub<0>(); }
    public   : const auto& dThetaSqr() const { return this->template sub<0>(); }
    public   : auto&       pDevSqr ()       { return this->template sub<1>(); }
    public   : const auto& pDevSqr () const { return this->template sub<1>(); }
    public   : auto&       rhoSqr ()       { return this->template sub<2>(); }
    public   : const auto& rhoSqr () const { return this->template sub<2>(); }
    public   : auto&       avSqr  ()       { return this->template sub<3>(); }
    public   : const auto& avSqr  () const { return this->template sub<3>(); }
    public   : auto&       prevPDevSqr ()       { return this->template sub<4>(); }
    public   : const auto& prevPDevSqr () const { return this->template sub<4>(); }
    
};
    
/*!@class StateRobotNmVW
 * @brief Defining the system numerical state variables.
 * 
 * Has to allways be an extended class from @ref StateMapArray, @ref StateMapVector or @ref StateMapTuple
 * 
 * @tparam TNumType Numerical type used internally
 * @tparam TLeafType The leaf type. This has to be templated as the numerical state jacobian class will use another (non-numerical) leaf type
 */
template<class TNumType, class TLeafType>
class StateRobotNmVW : public StateMapArray<TNumType, TLeafType, 2> {
    public   : using StateMapArray<TNumType, TLeafType, 2>::StateMapArray;
    public   : auto&       x    ()       { return this->template sub<0>(); }
    public   : const auto& x    () const { return this->template sub<0>(); }
    public   : auto&       y    ()       { return this->template sub<1>(); }
    public   : const auto& y    () const { return this->template sub<1>(); }
    public   : auto&       state()       { return *this; }
    public   : const auto& state() const { return *this; }
};


template<class TNumType, class TLeafType>
class StatePersonNmVW : public StateMapArray<TNumType, TLeafType, 3> {
    public   : using StateMapArray<TNumType, TLeafType, 3>::StateMapArray;
    public   : auto&       x    ()       { return this->template sub<0>(); }
    public   : const auto& x    () const { return this->template sub<0>(); }
    public   : auto&       y    ()       { return this->template sub<1>(); }
    public   : const auto& y    () const { return this->template sub<1>(); }
    public   : auto&       theta()       { return this->template sub<2>(); }
    public   : const auto& theta() const { return this->template sub<2>(); }
    public   : auto&       state()       { return *this; }
    public   : const auto& state() const { return *this; }
};

template<class TNumType, class TLeafType>
class StatePersonsNmVW : public StateMapVector<TNumType, StatePersonNmVW<TNumType,TLeafType>> {
    public   : using StateMapVector<TNumType, StatePersonNmVW<TNumType,TLeafType>>::StateMapVector;
    public   : auto&       state()       { return *this; }
    public   : const auto& state() const { return *this; }
};


/*!@class StateNmWithLVW
 * @brief Defining the system numerical state variables, including a scalar objective function L.
 * 
 * Has to allways be an extended class from @ref StateMapArray, @ref StateMapVector or @ref StateMapTuple
 * 
 * @tparam TNumType Numerical type used internally
 * @tparam TLeafType The leaf type. This has to be templated as the numerical state jacobian class will use another (non-numerical) leaf type
 */
template<class TNumType, class TLeafType>
class StateNmWithLVW : public StateMapTuple<TNumType, StateNmLVW<TNumType, TLeafType>, StateRobotNmVW<TNumType, TLeafType>, StatePersonsNmVW<TNumType, TLeafType>> {
    public   : using StateMapTuple<TNumType, StateNmLVW<TNumType, TLeafType>, StateRobotNmVW<TNumType, TLeafType>, StatePersonsNmVW<TNumType, TLeafType>>::StateMapTuple;
    public   : auto&       L      ()       { return this->template sub<0>(); }
    public   : const auto& L      () const { return this->template sub<0>(); }
    public   : auto&       x      ()       { return this->template sub<1>().x(); }
    public   : const auto& x      () const { return this->template sub<1>().x(); }
    public   : auto&       y      ()       { return this->template sub<1>().y(); }
    public   : const auto& y      () const { return this->template sub<1>().y(); }
    public   : auto&       persons()       { return this->template sub<2>(); }
    public   : const auto& persons() const { return this->template sub<2>(); }
    public   : auto&       state  ()       { return *this; }
    public   : const auto& state  () const { return *this; }
    public   : auto& var  (size_t i)       { if(i < this->template sub<0>().subSize()) 
                                                return this->template sub<0>().sub(i);
                                             else if(i<this->template sub<0>().subSize()+this->template sub<1>().subSize()) 
                                                return this->template sub<1>().sub(i-this->template sub<0>().subSize());
                                             else { 
                                                const size_t idx = i-this->template sub<0>().subSize()-this->template sub<1>().subSize();
                                                return this->template sub<2>().sub(idx/3).sub(idx%3);
                                             }
                                           }
    public   : size_t varSize     () const { return this->template sub<0>().subSize()+this->template sub<1>().subSize()+this->template sub<2>().subSize()*3; } 
};

template<class TNumType, class TLeafType>
class StateNmVW : public StateMapTuple<TNumType, StateRobotNmVW<TNumType, TLeafType>, StatePersonsNmVW<TNumType, TLeafType>> {
    public   : using StateMapTuple<TNumType, StateRobotNmVW<TNumType, TLeafType>, StatePersonsNmVW<TNumType, TLeafType>> ::StateMapTuple;
    public   : auto&       x      ()       { return this->template sub<0>().x(); }
    public   : const auto& x      () const { return this->template sub<0>().x(); }
    public   : auto&       y      ()       { return this->template sub<0>().y(); }
    public   : const auto& y      () const { return this->template sub<0>().y(); }
    public   : auto&       persons()       { return this->template sub<1>(); }
    public   : const auto& persons() const { return this->template sub<1>(); }
    public   : auto&       state  ()       { return *this; }
    public   : const auto& state  () const { return *this; }
    public   : auto& var  (size_t i)       { if(i < this->template sub<0>().subSize()) { return this->template sub<0>().sub(i); }
                                             else {
                                                const size_t idx = i-this->template sub<0>().subSize();
                                                return this->template sub<1>().sub(idx/3).sub(idx%3);
                                             }
                                           }
    public   : size_t varSize     () const { return this->template sub<0>().subSize()+this->template sub<1>().subSize()*3; }  
};
/*!@class StateCfVW
 * @brief Defining the system closed-form state variables.
 * 
 * Has to allways be an extended class from @ref StateMapArray, @ref StateMapVector or @ref StateMapTuple
 * 
 * @tparam TNumType Numerical type used internally
 * @tparam TLeafType The leaf type. This has to be templated as the closed-form state jacobian class will use another (non-numerical) leaf type
 */
template<class TNumType, class TLeafType>
class StateCfVW : public StateMapArray<TNumType, TLeafType, 7> {
    public   : using StateMapArray<TNumType, TLeafType, 7>::StateMapArray;
    public   : auto&       theta()       { return this->template sub<0>(); }
    public   : const auto& theta() const { return this->template sub<0>(); }
    public   : auto&       v    ()       { return this->template sub<1>(); }
    public   : const auto& v    () const { return this->template sub<1>(); }
    public   : auto&       w    ()       { return this->template sub<2>(); }
    public   : const auto& w    () const { return this->template sub<2>(); }
    public   : auto&       av   ()       { return this->template sub<3>(); }
    public   : const auto& av   () const { return this->template sub<3>(); }
    public   : auto&       aw   ()       { return this->template sub<4>(); }
    public   : const auto& aw   () const { return this->template sub<4>(); }
    public   : auto&       t    ()       { return this->template sub<5>(); }
    public   : const auto& t    () const { return this->template sub<5>(); }
    public   : auto&       s    ()       { return this->template sub<6>(); }
    public   : const auto& s    () const { return this->template sub<6>(); }
    public   : auto& var(size_t i)       { return this->sub(i); }
    public   : size_t varSize() const    { return this->subSize(); }   
};

static constexpr const size_t optParamBlockSize = 3;

/*!@class OptVarStructVW
 * @brief Defining the system optimization parameters structure
 * 
 * Has to allways be an extended class from @ref StateMapArray, @ref StateMapVector or @ref StateMapTuple
 * 
 * @tparam TNumType Numerical type used internally
 * @tparam TLeafType The leaf type.
 */
template<class TNumType, typename TLeafType>
class OptVarStructVW : public StateMapArray<TNumType, StateMapVector<TNumType, TLeafType>, optParamBlockSize> {
    public   : using StateMapArray<TNumType, StateMapVector<TNumType, TLeafType>, optParamBlockSize>::StateMapArray;
    public   : auto&        optParamV()       { return this->template sub<0>(); }//parameters influencing linear velocity
    public   : const auto&  optParamV() const { return this->template sub<0>(); }
    public   : auto&        optParamW()       { return this->template sub<1>(); }//parameters influencing angular velocity
    public   : const auto&  optParamW() const { return this->template sub<1>(); }
    public   : auto&        optParamT()       { return this->template sub<2>(); }//parameters influencing temporal location of previous parameters
    public   : const auto&  optParamT() const { return this->template sub<2>(); }
};

///In the following, we will define various types of states, making use of the meta-class @ref StateMap.
///@todo make it work with "Empty" numerical / closed-form types

///Simple full system state [xNm,xCf].
template<class TNumType> using StateVW              = StateMap        < TNumType, StateNmVW     , StateCfVW>;
///Full system state including scalar objective function [[xNm,L],xCf].
template<class TNumType> using StateWithLVW         = StateMap        < TNumType, StateNmWithLVW, StateCfVW>;
///Full system state, with derivatives [[xNm,L],xCf, d/dp([xNm,L]), d/dp(xCf)].
template<class TNumType> using StateWithGradVW      = StateWithGradMap< TNumType, StateNmVW     , StateCfVW, OptVarStructVW>;
///Full system state including scalar objective function, with derivatives [[xNm,L],xCf, d/dp([xNm,L]), d/dp(xCf)].
template<class TNumType> using StateWithLWithGradVW = StateWithGradMap< TNumType, StateNmWithLVW, StateCfVW, OptVarStructVW>;

/*!@class ParamType
 * @brief Defining the system optimization parameters structure
 * 
 * It can have arbitrary structure. However, it has to posess the member variables @ref state0, @ref paramFuncs, @ref cfData.
 * 
 * @tparam TNumType     Numerical type used internally
 * @tparam TCfDataType  The type of the map data object.
 */
template<class TNumType, class TCfDataType>
struct ParamType {
    ParamFuncsSpline0Dist<TNumType,2,1> paramFuncs;
    StateVW<TNumType>          state0;
    TCfDataType                         cfData;
    enum class ParamFuncVars{ V, W };
    
};


template<class TNumType, class MapDataType, class TStateType, template<class> class TDiscretizationType, class... TFuncsType>
class StateSimVWBase : public StateSimBase< StateSimVWBase<TNumType, MapDataType, TStateType, TDiscretizationType, TFuncsType...>, 
                                            ParamType<TNumType, MapDataType>, 
					    TStateType, 
					    TDiscretizationType, 
					    TFuncsType...> {
    using PFV = typename ParamType<TNumType, MapDataType>::ParamFuncVars;
    
    public   : void adjustXSizeImpl(auto& _XNm, auto& _XCf) {
        _XNm.persons().subResize(this->paramStruct->state0.stateNm().persons().subSize());
	this->paramStruct->paramFuncs.precompute();
    }
    public   : void setXNm0Impl(auto& _XNm0) {
	for(int i = 0; i < _XNm0.data().size(); ++i) { _XNm0.data()(i) = 0; }
	_XNm0.x() = this->paramStruct->state0.stateNm().x();
	_XNm0.y() = this->paramStruct->state0.stateNm().y();
        for(size_t i = 0; i<_XNm0.persons().subSize();i++) {
            _XNm0.persons().sub(i).x() = this->paramStruct->state0.stateNm().persons().sub(i).x();
            _XNm0.persons().sub(i).y() = this->paramStruct->state0.stateNm().persons().sub(i).y();
            _XNm0.persons().sub(i).theta() = this->paramStruct->state0.stateNm().persons().sub(i).theta();
        }
    }
    //same as before, but all closed-form variables have to be computed
    public  : void setXCfImpl ( auto& _XCf, const TNumType& _arc, const PfEaG& _eAG ) {
	setXCfNmStep(_XCf, _arc, _eAG);
	auto& paramFuncs = this->paramStruct->paramFuncs;
	_XCf.w () = paramFuncs.computeFuncVal  (asInt(PFV::W));
	_XCf.av() = paramFuncs.computeFuncDiff1(asInt(PFV::V));///@warn not well defined on ctrl-pt lattice
	_XCf.aw() = paramFuncs.computeFuncDiff1(asInt(PFV::W));
	_XCf.t () = _arc;
	_XCf.s () = paramFuncs.computeS();   
    }
    //implement evaluation of the numerical state variables derivatives at a given arc and a given evaluation order guarantee
    public  : void setXCfDotImpl ( auto& _XCfDot, const auto& _XCf, const TNumType& _arc, const PfEaG& _eAG ) const {
	_XCfDot.theta() = _XCf.w ();
	_XCfDot.v ()    = _XCf.av();///@warn not well defined on ctrl-pt lattice
	_XCfDot.w ()    = _XCf.aw();
	_XCfDot.av()    = 0;
	_XCfDot.aw()    = 0;
	_XCfDot.t ()    = 1.;
	_XCfDot.s ()    = fabs( _XCf.v() );
    }
    //implement evaluation of the numerical state variables derivatives at a given arc and a given evaluation order guarantee
    public  : void setXNmDotImpl ( auto& _XNmDot, auto& _stateCf, const auto& _stateNm, const TNumType& _arc, const PfEaG& _eAG ) {
	setXCfNmStep(_stateCf, _arc, _eAG);
	if( (arcNmDotCache_ == _arc) && (_eAG == PfEaG::NEAR_LAST) ) {  }
	else {
	    cosTheta_ = cos(_stateCf.theta());
	    sinTheta_ = sin(_stateCf.theta());
	    arcNmDotCache_ = _arc;
	}
        _XNmDot.x()     = _stateCf.v() * cosTheta_;
        _XNmDot.y()     = _stateCf.v() * sinTheta_;
        for(size_t i = 0; i<_XNmDot.persons().subSize();i++) {
            _XNmDot.persons().sub(i).x()     = 0;
            _XNmDot.persons().sub(i).y()     = 0;
            _XNmDot.persons().sub(i).theta() = 0;
            const double v = 1; //this->paramStruct->cfData.personVs[i];
            if(v>0.1) {
                const double theta = _stateNm.persons().sub(i).theta();
                _XNmDot.persons().sub(i).x() = v * cos(theta);
                _XNmDot.persons().sub(i).y() = v * sin(theta);
                
                const double x = _stateNm.persons().sub(i).x();
                const double y = _stateNm.persons().sub(i).y();
                //Using the all_dirs layer as sink
                auto &map = this->paramStruct->cfData.personHeatmap;
                auto &layers = this->paramStruct->cfData.personHeatmapLayers;
                grid_map::Index idx;
                grid_map::Position pos(x,y);
                if(map.isInside(pos)) {
                    map.getIndex(pos,idx);
                    const size_t linIdx = grid_map::getLinearIndexFromIndex(idx,map.getSize());
                    int direction;
                    if((theta>=-M_PI && theta<-3*M_PI/4) || (theta<=M_PI && theta>3*M_PI/4)) { direction = 3;}
                    else if(theta>=-3*M_PI/4 && theta<-M_PI/4) { direction = 2;}
                    else if(theta>=-M_PI/4 && theta<M_PI/4) { direction = 1;}
                    else { direction = 0;}
                    double thetaTar = 0;
                    double rotation = 0;
                    if(fabs((*layers[direction*2+1])(linIdx)) > 0 || fabs((*layers[direction*2])(linIdx)) > 0) {
                        thetaTar = atan2((*layers[direction*2+1])(linIdx),(*layers[direction*2])(linIdx));
                        rotation = thetaTar-theta;
                        if(rotation<-M_PI) {rotation+=2*M_PI;}
                        else if(rotation>M_PI) {rotation-=2*M_PI;}
                        _XNmDot.persons().sub(i).theta() = rotation*this->paramStruct->cfData.peoplePredictionP;
                    }
                } else {
                    _XNmDot.persons().sub(i).x()     = 0;
                    _XNmDot.persons().sub(i).y()     = 0;
                }
            }
        }
    }
    
    private : void setXCfNmStep ( auto& _XCf, const TNumType& _arc, const PfEaG& _eAG ) { 
	if( (arcCfNmStepCache_ == _arc) && (_eAG == PfEaG::NEAR_LAST) ) { return; } arcCfNmStepCache_ = _arc;
	auto& paramFuncs   = this->paramStruct->paramFuncs;
	const auto& state0 = this->paramStruct->state0;
	paramFuncs.setEvalArc (_arc, _eAG);
	
	const TNumType thetaIndef = paramFuncs.computeFuncInt1  ( asInt(PFV::W) );
	_XCf.theta() = thetaIndef + state0.stateCf().theta();
	_XCf.v    () = paramFuncs.computeFuncVal(asInt(PFV::V));
    }
    
    ///-----------------------------------Gradient information-----------------------------------///
    
    public   : void setGradXNm0Impl(auto& _gradXNm0, const auto& _XNm0) {
	for(int i = 0; i < _gradXNm0.data().size(); ++i) { _gradXNm0.data()(i) = 0; }
    }
    public   : void adjustGradXSizeImpl(auto& _gradXNm, auto& _gradXCf) {
        const bool personSizeChanged = _gradXNm.persons().subSize() != this->paramStruct->state0.stateNm().persons().subSize();
	auto& paramFuncs = this->paramStruct->paramFuncs;
	int ctrlPtOptNr = paramFuncs.funcCtrlPtSize(0)-1;
	if ( _gradXNm.var(0).sub(0).data().size() != ctrlPtOptNr || personSizeChanged) {
            _gradXNm.persons().subResize(this->paramStruct->state0.stateNm().persons().subSize());
	    for(size_t i = 0; i < _gradXNm.varSize(); ++i) {
		for(size_t j = 0; j < _gradXNm.var(i).subSize(); ++j) {
		    _gradXNm.var(i).sub(j).subResize(ctrlPtOptNr);
		}
	    }
	    for(size_t i = 0; i < _gradXCf.varSize(); ++i) {
		for(size_t j = 0; j < _gradXCf.var(i).subSize(); ++j) {
		    _gradXCf.var(i).sub(j).subResize(ctrlPtOptNr);
		}
	    }
	}
    }
    public  : void   setGradXCfImpl       ( auto& _gradXCf, const auto& _XCf, const TNumType& _arc, const PfEaG& _eAG ) {
	setGradXCfNmStep ( _gradXCf, _XCf, _arc, _eAG );
    }
    public  : void setGradXNmDotImpl ( auto& _gradXNmDot, auto& _XGradXCf, const auto& _XGradXNm, const TNumType& _arc, const PfEaG& _eAG ) {
	if( (arcGradNmStepCache_ == _arc) && (_eAG == PfEaG::NEAR_LAST) ) { return; } arcGradNmStepCache_ = _arc;
	
	auto& gradXCf   = _XGradXCf.stateGrad();
	const auto& XCf = _XGradXCf.state();
	setGradXCfNmStep(gradXCf, XCf, _arc, _eAG);
	
	//combining dfdx * GradX + dfdu * dudp
	static Eigen::Matrix<TNumType,2,1> dfduX;
	static Eigen::Matrix<TNumType,2,1> dfduY;
	
        for(int i=0;i<_gradXNmDot.data().size();i++) { _gradXNmDot.data()(i) = 0; }
        
	dfduX(0) = cosTheta_;
	dfduX(1) = sinTheta_;
	dfduY(0) = - XCf.v() * sinTheta_;
	dfduY(1) = + XCf.v() * cosTheta_;
	
	auto& dXdParamV = _gradXNmDot.x().optParamV().data();
	auto& dYdParamV = _gradXNmDot.y().optParamV().data();
	auto& dXdParamW = _gradXNmDot.x().optParamW().data();
	auto& dYdParamW = _gradXNmDot.y().optParamW().data();
	auto& dXdParamT = _gradXNmDot.x().optParamT().data();
	auto& dYdParamT = _gradXNmDot.y().optParamT().data();
	const auto& stateGradCfParamVIV     = gradXCf.v    ().optParamV();
	const auto& stateGradCfParamWITheta = gradXCf.theta().optParamW();
	const auto& stateGradCfParamTIV     = gradXCf.v    ().optParamT();
	const auto& stateGradCfParamTITheta = gradXCf.theta().optParamT();
	
	for(int i = 0; i < dXdParamV.size(); ++i) {
	    const auto& stateGradCfParamVIVI     = stateGradCfParamVIV.sub(i);
	    const auto& stateGradCfParamWIThetaI = stateGradCfParamWITheta.sub(i);
	    const auto& stateGradCfParamTIVI     = stateGradCfParamTIV.sub(i);
	    const auto& stateGradCfParamTIThetaI = stateGradCfParamTITheta.sub(i);
	    dXdParamV(i) = stateGradCfParamVIVI     * dfduX(0);
	    dYdParamV(i) = stateGradCfParamVIVI     * dfduX(1);
	    dXdParamW(i) = stateGradCfParamWIThetaI * dfduY(0);
	    dYdParamW(i) = stateGradCfParamWIThetaI * dfduY(1);
	    dXdParamT(i) = stateGradCfParamTIVI     * dfduX(0) + stateGradCfParamTIThetaI * dfduY(0);
	    dYdParamT(i) = stateGradCfParamTIVI     * dfduX(1) + stateGradCfParamTIThetaI * dfduY(1);
	}
    }
    
    
    //implement evaluation of the numerical state variables derivatives at a given arc and a given evaluation order guarantee
    private : void setGradXCfNmStep ( auto& _gradXCf, const auto& _XCf, const TNumType& _arc, const PfEaG& _eAG ) { 
	if( (arcGradCache_ == _arc) && (_eAG == PfEaG::NEAR_LAST) ) { return; } arcGradCache_ = _arc;
	auto& paramFuncs = this->paramStruct->paramFuncs;

	_gradXCf.data().setZero();
	auto& dVdParamV  = _gradXCf. v().optParamV();
	auto& dSdParamV  = _gradXCf. s().optParamV();
	for(size_t i = 0; i < dVdParamV.subSize(); ++i) {
	    auto& dVdParamVI  = dVdParamV .sub(i);
	    auto& dSdParamVI  = dSdParamV .sub(i);
	    
	    if(i+1 < dVdParamV.subSize()) {
		const TNumType& evalArcAbove = paramFuncs.ctrlPtVal(0,i+2,CtrlPtDim::ARC);
		const TNumType& evalArcBelow = paramFuncs.ctrlPtVal(0,i+1,CtrlPtDim::ARC);
		if(_arc > evalArcBelow) {
		    const TNumType arcIntEnd = fmin(_arc, evalArcAbove);
		    const TNumType deltaEvalArc = evalArcAbove-evalArcBelow;
		    dSdParamVI = + (arcIntEnd - evalArcBelow) * (evalArcBelow - 2. * _arc + arcIntEnd) / (2.*deltaEvalArc);
		    dVdParamVI = - (arcIntEnd - evalArcBelow) / deltaEvalArc;
		}
	    }
	    const TNumType& evalArcAbove = paramFuncs.ctrlPtVal(0,i+1,CtrlPtDim::ARC);
	    const TNumType& evalArcBelow = paramFuncs.ctrlPtVal(0,i  ,CtrlPtDim::ARC);
	    if(_arc > evalArcBelow) {
		const TNumType arcIntEnd = fmin(_arc, evalArcAbove);
		const TNumType deltaEvalArc = evalArcAbove-evalArcBelow;
		dSdParamVI += - (arcIntEnd - evalArcBelow) * (evalArcBelow - 2. * _arc + arcIntEnd) / (2.*deltaEvalArc);
		dVdParamVI += + (arcIntEnd - evalArcBelow) / deltaEvalArc;
	    }
	}
	
	auto& dThdParamW = _gradXCf.theta().optParamW();
	auto& dWdParamW  = _gradXCf.w    ().optParamW();
	for(size_t i = 0; i < dThdParamW.subSize(); ++i) {
	    auto& dThdParamWI = dThdParamW.sub(i);
	    auto& dWdParamWI  = dWdParamW .sub(i);
	    if( i+1 < dThdParamW.subSize() ) {
		const TNumType& evalArcAbove = paramFuncs.ctrlPtVal(1,i+2,CtrlPtDim::ARC);
		const TNumType& evalArcBelow = paramFuncs.ctrlPtVal(1,i+1,CtrlPtDim::ARC);
		if(_arc > evalArcBelow) {
		    const TNumType arcIntEnd = fmin(_arc, evalArcAbove);
		    const TNumType deltaEvalArc = evalArcAbove-evalArcBelow;
		    dThdParamWI = + (arcIntEnd - evalArcBelow) * (evalArcBelow - 2. * _arc + arcIntEnd) / (2.*deltaEvalArc);
		    dWdParamWI  = - (arcIntEnd - evalArcBelow) / deltaEvalArc;
		}
	    }
	    const TNumType& evalArcAbove = paramFuncs.ctrlPtVal(1,i+1,CtrlPtDim::ARC);
	    const TNumType& evalArcBelow = paramFuncs.ctrlPtVal(1,i  ,CtrlPtDim::ARC);
	    if(_arc > evalArcBelow) {
		const TNumType arcIntEnd = fmin(_arc, evalArcAbove);
		const TNumType deltaEvalArc = evalArcAbove-evalArcBelow;
		dThdParamWI += - (arcIntEnd - evalArcBelow) * (evalArcBelow - 2. * _arc + arcIntEnd) / (2.*deltaEvalArc);
		dWdParamWI  += + (arcIntEnd - evalArcBelow) / deltaEvalArc;
	    }
	}
	
	auto& dThdParamT = _gradXCf.theta().optParamT();
	auto& dVdParamT  = _gradXCf.v    ().optParamT();
	auto& dWdParamT  = _gradXCf.w    ().optParamT();
	auto& dSdParamT  = _gradXCf.s    ().optParamT();
	
	auto& dAVdParamV = _gradXCf.av().optParamV();
	auto& dAVdParamT = _gradXCf.av().optParamT();
	auto& dAWdParamW = _gradXCf.aw().optParamW();
	auto& dAWdParamT = _gradXCf.aw().optParamT();
	for(size_t i = 0; i < dVdParamT.subSize(); ++i) {
	    auto& dThdParamTI = dThdParamT.sub(i);
	    auto& dVdParamTI  = dVdParamT .sub(i);
	    auto& dWdParamTI  = dWdParamT .sub(i);
	    auto& dSdParamTI  = dSdParamT .sub(i);
	    
	    auto& dAVdParamVI = dAVdParamV.sub(i);
	    auto& dAWdParamWI = dAWdParamW.sub(i);
	    auto& dAVdParamTI = dAVdParamT.sub(i);
	    auto& dAWdParamTI = dAWdParamT.sub(i);
	    if ( i+1 < paramFuncs.funcCtrlPtSize(0) ) {
		const TNumType& evalArcAbove  = paramFuncs.ctrlPtVal(1,i+1,CtrlPtDim::ARC);
		const TNumType& evalArcBelow  = paramFuncs.ctrlPtVal(1,i  ,CtrlPtDim::ARC);
		if ( ( _arc <= evalArcAbove ) && ( _arc > evalArcBelow ) ) {
		    const TNumType& vP              = paramFuncs.ctrlPtVal(0,i+1,CtrlPtDim::VAL);
		    const TNumType& vM              = paramFuncs.ctrlPtVal(0,i+0,CtrlPtDim::VAL);
		    const TNumType& wP              = paramFuncs.ctrlPtVal(1,i+1,CtrlPtDim::VAL);
		    const TNumType& wM              = paramFuncs.ctrlPtVal(1,i+0,CtrlPtDim::VAL);
		    const TNumType arcIntEnd        = fmin(_arc, evalArcAbove);
		    const TNumType deltaEvalArc     = evalArcAbove-evalArcBelow;
		    const TNumType deltaEvalArcSqr  = deltaEvalArc*deltaEvalArc;
		    const TNumType deltaArcIntBound = arcIntEnd - evalArcBelow;
		    dThdParamTI = + deltaArcIntBound * (wP-wM) * (evalArcBelow - 2. * _arc + arcIntEnd) / (2.*deltaEvalArcSqr);
		    dVdParamTI  = - deltaArcIntBound * (vP-vM) / deltaEvalArcSqr;
		    dWdParamTI  = - deltaArcIntBound * (wP-wM) / deltaEvalArcSqr;
		    dAVdParamTI = - (vP-vM) / deltaEvalArcSqr;
		    dAWdParamTI = - (wP-wM) / deltaEvalArcSqr;
		    dAVdParamVI = + 1. / deltaEvalArc;
		    dAWdParamWI = + 1. / deltaEvalArc;
		    dSdParamTI  = + deltaArcIntBound * (vP-vM) * (evalArcBelow - 2. * _arc + arcIntEnd) / (2.*deltaEvalArcSqr);
		} else if ( ( _arc > evalArcBelow ) && ( i+1 < dVdParamT.subSize() ) ) {
		    const TNumType& evalArcAbove2  = paramFuncs.ctrlPtVal(1,i+2,CtrlPtDim::ARC);
		    const TNumType deltaEvalArc    = evalArcAbove2-evalArcAbove;
		    const TNumType deltaEvalArcSqr = deltaEvalArc*deltaEvalArc;
		    if ( _arc <=  evalArcAbove2 ) {
			dAVdParamVI = - 1. / deltaEvalArc;
			dAWdParamWI = - 1. / deltaEvalArc;
			const TNumType& vP = paramFuncs.ctrlPtVal(0,i+2,CtrlPtDim::VAL);
			const TNumType& vM = paramFuncs.ctrlPtVal(0,i+1,CtrlPtDim::VAL);
			const TNumType& wP = paramFuncs.ctrlPtVal(1,i+2,CtrlPtDim::VAL);
			const TNumType& wM = paramFuncs.ctrlPtVal(1,i+1,CtrlPtDim::VAL);
			dAVdParamTI = + (vP-vM) / deltaEvalArcSqr;
			dAWdParamTI = + (wP-wM) / deltaEvalArcSqr;
		    }
		    if ( _arc < evalArcAbove2 ) {
			const TNumType& vP  = paramFuncs.ctrlPtVal(0,i+2,CtrlPtDim::VAL);
			const TNumType& vM  = paramFuncs.ctrlPtVal(0,i+1,CtrlPtDim::VAL);
			const TNumType& vMM = paramFuncs.ctrlPtVal(0,i+0,CtrlPtDim::VAL);
			const TNumType& wP  = paramFuncs.ctrlPtVal(1,i+2,CtrlPtDim::VAL);
			const TNumType& wM  = paramFuncs.ctrlPtVal(1,i+1,CtrlPtDim::VAL);
			const TNumType& wMM = paramFuncs.ctrlPtVal(1,i+0,CtrlPtDim::VAL);
			const TNumType arcIntEnd = fmin(_arc, evalArcAbove2);
			
			const TNumType deltaArcIntBound = evalArcAbove2 - arcIntEnd;
			dThdParamTI = - ( +(wM-wMM)* evalArcAbove2*evalArcAbove2
					+(wP-wMM)*(evalArcAbove*evalArcAbove-2.*evalArcAbove*evalArcAbove2)
					+(wP-wM )*(arcIntEnd*arcIntEnd + 2.*_arc*(evalArcAbove2 - arcIntEnd)) ) / (2.*deltaEvalArcSqr);
			dVdParamTI  = - deltaArcIntBound * (vP-vM) / deltaEvalArcSqr;
			dWdParamTI  = - deltaArcIntBound * (wP-wM) / deltaEvalArcSqr;
			dSdParamTI  = - ( +(vM-vMM)* evalArcAbove2*evalArcAbove2
					+(vP-vMM)*(evalArcAbove*evalArcAbove-2.*evalArcAbove*evalArcAbove2)
					+(vP-vM )*(arcIntEnd*arcIntEnd + 2.*_arc*(evalArcAbove2 - arcIntEnd)) ) / (2.*deltaEvalArcSqr);
		    } else {
			const TNumType& vP = paramFuncs.ctrlPtVal(0,i+2,CtrlPtDim::VAL);
			const TNumType& vM = paramFuncs.ctrlPtVal(0,i+0,CtrlPtDim::VAL);
			const TNumType& wP = paramFuncs.ctrlPtVal(1,i+2,CtrlPtDim::VAL);
			const TNumType& wM = paramFuncs.ctrlPtVal(1,i+0,CtrlPtDim::VAL);
			dThdParamTI = - (wP-wM)/2.;
			dSdParamTI  = - (vP-vM)/2.;
		    }
		} else if ( _arc > evalArcBelow ) {
		    const TNumType& vP = paramFuncs.ctrlPtVal(0,i+1,CtrlPtDim::VAL);
		    const TNumType& vM = paramFuncs.ctrlPtVal(0,i+0,CtrlPtDim::VAL);
		    const TNumType& wP = paramFuncs.ctrlPtVal(1,i+1,CtrlPtDim::VAL);
		    const TNumType& wM = paramFuncs.ctrlPtVal(1,i+0,CtrlPtDim::VAL);
		    dThdParamTI = - (wP-wM)/2.;
		    dSdParamTI  = - (vP-vM)/2.;
		}
	    }
	}
    }
    
    //internal helper variables
    protected: TNumType cosTheta_;
    protected: TNumType sinTheta_;
    protected: TNumType arcCfNmStepCache_;
    protected: TNumType arcNmDotCache_;
    protected: TNumType arcGradCache_;
    protected: TNumType arcGradNmStepCache_;
};
//---------------------------------------------------------------------Optimization parameters


template<class TNumType,class TParamStructType>
struct OptVarMapVW {
    static void setOptVar( TParamStructType& _paramStruct, const Eigen::Matrix<TNumType, -1, 1>& _optVarExt ) {
	auto& paramFuncs = _paramStruct.paramFuncs;
	size_t idxOptVec = 0;
	for ( size_t i = 0; i < paramFuncs.funcsSize(); ++i ) {
	    for ( size_t j = 1; j < paramFuncs.funcCtrlPtSize(i); ++j ) {
		paramFuncs.ctrlPtVal ( i, j, CtrlPtDim::VAL ) = _optVarExt(idxOptVec++);
	    }
	}
	for ( size_t j = 1; j < paramFuncs.funcsArcSize(0); ++j ) {
	    paramFuncs.funcsArc ( 0, j ) = _optVarExt(idxOptVec++);
	}
    }
    static void getOptVar(Eigen::Matrix<TNumType, -1, 1>& _optVarExt, const TParamStructType& _paramStruct) {
	auto& paramFuncs = _paramStruct.paramFuncs;
	int newSize = paramFuncs.funcsSize() * (paramFuncs.funcCtrlPtSize(0)-1) + ( paramFuncs.funcsArcSize(0) - 1 );
	if ( newSize != _optVarExt.size() ) { _optVarExt.resize( newSize ); }
	size_t idxOptVec = 0;
	for ( size_t i = 0; i < paramFuncs.funcsSize(); ++i ) {
	    for ( size_t j = 1; j < paramFuncs.funcCtrlPtSize(i); ++j ) {
		_optVarExt(idxOptVec++) = paramFuncs.ctrlPtVal ( i, j, CtrlPtDim::VAL );
	    }
	}
	for ( size_t j = 1; j < paramFuncs.funcsArcSize(0); ++j ) {
	    _optVarExt(idxOptVec++)  = paramFuncs.funcsArc ( 0, j );
	}
    }
};

template<class TNumType, class TMapDataType, template<class> class TDiscretizationType>
class StateSimVW              : public StateSimVWBase<TNumType, TMapDataType, StateVW             <TNumType>, TDiscretizationType > {};

template<class TNumType, class TMapDataType, template<class> class TDiscretizationType, class... TCostFuncType>
class StateWithLSimVW         : public StateSimVWBase<TNumType, TMapDataType, StateWithLVW        <TNumType>, TDiscretizationType, TCostFuncType... > {};

template<class TNumType, class TMapDataType, template<class> class TDiscretizationType>
class StateWithGradSimVW      : public StateSimVWBase<TNumType, TMapDataType, StateWithGradVW     <TNumType>, TDiscretizationType > {};

template<class TNumType, class TMapDataType, template<class> class TDiscretizationType, class... TCostFuncType>
class StateWithLWithGradSimVW : public StateSimVWBase<TNumType, TMapDataType, StateWithLWithGradVW<TNumType>, TDiscretizationType, TCostFuncType... > {};

}

}

#endif // AGV_DIFF_DRIVE_V_W_HPP
