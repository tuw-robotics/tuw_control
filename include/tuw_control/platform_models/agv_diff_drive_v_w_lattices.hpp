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

#ifndef AGV_DIFF_DRIVE_V_W_LATTICES_HPP
#define AGV_DIFF_DRIVE_V_W_LATTICES_HPP

#include <tuw_control/platform_models/agv_diff_drive_v_w.hpp>
#include <tuw_control/trajectory_simulator_new_2/trajectory_simulator.hpp>
#include <tuw_control/utils.h>

namespace tuw {

template<typename TNumType, class TSimType, class... TCostFuncsType>
class LatticeTypeStateSimBeginEnd : public LatticeTypeBaseCRTP<LatticeTypeStateSimBeginEnd<TNumType, TSimType, TCostFuncsType...>, TNumType, TSimType, TCostFuncsType...> {
    public   : using LatticeTypeBaseCRTP<LatticeTypeStateSimBeginEnd, TNumType, TSimType, TCostFuncsType...>::LatticeTypeBaseCRTP;
    public   : static void computeLatticeArcsImpl (TSimType& _sim, std::vector<TNumType>& _latticeArcs ) {
	_latticeArcs.resize( 1 );
	_latticeArcs[0] = _sim.paramStruct->paramFuncs.funcsArcEnd();
    }
    public   : void computeDArcIdPImpl(const TSimType& _sim, const TNumType _arc, const size_t& _lattIdx, auto& _dstStateGrad ){
	auto& dstStateGradCf = _dstStateGrad.stateCf();
	auto& dstStateGradNm = _dstStateGrad.stateNm();
	size_t optParamTBackIdx = dstStateGradCf.sub(0).data().size()-1;
	dstStateGradCf.mat().col(optParamTBackIdx) += _sim.stateCfDotCache().data();
	dstStateGradNm.mat().col(optParamTBackIdx) += _sim.stateNmDotCache().data();
    }
    public   : virtual void precompute(TSimType& _sim) override {}
};

template<typename TNumType, class TSimType, class... TCostFuncsType>
class LatticeTypeStateSimEqDt : public LatticeTypeBaseCRTP<LatticeTypeStateSimEqDt<TNumType, TSimType, TCostFuncsType...>, TNumType, TSimType, TCostFuncsType...> {
    public   : using LatticeTypeBaseCRTP<LatticeTypeStateSimEqDt, TNumType, TSimType, TCostFuncsType...>::LatticeTypeBaseCRTP;
    public   : void computeLatticeArcsImpl (TSimType& _sim, std::vector<TNumType>& _latticeArcs ) const {
	const TNumType arcParamMax    =  _sim.paramStruct->paramFuncs.funcsArcEnd();
	double dt = dt_;
	if (nrPts_ > 1) { dt = arcParamMax / (TNumType)(nrPts_); }
	const size_t latticeSize = std::max( 0, (int)( ceil( arcParamMax / dt )  ) ); 
	if(latticeSize != _latticeArcs.size()) { _latticeArcs.resize( latticeSize ); }
	for ( size_t i = 0; i < latticeSize; ++i ) { _latticeArcs[i] = (i+1) * dt; }
	_latticeArcs.back() = arcParamMax;
    }
    public   : void computeDArcIdPImpl(const TSimType& _sim, const TNumType _arc, const size_t& _lattIdx, auto& _dstStateGrad ) {
// 	if (nrPts_ > 1) { 
	    auto& dstStateGradCf = _dstStateGrad.stateCf();
	    auto& dstStateGradNm = _dstStateGrad.stateNm();
	    
	    const TNumType dtIdpEnd = (TNumType)(_lattIdx+1) / (TNumType)(this->lattice.size()/*-1*/);
	    size_t optParamTBackIdx = dstStateGradCf.sub(0).data().size()-1;
	    
	    dstStateGradCf.mat().col(optParamTBackIdx) += _sim.stateCfDotCache().data() * dtIdpEnd;
	    dstStateGradNm.mat().col(optParamTBackIdx) += _sim.stateNmDotCache().data() * dtIdpEnd;
// 	}
    }
    public   : virtual void precompute(TSimType& _sim) override {}
    public   : const TNumType& dt () { return dt_; }
    public   : void setDt   (const TNumType& _dt) { dt_ = _dt; nrPts_ = -1; }
    public   : void setNrPts(const size_t& _nrPts) { nrPts_ = _nrPts; }
    private  : TNumType dt_;
    private  : int nrPts_;
};

template<typename TNumType, class TSimType, class... TCostFuncsType>
class LatticeTypeStateSimEqDs : public LatticeTypeBaseCRTP<LatticeTypeStateSimEqDs<TNumType, TSimType, TCostFuncsType...>, TNumType, TSimType, TCostFuncsType...> {
    public   : using LatticeTypeBaseCRTP<LatticeTypeStateSimEqDs, TNumType, TSimType, TCostFuncsType...>::LatticeTypeBaseCRTP;
    public   : void computeLatticeArcsImpl (TSimType& _sim, std::vector<TNumType>& _latticeArcs ) {
	if ( (ds_ > 0) || (nrPts_ > 1) ) { 
	    auto& paramFuncs = _sim.paramStruct->paramFuncs;
	    if(nrPts_ > 1) {
		paramFuncs.setEvalArc(  paramFuncs.funcsArcEnd()   );
		ds_ =  paramFuncs.computeS() / (TNumType)(nrPts_);
		paramFuncs.setEvalArc(  paramFuncs.funcsArcBegin() );
	    }
	    paramFuncs.computeS2TLattice( ds_, ds_, _latticeArcs );
	} 
	else        { _latticeArcs.clear(); }
    }
    public   : void computeDArcIdPImpl(const TSimType& _sim, const TNumType _arc, const size_t& _lattIdx, auto& _dstStateGrad ) {
	if (nrPts_ > 1) { 
	    static Eigen::Matrix<TNumType,-1,1> dsdp;
	    static Eigen::Matrix<TNumType,-1,1> stateDotCache;
	    static Eigen::Matrix<TNumType,-1,1> dtEndDp;
	    if(_lattIdx == 0) { 
		TNumType tEnd = _sim.paramStruct->paramFuncs.funcsArcEnd();
		auto& simNonConst = const_cast<TSimType&>(_sim);
		simNonConst.setXCf     (  tEnd, PfEaG::NONE );
		simNonConst.setGradXCf (  tEnd, PfEaG::NONE );
		simNonConst.setXCfDot  (  tEnd, PfEaG::NONE );
		
		dsdp = _sim.state().stateGradCf().s().data();
		dsdp(dsdp.rows()-1) += _sim.stateCfDotCache().s();
		dtEndDp = dsdp * 0;
		
		simNonConst.setXCf     (  0, PfEaG::AT_BEGIN );
		simNonConst.setGradXCf (  0, PfEaG::AT_BEGIN );
		stateDotCache.resize(_sim.stateNmDotCache().data().size()+ _sim.stateCfDotCache().data().size());
	    }
	    stateDotCache.block(                                   0, 0, _sim.stateNmDotCache().data().size(), 1) =  _sim.stateNmDotCache().data();
	    stateDotCache.block(_sim.stateNmDotCache().data().size(), 0, _sim.stateCfDotCache().data().size(), 1) =  _sim.stateCfDotCache().data();
	    auto& v = _sim.state().stateCf().v();
	    if(fabs(v) > 1e-3) {///@todo would be nicer to do linear interp between singularity
		TNumType iByNS = _sim.state().stateCf().s() / (ds_*nrPts_);
		_dstStateGrad.mat() += ( stateDotCache ) * ( ( iByNS * dsdp - _dstStateGrad.stateCf().s().data() )  / v).transpose();
	    }
	}
    }
    public   : virtual void precompute(TSimType& _sim) override {}
    public   : const TNumType& dt () { return ds_; }
    public   : void setDs   (const TNumType& _ds) { ds_ = _ds; nrPts_ = -1; }
    public   : void setNrPts(const size_t& _nrPts) { nrPts_ = _nrPts; }
    public   : TNumType ds_;
    private  : int nrPts_;
};


template<typename TNumType, class TSimType, class... TCostFuncsType>
class LatticeTypeStateSimCtrlPtKnots : public LatticeTypeBaseCRTP<LatticeTypeStateSimCtrlPtKnots<TNumType, TSimType, TCostFuncsType...>, TNumType, TSimType, TCostFuncsType...> {
    public   : using LatticeTypeBaseCRTP<LatticeTypeStateSimCtrlPtKnots, TNumType, TSimType, TCostFuncsType...>::LatticeTypeBaseCRTP;
    public   : void computeLatticeArcsImpl (TSimType& _sim, std::vector<TNumType>& _latticeArcs ) const {
	if(inUse){
	    _latticeArcs.resize( _sim.paramStruct->paramFuncs.funcsArcSize(0)-1);
	    auto& paramFuncs = _sim.paramStruct->paramFuncs;
	    for ( size_t i = 0; i < _latticeArcs.size(); ++i ) { _latticeArcs[i] =  paramFuncs.funcsArc(0, i+1); }
	} else {
	    _latticeArcs.clear();
	}
    }
    public   : void computeDArcIdPImpl(const TSimType& _sim, const TNumType _arc, const size_t& _lattIdx, auto& _dstStateGrad ) {
	if(inUse) {
	    auto& dstStateGradCf = _dstStateGrad.stateCf();
	    auto& dstStateGradNm = _dstStateGrad.stateNm();
	    size_t optParamTIdx = dstStateGradCf.sub(0).optParamV().data().size() + dstStateGradCf.sub(0).optParamW().data().size();
	    dstStateGradCf.mat().col(optParamTIdx + _lattIdx) += _sim.stateCfDotCache().data();
	    dstStateGradNm.mat().col(optParamTIdx + _lattIdx) += _sim.stateNmDotCache().data();
	}
    }
    public   : virtual void precompute(TSimType& _sim) override {}
    public   : bool inUse;
};

}

#endif // AGV_DIFF_DRIVE_V_W_LATTICES_HPP
