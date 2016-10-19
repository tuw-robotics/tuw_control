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

#include <tuw_control/param_func/param_func.h>

#include <float.h>
#include <math.h>

using namespace tuw;
using namespace std;

void ParamFuncs::init ( vector<ParamFuncsStructure>& _paramFuncsStructure ) {
    const size_t funcsSize = _paramFuncsStructure.size();
    if ( funcsSize < 1 ) { throw "Error in ParamFuncs::checkParamFuncsStructure: initialization with 0 number of functions!"; }
    funcCtrlPt_   .resize ( funcsSize ); for( auto& funcCtrlPtrI : funcCtrlPt_ ) { funcCtrlPtrI.clear(); }
    func2Arc_     .resize ( funcsSize );
    funcEvalReq_  .resize ( funcsSize );
    funcCtrlPtArc_.clear();
    vector<size_t> funcSameArcCtrlPtSize;
    for ( size_t i = 0; i < funcsSize; ++i ) {
	const size_t& funcCtrlPtsSize = _paramFuncsStructure[i].ctrlPtsSize;
	if( funcCtrlPtsSize < 2 ) { throw "Error in ParamFuncs::checkParamFuncsStructure: function has less than 2 control points!"; }
	
	func2Arc_[i] = _paramFuncsStructure[i].ctrlPtsArcRefIdx;
        if ( func2Arc_[i] >= funcCtrlPtArc_.size() ) {
            if( func2Arc_[i] == funcCtrlPtArc_.size() ) { funcCtrlPtArc_.emplace_back ( vector<double> ( funcCtrlPtsSize-2, 0 ) ); funcSameArcCtrlPtSize.emplace_back( funcCtrlPtsSize ); }
            else                                        { throw "Error in ParamFuncs::init: a ctrlPtsArcRefIdx is not consistent (new Idx not unit incremental)"; }
        }
        if( funcCtrlPtsSize != funcSameArcCtrlPtSize[func2Arc_[i]] ) { 
	    throw "Error in ParamFuncs::init: inconsistent number of control points for functions with same arc parametrization reference";
	}
        funcCtrlPt_[i].reserve ( funcCtrlPtsSize );
        funcCtrlPt_[i].emplace_back ( FuncCtrlPt ( 0, funcsArcBegin_ ) );
        for ( size_t j = 1; j < funcCtrlPtsSize - 1; ++j ) {
            funcCtrlPt_[i].emplace_back ( FuncCtrlPt ( 0, funcCtrlPtArc_[func2Arc_[i]][j-1] ) );
        }
        funcCtrlPt_[i].emplace_back ( FuncCtrlPt ( 0, funcsArcEnd_ ) );
	
	funcEvalReq_[i] = _paramFuncsStructure[i].evalReq;
    }
    initImpl();
}

void ParamFuncs::shiftStartCtrlPt ( const double& _dt ) {
    std::vector<std::vector<size_t> > ctrlPtModif(funcsSize(), std::vector<size_t>(0,0));
    ctrlPtModif.reserve(10);///@todo not nice
    const double evalArc = funcsArcBegin_ + _dt;
    setEvalArc ( evalArc );
    for ( size_t i = 0; i < funcsSize(); ++i ) { 
	for( size_t j = 0; j < funcCtrlPtSize(i); ++j ) {
	    if( ctrlPtVal(i,j,CtrlPtDim::ARC) < evalArc ) { ctrlPtModif[i].emplace_back ( j ); }
	}
    }
    for ( size_t i = 0; i < ctrlPtModif.size(); ++i ) { 
	const double newVal = computeFuncVal(i);
	for( size_t j = 0; j < ctrlPtModif[i].size(); ++j ) {
	    ctrlPtVal(i,j,CtrlPtDim::VAL) = newVal;
	}
    }
    for ( size_t k = 0; k < funcsArcSize(); ++k ) {
	for ( size_t j = 0; j < funcCtrlPtArc_[k].size(); ++j ) {
	    funcCtrlPtArc_[k][j] -= _dt;
	}
    }
    funcsArcEnd_ -= _dt;
    precompute();
    setEvalArc(funcsArcBegin_);
}

size_t ParamFuncs::funcsSize() const {
    return funcCtrlPt_.size();
}
size_t ParamFuncs::funcCtrlPtSize ( const size_t& _i ) const {
    return funcCtrlPt_[_i].size();
}
size_t ParamFuncs::funcsArcSize() const {
    return funcCtrlPtArc_.size();
}

double& ParamFuncs::funcsArcBegin() {
    return funcsArcBegin_;
}
const double& ParamFuncs::funcsArcBegin() const {
    return funcsArcBegin_;
}
double& ParamFuncs::funcsArcEnd() {
    return funcsArcEnd_;
}
const double& ParamFuncs::funcsArcEnd() const {
    return funcsArcEnd_;
}
const double& ParamFuncs::funcsArcEval() const {
    return funcsArcEval_;
}

size_t ParamFuncs::funcsArcSize ( const size_t& _i ) const {
    return funcCtrlPtArc_[_i].size() + 2;
}

double& ParamFuncs::funcsArc ( const size_t& _i, const size_t& _j ) {
    if ( _j == 0                         ) { return funcsArcBegin_; }
    if ( _j  > funcCtrlPtArc_[_i].size() ) { return funcsArcEnd_  ; }
    return funcCtrlPtArc_[_i][_j-1];
}
const double& ParamFuncs::funcsArc ( const size_t& _i, const size_t& _j ) const {
    if ( _j == 0                         ) { return funcsArcBegin_; }
    if ( _j  > funcCtrlPtArc_[_i].size() ) { return funcsArcEnd_  ; }
    return funcCtrlPtArc_[_i][_j-1];
}

double& ParamFuncs::ctrlPtVal ( const size_t& _funcIdx, const size_t& _funcCtrlPtIdx, const CtrlPtDim& _ctrlPtDim ) {
    switch ( _ctrlPtDim ) {
	case CtrlPtDim::VAL : return funcCtrlPt_[_funcIdx][_funcCtrlPtIdx].val;      break;
	case CtrlPtDim::ARC : return funcCtrlPt_[_funcIdx][_funcCtrlPtIdx].arc;      break;
	default: throw "CtrlPtDim unrecognised in function ParamFuncs::ctrlPtVal !"; break;
    }
}
const double& ParamFuncs::ctrlPtVal ( const size_t& _funcIdx, const size_t& _funcCtrlPtIdx, const CtrlPtDim& _ctrlPtDim ) const {
    switch ( _ctrlPtDim ) {
	case CtrlPtDim::VAL : return funcCtrlPt_[_funcIdx][_funcCtrlPtIdx].val;      break;
	case CtrlPtDim::ARC : return funcCtrlPt_[_funcIdx][_funcCtrlPtIdx].arc;      break;
	default: throw "CtrlPtDim unrecognised in function ParamFuncs::ctrlPtVal !"; break;
    }
}

ParamFuncs::FuncCtrlPt& ParamFuncs::ctrlPt ( const size_t& _funcIdx, const size_t& _funcCtrlPtIdx ) {
    return funcCtrlPt_[_funcIdx][_funcCtrlPtIdx];
}
const ParamFuncs::FuncCtrlPt& ParamFuncs::ctrlPt ( const size_t& _funcIdx, const size_t& _funcCtrlPtIdx ) const {
    return funcCtrlPt_[_funcIdx][_funcCtrlPtIdx];
}

// void ParamFuncs::computeS2TLattice ( const double& _arc0, const double& _ds, vector< double >& _tLattice ) {
//     std::vector<double> sLattice;
//     
//     setEvalArc(      funcsArcEnd_        , EvalArcGuarantee::AT_END ); double sEnd = computeS();
//     setEvalArc( fmin(funcsArcEnd_, _arc0) );
//     
//     size_t idxBeforeStart = static_cast<int>( computeS() / _ds );
//     
//     sLattice.resize( static_cast<int>( sEnd/ _ds ) - idxBeforeStart );
//     for( size_t i = 0; i < sLattice.size(); ++i ) { sLattice[i] = _ds * ( idxBeforeStart + i + 1 ); }
//     computeS2TLattice(sLattice, _tLattice);
//     
//     setEvalArc(funcsArcBegin_);
// }
