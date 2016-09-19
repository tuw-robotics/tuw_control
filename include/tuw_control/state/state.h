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

#ifndef STATE_H
#define STATE_H


#include <float.h>
#include <memory>

#include <eigen3/Eigen/Eigen>

namespace tuw {

/*!@class State 
 * @brief Generic tree-like recursive structure that allows sub-structure access as well as ordered (array-like) value access.
 * @todo cout the the state variables (horizontally)
 */
class State;
using StateSPtr      = std::shared_ptr<State>;
using StateConstSPtr = std::shared_ptr<State const>;
using StateUPtr      = std::unique_ptr<State>;
using StateConstUPtr = std::unique_ptr<State const>;
using StateVectorSPtr     = std::shared_ptr<std::vector<State   > >;
using StateSPtrVectorSPtr = std::shared_ptr<std::vector<StateSPtr> >;
class State {
    
    //special class member functions
    public   : State           (State* _parent) : parent_(_parent) { callRootUpdateSize(); }
    public   : State           ()               : parent_(nullptr) { callRootUpdateSize(); }
    public   : virtual ~State  ()             = default;
    public   : State           (const State&) = default;
    public   : State& operator=(const State&) = default;
    public   : State           (State&&)      = default;
    public   : State& operator=(State&&)      = default;
    
    //pure virtual functions
    ///@brief Clone-to-base-class-ptr function.
    public   : virtual StateSPtr     cloneState () const = 0;
    ///@brief Access state variable based on index @ref _i.
    public   : virtual double&       value      ( const std::size_t& _i ) = 0;
    ///@brief Const access state variable based on index @ref _i.
    public   : virtual const double& value      ( const std::size_t& _i ) const = 0;
    ///@brief Size of the state variables.
    public   : virtual size_t        valueSize  () const = 0;
    
    ///@brief Performs internal manipulation when any of the underlying arrays are being resized.
    public   : virtual void          updateSize () {}
    ///@brief Resizes the array.
    public   : virtual void          resize     ( const size_t& _i ) { throw "function not supported for the instantiated class"; }
    ///@brief Access sub-state based on index @ref _i.
    public   : virtual StateSPtr&    state      ( const std::size_t& _i ) { static StateSPtr stateNullPtr_ = nullptr; return stateNullPtr_; }
    ///@brief Size of the sub-states.
    public   : virtual size_t        stateSize  () const { return 0; }
    ///@brief Converts all the array values to an STL vector.
    public   : void toSTLVec    (std::vector<double>& _vec) { _vec.resize( valueSize() ); for(size_t i = 0; i < _vec.size(); ++i) { _vec[i] = value(i); } }
    ///@brief Converts all the array values to an Eigen vector.
    public   : void toEIGENVec  (Eigen::VectorXd&     _vec) { _vec.resize( valueSize() ); for(size_t i = 0; i < (size_t)_vec.rows(); ++i) { _vec(i) = value(i); } }
    ///@brief Copies all values from an STL vector. The @ref valueSize of the State object has to be equal with the STL vector size.
    public   : void fromSTLVec  (std::vector<double>& _vec) { if( _vec.size() != valueSize  () ) { throw "cannot copy from container of different size"; }; for(size_t i = 0; i < _vec.size(); ++i) { value(i) = _vec[i]; } }
    ///@brief Copies all values from an Eigen vector. The @ref valueSize of the State object has to be equal with the Eigen vector size.
    public   : void fromEIGENVec(Eigen::VectorXd&     _vec) { if( (size_t)_vec.rows() != valueSize  () ) { throw "cannot copy from container of different size"; }; for(size_t i = 0; i < (size_t)_vec.rows(); ++i) { value(i) = _vec(i); } }
    /** @brief Performs addition. Left and right operand are required to have the same @ref valueSize.
     *  @param _lhs Left operand
     *  @param _rhs Right operand
     *  @param _ans Computation result
     *  @return Computation result
     */
    static std::vector<double>& plus(State& _lhs, State& _rhs, std::vector<double>& _ans) {
	if ( _lhs.valueSize() != _rhs.valueSize() ){ throw "cannot add two containers of different sizes"; }
	_ans.resize(_lhs.valueSize());
	for(size_t i = 0; i < _lhs.valueSize(); ++i) { _ans[i] = _lhs.value(i) + _rhs.value(i); }
	return _ans;
    }
    /** @brief Performs addition. Left and right operand are required to have the same @ref valueSize.
     *  @param _lhs Left operand
     *  @param _rhs Right operand
     *  @param _ans Computation result
     *  @return Computation result
     */
    static Eigen::VectorXd& plus(State& _lhs, State& _rhs, Eigen::VectorXd& _ans) { 
	if ( _lhs.valueSize() != _rhs.valueSize() ){ throw "cannot add two containers of different sizes"; }
	_ans.resize(_lhs.valueSize());
	for(size_t i = 0; i < _lhs.valueSize(); ++i) { _ans[i] = _lhs.value(i) + _rhs.value(i); }
	return _ans;
    }
    /** @brief Performs addition. Answer variable, left operand and right operand are required to have the same @ref valueSize.
     *  @param _lhs Left operand
     *  @param _rhs Right operand
     *  @param _ans Computation result
     *  @return Computation result
     */
    static State& plus(State& _lhs, State& _rhs, State& _ans) {
	if (!(( _ans.valueSize() == _rhs.valueSize() ) && ( _lhs.valueSize() == _rhs.valueSize() ))){ throw "cannot add two containers of different sizes"; }
	for(size_t i = 0; i < _ans.valueSize(); ++i) { _ans.value(i) = _lhs.value(i) + _rhs.value(i); }
	return _ans;
    }
    /** @brief Performs substraction. Left and right operand are required to have the same @ref valueSize.
     *  @param _lhs Left operand
     *  @param _rhs Right operand
     *  @param _ans Computation result
     *  @return Computation result
     */
    static std::vector<double>& minus(State& _lhs, State& _rhs, std::vector<double>& _ans) {
	if ( _lhs.valueSize() != _rhs.valueSize() ){ throw "cannot add two containers of different sizes"; }
	_ans.resize(_lhs.valueSize());
	for(size_t i = 0; i < _lhs.valueSize(); ++i) { _ans[i] = _lhs.value(i) - _rhs.value(i); }
	return _ans;
    }
    /** @brief Performs substraction. Left and right operand are required to have the same @ref valueSize.
     *  @param _lhs Left operand
     *  @param _rhs Right operand
     *  @param _ans Computation result
     *  @return Computation result
     */
    static Eigen::VectorXd& minus(State& _lhs, State& _rhs, Eigen::VectorXd& _ans) { 
	if ( _lhs.valueSize() != _rhs.valueSize() ){ throw "cannot add two containers of different sizes"; }
	_ans.resize(_lhs.valueSize());
	for(size_t i = 0; i < _lhs.valueSize(); ++i) { _ans[i] = _lhs.value(i) - _rhs.value(i); }
	return _ans;
    }
    /** @brief Performs substraction. Answer variable, left operand and right operand are required to have the same @ref valueSize.
     *  @param _lhs Left operand
     *  @param _rhs Right operand
     *  @param _ans Computation result
     *  @return Computation result
     */
    static State& minus(State& _lhs, State& _rhs, State& _ans) {
	if (!(( _ans.valueSize() == _rhs.valueSize() ) && ( _lhs.valueSize() == _rhs.valueSize() ))){ throw "cannot add two containers of different sizes"; }
	for(size_t i = 0; i < _ans.valueSize(); ++i) { _ans.value(i) = _lhs.value(i) - _rhs.value(i); }
	return _ans;
    }
    
    
    protected: void callRootUpdateSize () { if(parent_){ parent_->callRootUpdateSize(); } else { updateSize(); } }///< Calls (if present) the parent @ref updateSize procedure. Otherwise performs root @ref updateSize
    protected: State* parent_;///< Pointer to the parent @ref State structure
};



}

#endif // STATE_H