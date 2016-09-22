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
\code{.cpp}
#include <tuw_control/utils.h>
#include <tuw_control/state/state_array.hpp>
#include <tuw_control/state/state_nested_set.h>
#include <tuw_control/state/state_nested_vector.h>
#include <tuw_control/state/state_nested_array.h>

#include <iostream>

using namespace tuw;

struct I2WS_Wheel_State {
    double steering;
    double revolute;
    void fromState ( const StateSPtr state ) {
        steering = state->value ( 0 );
        revolute = state->value ( 1 );
    }
    friend std::ostream &operator << ( std::ostream &os, const I2WS_Wheel_State &o ) {
        os << "[" << o.steering <<  ", " << o.revolute << "]";
        return os;
    };
};
struct I2WS_Wheels_State {
    I2WS_Wheel_State left;
    I2WS_Wheel_State right;
    void fromState ( const StateSPtr state ) {
#ifdef true
        left. fromState ( state->state ( 0 ) );
        right.fromState ( state->state ( 1 ) );
#else
        left. steering = state->value ( 0 );
        left. revolute = state->value ( 1 );
        right.steering = state->value ( 2 );
        right.revolute = state->value ( 3 );
#endif
    }
    friend std::ostream &operator << ( std::ostream &os, const I2WS_Wheels_State &o ) {
        os << "[" << o.left <<  ", " << o.right << "]";
        return os;
    };
};

///@brief Enum defining the semantics of the Iws wheel state.
enum class StateWheel { Steer, Revol, ENUM_SIZE };
///@brief State definition for the wheels input. It is thus a vector of @ref StateWheel sub-states.
using StateWhInpType   = StateNestedVector<StateArrayScoped<StateWheel>>; 
using StateWhInpPtrType = std::shared_ptr<StateWhInpType>;


int main ( int argc, char **argv ) {
    std::cout << "iws utils test" << std::endl;
    StateWhInpPtrType tuw_i2ws_wheel_state = std::make_shared<StateWhInpType>();
    tuw_i2ws_wheel_state->resize ( 2 );
    tuw_i2ws_wheel_state->value(0) = 10;
    tuw_i2ws_wheel_state->value(1) = 11;
    tuw_i2ws_wheel_state->value(2) = 12;
    tuw_i2ws_wheel_state->value(3) = 13;
    /// printing the state
    std::cout << "tuw      : "  << *tuw_i2ws_wheel_state << std::endl;
    
    
    
    /// convert to stl vector
    std::vector<double> values;
    tuw_i2ws_wheel_state->toSTLVec ( values );
    
    /// to a classical struct
    I2WS_Wheels_State classical_state;
    classical_state.fromState(tuw_i2ws_wheel_state);
    
    /// printing the classical struct
    std::cout << "classical: " << classical_state << std::endl;
    /// printing the classical struct
    std::cout << "tuw      : [" << *tuw_i2ws_wheel_state->state(0) << ", " << *tuw_i2ws_wheel_state->state(1) << "]" << std::endl;
    
    /// accessing varibles
    std::cout << "tuw      : " << tuw_i2ws_wheel_state->stateScoped(1)->value<StateWheel::Steer>() << std::endl;   
}
\endcode
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
    public   : void fromSTLVec  (const std::vector<double>& _vec) { if( _vec.size() != valueSize  () ) { throw "cannot copy from container of different size"; }; for(size_t i = 0; i < _vec.size(); ++i) { value(i) = _vec[i]; } }
    ///@brief Copies all values from an Eigen vector. The @ref valueSize of the State object has to be equal with the Eigen vector size.
    public   : void fromEIGENVec(const Eigen::VectorXd&     _vec) { if( (size_t)_vec.rows() != valueSize  () ) { throw "cannot copy from container of different size"; }; for(size_t i = 0; i < (size_t)_vec.rows(); ++i) { value(i) = _vec(i); } }
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
    /** @brief prints the state value to an output stream.
     *  @param os 
     *  @param o 
     *  @return stream
     */
    friend std::ostream &operator << ( std::ostream &os, const State &o ) {
        for(size_t i = 0; i < o.valueSize(); i++){
          os << (i==0?"[ ":", ") << o.value(i); 
        }
        return os << "]";
    };
    
    protected: void callRootUpdateSize () { if(parent_){ parent_->callRootUpdateSize(); } else { updateSize(); } }///< Calls (if present) the parent @ref updateSize procedure. Otherwise performs root @ref updateSize
    protected: State* parent_;///< Pointer to the parent @ref State structure
};



}

#endif // STATE_H