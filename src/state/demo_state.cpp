/***************************************************************************
 *   Software License Agreement (BSD License)                              *
 *   Copyright (C) 2016 by Horatiu George ran <todorangrg@gmail.com>   *
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
enum class StateWheel {
    Steer, Revol, ENUM_SIZE
};
///@brief State definition for the wheels input. It is thus a vector of @ref StateWheel sub-states.
class OneWheelType : public StateArrayScoped<StateWheel> {
public:
    using StateArrayScoped::StateArrayScoped;
    const double &steer() const {
        return values_[0];
    }
    const double &revol() const {
        return values_[1];
    }
    double &steer() {
        return values_[0];
    }
    double &revol() {
        return values_[1];
    }
};
class StateWhInpType : public StateNestedVector<OneWheelType> {
public:
    using StateNestedVector::StateNestedVector;
    const OneWheelType &wheel ( size_t i ) const {
        return *states_[i];
    }
    OneWheelType &wheel ( size_t i ) {
        return *states_[i];
    }
    const OneWheelType& operator[] ( size_t i ) const {
        return *states_[i];
    }
    OneWheelType& operator[] ( size_t i ) {
        return *states_[i];
    }
    const OneWheelType& at ( size_t i ) const {
        return *states_[i];
    }
    OneWheelType& at ( size_t i ) {
        return *states_[i];
    }
};



//using StateWhInpType   = StateNestedVector<StateArrayScoped<StateWheel>>;
using StateWhInpPtrType = std::shared_ptr<StateWhInpType>;


int main ( int argc, char **argv ) {
    std::cout << "iws utils test" << std::endl;
    StateWhInpPtrType tuw_i2ws_wheel_state = std::make_shared<StateWhInpType>();
    tuw_i2ws_wheel_state->resize ( 2 );
    tuw_i2ws_wheel_state->value ( 0 ) = 10;
    tuw_i2ws_wheel_state->value ( 1 ) = 11;
    tuw_i2ws_wheel_state->value ( 2 ) = 12;
    tuw_i2ws_wheel_state->value ( 3 ) = 13;
    /// printing the state
    std::cout << "tuw      : "  << *tuw_i2ws_wheel_state << std::endl;



    /// convert to stl vector
    std::vector<double> values;
    tuw_i2ws_wheel_state->toSTLVec ( values );

    /// to a classical struct
    I2WS_Wheels_State classical_state;
    classical_state.fromState ( tuw_i2ws_wheel_state );

    /// printing the classical struct
    std::cout << "classical: " << classical_state << std::endl;
    /// printing the classical struct
    std::cout << "tuw      : [" << *tuw_i2ws_wheel_state->state ( 0 ) << ", " << *tuw_i2ws_wheel_state->state ( 1 ) << "]" << std::endl;

    /// accessing varibles
    std::cout << "tuw      tuw_i2ws_wheel_state->stateScoped ( 1 )->value<StateWheel::Steer>() = " << tuw_i2ws_wheel_state->stateScoped ( 1 )->value<StateWheel::Steer>() << std::endl;

    /// accessing varibles
    std::cout << "tuw      ( *tuw_i2ws_wheel_state ) [1].steer()     = " << ( *tuw_i2ws_wheel_state ) [1].steer() << std::endl;
    /// accessing varibles
    std::cout << "tuw      tuw_i2ws_wheel_state->wheel ( 1 ).steer() = " << tuw_i2ws_wheel_state->wheel ( 1 ).steer() << std::endl;

}
