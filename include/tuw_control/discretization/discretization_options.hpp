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

#ifndef DISCRETIZATION_OPTIONS_HPP
#define DISCRETIZATION_OPTIONS_HPP

#include <float.h>
#include <memory>
#include <array>

#include <boost/numeric/odeint.hpp>
#include <boost/array.hpp>

#include <algorithm>

namespace tuw {

namespace odeint = boost::numeric::odeint;

template< class TValue = double>
struct heun_abc {
//     struct rk_b {
// 	static boost::array< TValue , stageNr   > b;
//     };
//     struct rk_c {
// 	static boost::array< TValue , stageNr   > c;
//     };
    static constexpr const size_t stageNr = 2;
    static constexpr const boost::array< TValue , 1         > a1 = {{ 1.0 / 2.0 }};
    static constexpr const boost::array< TValue , stageNr   > b  = {{ 0.0      , 1.0       }};
    static constexpr const boost::array< TValue , stageNr   > c  = {{ 0.0      , 1.0 / 2.0 }};
    static auto  a(){ return std::move( boost::fusion::make_vector( a1 ) ); }
//     static auto  b(){ return b_; }
//     static auto  b(){ return c_; }
//     static auto  b(){ return static_cast<TDerived&>(*this)->b; }
};
template< class Value >constexpr const boost::array< Value , 1                       >heun_abc<Value>::a1;
template< class Value >constexpr const boost::array< Value , heun_abc<Value>::stageNr>heun_abc<Value>::b;
template< class Value >constexpr const boost::array< Value , heun_abc<Value>::stageNr>heun_abc<Value>::c;

template<
    template<class> class MethodType,
    class State ,
    class Value = double ,
    class Deriv = State ,
    class Time = Value ,
    class Algebra = odeint::range_algebra ,
    class Operations = odeint::default_operations ,
    class Resizer = odeint::initially_resizer
>
class explicit_generic_rk_impl : public odeint::explicit_generic_rk< MethodType<Value>::stageNr, MethodType<Value>::stageNr, State , Value , Deriv , Time , Algebra , Operations , Resizer > {
public:
    using stepper_base_type  = odeint::explicit_generic_rk< MethodType<Value>::stageNr , MethodType<Value>::stageNr, State , Value , Deriv , Time , Algebra , Operations , Resizer >;
    using state_type         = typename stepper_base_type::state_type;
    using wrapped_state_type = typename stepper_base_type::wrapped_state_type;
    using value_type         = typename stepper_base_type::value_type;
    using deriv_type         = typename stepper_base_type::deriv_type;
    using wrapped_deriv_type = typename stepper_base_type::wrapped_deriv_type;
    using time_type          = typename stepper_base_type::time_type;
    using algebra_type       = typename stepper_base_type::algebra_type;
    using operations_type    = typename stepper_base_type::operations_type;
    using resizer_type       = typename stepper_base_type::resizer_type;
    using stepper_type       = typename stepper_base_type::stepper_type;
    
    explicit_generic_rk_impl( const algebra_type &algebra = algebra_type() ) : 
	stepper_base_type( MethodType<Value>::a(), MethodType<Value>::b , MethodType<Value>::c , algebra ) { 
	
    }
};

}

#endif // DISCRETIZATION_OPTIONS_HPP