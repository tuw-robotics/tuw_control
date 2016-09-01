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

#ifndef UTILS_H
#define UTILS_H

#include <memory>
#include <math.h>
#include <tuple>
#include <utility> 


namespace tuw {

template <typename T> inline constexpr
int signum(T x, std::false_type is_signed) {
    return T(0) < x;
}

template <typename T> inline constexpr
int signum(T x, std::true_type is_signed) {
    return (T(0) < x) - (x < T(0));
}

template <typename T> inline constexpr
int signum(T x) {
    return signum(x, std::is_signed<T>());
}

template <typename T> inline constexpr
T normalizeRad ( T _x ) {
    constexpr const double TwoPi = 2 * M_PI;
    _x = fmod( _x        , TwoPi );
    _x = fmod( _x + TwoPi, TwoPi );
    if( _x > M_PI ){ _x -= TwoPi; }
    return _x;
}
template <typename Enumeration>
constexpr auto asInt(Enumeration const value) -> typename std::underlying_type<Enumeration>::type {
    return static_cast<typename std::underlying_type<Enumeration>::type>(value);
}

template<std::intmax_t Num, std::intmax_t Denom = 1 >
struct RatioEval { 
    static constexpr const std::intmax_t num   = Num; 
    static constexpr const std::intmax_t denom = Denom; 
    static constexpr const double        val   = (double)(Num) / (double)(Denom); 
}; 


template<std::size_t II = 0, typename FuncT, typename... Tp>
inline typename std::enable_if<II == sizeof...(Tp), void>::type
  for_each_tuple(std::tuple<Tp...> &, FuncT) { }
  

template<std::size_t II = 0, typename FuncT, typename... Tp>
inline typename std::enable_if<II < sizeof...(Tp), void>::type
  for_each_tuple(std::tuple<Tp...>& t, FuncT f) {
    f(std::get<II>(t));
    for_each_tuple<II + 1, FuncT, Tp...>(t, f);
  }
}

template <class T, class    Tuple>          struct Get_Tuple_Index;
template <class T,          class... Types> struct Get_Tuple_Index<T, std::tuple<T, Types...>> { static const std::size_t value = 0; };
template <class T, class U, class... Types> struct Get_Tuple_Index<T, std::tuple<U, Types...>> { static const std::size_t value = 1 + Get_Tuple_Index<T, std::tuple<Types...>>::value; };

#endif // UTILS_H
