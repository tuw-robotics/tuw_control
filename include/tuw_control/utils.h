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
#include <functional>



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

#if __cplusplus <= 201103L
template <typename T> inline 
T normalizeRad ( T _x ) {
    while ( _x > + M_PI ) _x -= ( 2 * M_PI );
    while ( _x < - M_PI ) _x += ( 2 * M_PI );
    return _x;
}
#else
template <typename T> inline constexpr
T normalizeRad ( T _x ) {
    _x = fmod( _x           , 2 * M_PI );
    _x = fmod( _x + 2 * M_PI, 2 * M_PI );
    if( _x > M_PI ){ _x -= (2 * M_PI); }
    return _x;
}
#endif

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


template<std::size_t II = 0, class FuncT, typename... Tp>
constexpr inline typename std::enable_if<II == sizeof...(Tp), void>::type
  for_each_tuple(std::tuple<Tp...> &, FuncT) { }
  

template<std::size_t II = 0, class FuncT, typename... Tp>
constexpr inline typename std::enable_if<II < sizeof...(Tp), void>::type
  for_each_tuple(std::tuple<Tp...>& t, FuncT f) {
    f(std::get<II>(t));
    for_each_tuple<II + 1, FuncT, Tp...>(t, f);
  }
  
  
template<std::size_t II = 0, class FuncT, typename... Tp>
constexpr inline typename std::enable_if<II == sizeof...(Tp), void>::type
  for_each_2_tuples(const std::tuple<Tp...> &, std::tuple<Tp...> &, FuncT) { }
  

template<std::size_t II = 0, class FuncT, typename... Tp>
constexpr inline typename std::enable_if<II < sizeof...(Tp), void>::type
  for_each_2_tuples(const std::tuple<Tp...>& t1, std::tuple<Tp...>& t2, FuncT f) {
    f(std::get<II>(t1), std::get<II>(t2));
    for_each_2_tuples<II + 1, FuncT, Tp...>(t1, t2, f);
  }
  
  
template<std::size_t II = 0, typename ArrayType, typename... Tp>
inline typename std::enable_if<II == sizeof...(Tp), void>::type
  store_tuple_ptr_to_array(std::tuple<Tp...>&, ArrayType&) { }
  

template<std::size_t II = 0, typename ArrayType, typename... Tp>
inline typename std::enable_if<II < sizeof...(Tp), void>::type
  store_tuple_ptr_to_array(std::tuple<Tp...>& t, ArrayType& _array) {
    _array[II] = &std::get<II>(t);
    for_each_tuple<II + 1, ArrayType, Tp...>(t, _array);
  }

template <class T, class    Tuple>          struct Get_Tuple_Index;
template <class T,          class... Types> struct Get_Tuple_Index<T, std::tuple<T, Types...>> { static const std::size_t value = 0; };
template <class T, class U, class... Types> struct Get_Tuple_Index<T, std::tuple<U, Types...>> { static const std::size_t value = 1 + Get_Tuple_Index<T, std::tuple<Types...>>::value; };


};

// namespace std {
// 
// // helper class
// template<typename R, template<typename...> class Params, typename... Args, std::size_t... I>
// R call_helper(std::function<R(Args...)> const&func, Params<Args...> const&params, std::index_sequence<I...>)
// { return func(std::get<I>(params)...); }
// 
// // "return func(params...)"
// template<typename R, template<typename...> class Params, typename... Args>
// R call(std::function<R(Args...)> const&func, Params<Args...> const&params)
// { return call_helper(func,params,std::index_sequence_for<Args...>{}); }
// 
// 
// template <typename ...Args>
// struct save_it_for_later
// {
//   std::tuple<Args...> params;
//   std::function<void(Args...)> func;
//   void delayed_dispatch() { std::call(func,params); }
// };
// 
// 
// }

#if __cplusplus <= 201103L
/// Helper function needed to upgrade c++ 2011 
namespace std {
template<typename T, typename... Args>
std::unique_ptr<T> make_unique(Args&&... args)
{
    return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
}
namespace {
template <class T, std::size_t N, class... Args>
struct get_number_of_element_from_tuple_by_type_impl
{
    static constexpr auto value = N;
};

template <class T, std::size_t N, class... Args>
struct get_number_of_element_from_tuple_by_type_impl<T, N, T, Args...>
{
    static constexpr auto value = N;
};

template <class T, std::size_t N, class U, class... Args>
struct get_number_of_element_from_tuple_by_type_impl<T, N, U, Args...>
{
    static constexpr auto value = get_number_of_element_from_tuple_by_type_impl<T, N + 1, Args...>::value;
};
};

template <class T, class... Args>
T& get(std::tuple<Args...>& t)
{
    return std::get<get_number_of_element_from_tuple_by_type_impl<T, 0, Args...>::value>(t);
}
};
#endif // __cplusplus <= 201103L
#endif // UTILS_H
