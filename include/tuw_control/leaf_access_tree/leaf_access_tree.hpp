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

#ifndef TUW_LEAF_ACCESS_TREE_HPP
#define TUW_LEAF_ACCESS_TREE_HPP


#include <tuw_control/utils.h>

namespace tuw {

template<typename LeafType, typename FixContainer, int ContSize = 0>
class LeafAccessTreeBase;


///dumb test
template <typename SubType>
struct SubVecContainer { std::vector<SubType> sub_; };
template <typename SubType, size_t N>
struct SubArrContainer { std::array< SubType, N> sub_; };
template <typename... SubType>
struct SubSetContainer { std::tuple<SubType...> sub_; };
struct SubEmpty {  };

template <typename NumType, size_t N>
struct ValArr { std::array<NumType, N> sub_; };
template <typename NumType>
struct ValVec { std::vector<NumType> sub_; };

template <typename NumType>
struct ValRef { std::vector<NumType*> valRef_; };


// template<class Derived, typename LeafType, typename DynContainer              > class StateBaseCRTP;
// template<class Derived, typename LeafType, typename FixContainer, int ContSize = 0> class LeafAccessTreeBaseCRTP;
// template<class Derived, typename LeafType, typename SetContainer              > class StateBaseCRTP;


template<class Derived, typename LeafType, typename FixContainer, int ContSize = 0>
class LeafAccessTreeBaseCRTP/*< Derived, LeafType, FixContainer, ContSize >*/
                    : std::conditional<std::is_same<LeafType,FixContainer>::value,
                                       typename std::conditional<ContSize == -1,//isLeaf is true,
				                                 ValVec<LeafType>,//val is dynamic
								 ValArr<LeafType, ContSize>
				       >::type,
				       typename std::conditional<ContSize == -1, //isLeaf is false
				                                 SubVecContainer<LeafType>, //sub is dynamic
								 SubArrContainer<LeafType, ContSize>
				       >::type
		       >::type {
public:
    LeafAccessTreeBaseCRTP() {
    }

// private  :
    static constexpr const bool isLeaf = std::is_same<LeafType,FixContainer>::value;
    static constexpr const bool subDynamic = ContSize == -1;

    //here have to add efficient ones for full static substates
//     constexpr        LeafType&  val(const size_t& _i) { return *(this->valRef_[_i]); }
//     constexpr  const LeafType&  val(const size_t& _i) { return *(this->valRef_[_i]); }

    template<bool IsLeaf = isLeaf, typename std::enable_if< (  IsLeaf ) >::type* = nullptr> /*constexpr  */      LeafType&  val(const size_t& _i)       { return this->sub_[_i]; }
    template<bool IsLeaf = isLeaf, typename std::enable_if< (  IsLeaf ) >::type* = nullptr> /*constexpr  */const LeafType&  val(const size_t& _i) const { return this->sub_[_i]; }

    //here have to add efficient ones for full static substates
    template<bool IsLeaf = isLeaf, typename std::enable_if< ( !IsLeaf ) >::type* = nullptr> constexpr        LeafType&  val(const size_t& _i) { return *(this->valRef_[_i]); }
    template<bool IsLeaf = isLeaf, typename std::enable_if< ( !IsLeaf ) >::type* = nullptr> constexpr  const LeafType&  val(const size_t& _i) { return *(this->valRef_[_i]); }
};





template<class Derived, typename LeafType, typename...SetTypes, int ContSize>
class LeafAccessTreeBaseCRTP< Derived, LeafType, std::tuple<SetTypes...>, ContSize > {
public:
    LeafAccessTreeBaseCRTP() : sub_( std::make_tuple(SetTypes(/*this*/)...) ) {
// 	store_tuple_ptr_to_array(sub_, subRef_);
    }
    using TupleType = std::tuple<SetTypes...>;
    constexpr        LeafType&  val(const size_t& _i) { return *(this->valRef_[_i]); }
    
    template<size_t _I> /*constexpr*/       typename std::tuple_element<_I, TupleType >::type&  sub ()       { return std::get<_I>(this->sub_); }
    template<size_t _I> /*constexpr*/ const typename std::tuple_element<_I, TupleType >::type&  sub () const { return std::get<_I>(this->sub_); }
//     /*constexpr */      LeafAccessTreeBase<LeafType, std::tuple<SetTypes...>>&  sub (const size_t& _i)       { return this->subRef_[_i]; }
//     /*constexpr */const LeafAccessTreeBase<LeafType, std::tuple<SetTypes...>>&  sub (const size_t& _i) const { return this->subRef_[_i]; }

    std::vector<LeafType*> valRef_;
    TupleType sub_;
//     std::array<LeafAccessTreeBase<LeafType, std::tuple<SetTypes...>>*, (sizeof...(SetTypes))> subRef_;
};

// template<class Derived, typename NumType, int SubSize, class... SubType>
// class StateBaseCRTP : std::conditional<std::is_same<NumType,typename std::tuple_element<0, std::tuple< SubType... >>::type>::value,
//                                        typename std::conditional<SubSize == -1,//isLeaf is true,
// 				                                 ValVec<NumType>,//val is dynamic
// 								 ValArr<SubSize, NumType>
// 				       >::type,
// 				       typename std::conditional<SubSize == -1, //isLeaf is false
// 				                                 SubVecContainer<SubType...>, //sub is dynamic
// 								 typename std::conditional<(sizeof...(SubType) >= 2), //sub is set
// 								                           SubSetContainer<SubType...>,
// 											   SubArrContainer<SubSize, SubType...>
// 								 >::type
// 					>::type
// 		       >::type,
// 		      std::conditional</*my fancy stuff to chekc*/true,
// 		                       SubEmpty,//todo for fancy efficient stuff
// 				       ValRef<NumType>
// 		      >::type  {
// public:
//   StateBaseCRTP() {}
//   const double& val(const size_t& _i) { return ((Derived*)(this))->valImplCRTP(_i); }
// // private  :
//     static constexpr const bool isLeaf = std::is_same<NumType,typename std::tuple_element<0, std::tuple< SubType... >>::type>::value;
//     static constexpr const bool subDynamic = SubSize == -1;
//     static constexpr const bool subIsSet = sizeof...(SubType) >= 2;
// //     constexpr const bool anyDynamic = forAllSubType();
//
//     template<bool IsLeaf = isLeaf, typename std::enable_if< (  IsLeaf ) >::type* = nullptr> constexpr        NumType&  val(const size_t& _i) { return this->sub_[_i]; }
//     template<bool IsLeaf = isLeaf, typename std::enable_if< (  IsLeaf ) >::type* = nullptr> constexpr  const NumType&  val(const size_t& _i) { return this->sub_[_i]; }
//
//     //here have to add efficient ones for full static substates
//     template<bool IsLeaf = isLeaf, typename std::enable_if< ( !IsLeaf ) >::type* = nullptr> constexpr        NumType&  val(const size_t& _i) { return *(this->valRef_[_i]); }
//     template<bool IsLeaf = isLeaf, typename std::enable_if< ( !IsLeaf ) >::type* = nullptr> constexpr  const NumType&  val(const size_t& _i) { return *(this->valRef_[_i]); }
//
//
//
//     template<bool IsLeaf = isLeaf, bool SubIsSet = subIsSet, typename std::enable_if< (!IsLeaf && !subIsSet) >::type* = nullptr> constexpr       State&  sub(const size_t& _i) { return this->sub_[_i]; }
//     template<bool IsLeaf = isLeaf, bool SubIsSet = subIsSet, typename std::enable_if< (!IsLeaf && !subIsSet) >::type* = nullptr> constexpr const State&  sub(const size_t& _i) { return this->sub_[_i]; }
//     template<bool IsLeaf = isLeaf, bool SubIsSet = subIsSet, typename std::enable_if< (!IsLeaf &&  subIsSet) >::type* = nullptr> constexpr       State&  sub(const size_t& _i) { return std::get<_i>(this->sub_); }
//     template<bool IsLeaf = isLeaf, bool SubIsSet = subIsSet, typename std::enable_if< (!IsLeaf &&  subIsSet) >::type* = nullptr> constexpr const State&  sub(const size_t& _i) { return std::get<_i>(this->sub_); }
//
//     template<bool IsLeaf = isLeaf, bool SubIsSet = subIsSet, typename std::enable_if< (!IsLeaf && !subIsSet) >::type* = nullptr> constexpr const size_t& size(const size_t& _i) { return this->sub_.size();    }
//     template<bool IsLeaf = isLeaf, bool SubIsSet = subIsSet, typename std::enable_if< (!IsLeaf &&  subIsSet) >::type* = nullptr> constexpr const size_t& size(const size_t& _i) { return (sizeof...(SubType)); }
//
//     //need subResize();
//
//     //need updateSize();
// };

class LeafAccessTreeBaseVirt {
public:
  virtual ~LeafAccessTreeBaseVirt() = default;
  const double& val(const size_t& _i) { return valImplVirt(_i); }
private:
  virtual const double& valImplVirt (const size_t& _i)  = 0;
};


template<typename LeafType, typename FixContainer, int ContSize>
class LeafAccessTreeBase : public LeafAccessTreeBaseCRTP<LeafAccessTreeBase<LeafType, FixContainer, ContSize>, LeafType, FixContainer, ContSize>, public LeafAccessTreeBaseVirt {
public:
    using LeafAccessTreeBaseCRTP<LeafAccessTreeBase, LeafType, FixContainer, ContSize>::val;
private:
    using LeafAccessTreeBaseVirt::val;
    const double& valImplVirt(const size_t& _i) { return LeafAccessTreeBaseCRTP<LeafAccessTreeBase, LeafType, FixContainer, ContSize>::val(_i); }
//     const double& valImplCRTP(const size_t& _i) { x = 10 * _i; return x; }
    friend class LeafAccessTreeBaseCRTP<LeafAccessTreeBase, LeafType, FixContainer, ContSize>;
    double x;
};



// template<class Derived>
// class StateBase {
// public:
//   StateBase() {}
//   const double& val(const size_t& _i) { return ((Derived*)(this))->valImplCRTP(_i); }
// };
// 
// class StateBaseVirtual {
// public:
//   virtual ~StateBaseVirtual() = default;
//   const double& val(const size_t& _i) { return valImplVirt(_i); }
// private:
//   virtual const double& valImplVirt (const size_t& _i)  = 0;
// };
// 
// class State : public StateBase<State>, public StateBaseVirtual {
// public:
//     using StateBase::val;
// private:
//     using StateBaseVirtual::val;
//     const double& valImplVirt(const size_t& _i) { return valImplCRTP(_i); }
//     const double& valImplCRTP(const size_t& _i) { x = 10 * _i; return x; }
//     friend class StateBase;
//     double x;
// };
// 
// 
// class StateOnlyVirtBase : public StateBaseVirtual {
// public:
// private:
//     const double& valImplVirt(const size_t& _i) { x = 10 * _i; return x; }
//     double x;
// };



// void doTestVirt(double nn, double& x2, StateBaseVirtual* stateBaseVirt) {
//     cout<<"timeVirt: ";
//     { boost::timer::auto_cpu_timer tt;
//     for(size_t i = 0; i < nn; ++i){
// 	for(size_t j = 0; j < nn; ++j){
// 	    x2 += stateBaseVirt->val(i*nn+j);
// 	}
//     }
//     cout<<"(x="<<x2<<") :";
//     }
// }
// void doTestCRTP(double nn, double& x1, StateBase<State>* stateBaseCRTP) {
//     cout<<"timeCRTP: ";
//     { boost::timer::auto_cpu_timer tt;
//     for(size_t i = 0; i < nn; ++i){
// 	for(size_t j = 0; j < nn; ++j){
// 	    x1 += stateBaseCRTP->val(i*nn+j);
// 	}
//     }
//     cout<<"(x="<<x1<<") :";
//     }
// }
// 
// StateBaseCRTP<State, double, std::tuple<State, State> > testtt;
// //     cout<<testtt.isLeaf<<endl;
// //     cout<<testtt.subDynamic<<endl;
// //     cout<<testtt.subIsSet<<endl;
// 
// StateOnlyVirtBase state0;
// State state1, state2;
// 
// 
// StateBase<State>* stateBaseCRTP;
// StateBaseVirtual* stateBaseVirt;
// 
// stateBaseCRTP = &state1;
// //     stateBaseVirt = &state2;
// stateBaseVirt = &state0;
// 
// //     stateBaseVirt = &state0;
// //     stateBaseVirt = &state;
// 
// double x1 = 0;
// double x2 = 0;
// size_t nn = 10000;
// 
// 
// doTestCRTP(nn, x1, stateBaseCRTP);
// doTestVirt(nn, x2, stateBaseVirt);
// 
// cout<<endl;

}

#endif // TUW_LEAF_ACCESS_TREE_HPP
