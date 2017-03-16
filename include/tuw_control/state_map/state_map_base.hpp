/***************************************************************************
 *   Software License Agreement (BSD License)                              *
 *   Copyright (C) 2016 by Markus Bader <markus.bader@tuwien.ac.at         *
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

#ifndef STATE_MAP_BASE_HPP
#define STATE_MAP_BASE_HPP


#include <iostream>
#include <stdexcept>
#include <vector>
#include <memory>
#include <eigen3/Eigen/Eigen>
#include <stdexcept>

namespace tuw {

namespace {
    
template<class TDerived>
struct StateMapBaseCRTPTraits;

class EmptyLeafType{};

template<class ThisStateTraits>constexpr inline 
typename std::enable_if<  std::is_same<typename ThisStateTraits::NumericType,typename ThisStateTraits::LeafType>::value, int>::type getMapSize();
template<class ThisStateTraits>constexpr inline 
typename std::enable_if<(!std::is_same<typename ThisStateTraits::NumericType,typename ThisStateTraits::LeafType>::value)&&
                        (!std::is_same<EmptyLeafType,typename ThisStateTraits::LeafType>::value)                       , int>::type getMapSize();
template<class ThisStateTraits>constexpr inline 
typename std::enable_if<(!std::is_same<typename ThisStateTraits::NumericType,typename ThisStateTraits::LeafType>::value)&&
                        ( std::is_same<EmptyLeafType,typename ThisStateTraits::LeafType>::value)                       , int>::type getMapSize();
  
  
template<typename Tuple, std::size_t II = 0> constexpr inline 
typename std::enable_if<II == std::tuple_size<Tuple>::value, int>::type get_tuple_map_size() { 
    return 0;
}
template<typename Tuple, std::size_t II = 0> constexpr inline 
typename std::enable_if<II < std::tuple_size<Tuple>::value, int>::type get_tuple_map_size() {
    constexpr const int sizeSubThis = getMapSize<StateMapBaseCRTPTraits<typename std::tuple_element<II, Tuple>::type> >();
    constexpr const int sizeSubNext = get_tuple_map_size<Tuple, II + 1>();
    return +  ( (sizeSubThis == -1) || ( sizeSubNext == -1) ) * (-1)
           + !( (sizeSubThis == -1) || ( sizeSubNext == -1) ) * ( sizeSubThis + sizeSubNext );
}


template<class ThisStateTraits> constexpr inline 
typename std::enable_if<  std::is_same<typename ThisStateTraits::NumericType,typename ThisStateTraits::LeafType>::value, int>::type getMapSize() { 
    return ThisStateTraits::subSize; 
} 
template<class ThisStateTraits> constexpr inline 
typename std::enable_if<(!std::is_same<typename ThisStateTraits::NumericType,typename ThisStateTraits::LeafType>::value)&&
                        (!std::is_same<EmptyLeafType,typename ThisStateTraits::LeafType>::value)                       , int>::type getMapSize() { 
      constexpr const int sizeSubThis = ThisStateTraits::subSize;
      constexpr const int sizeSubLeaf = getMapSize<StateMapBaseCRTPTraits<typename ThisStateTraits::LeafType>>();
      return ( (sizeSubThis == -1) || ( sizeSubLeaf == -1) ) * (-1) +
            !( (sizeSubThis == -1) || ( sizeSubLeaf == -1) ) * ( sizeSubThis * sizeSubLeaf ); 
}
template<class ThisStateTraits> constexpr inline 
typename std::enable_if<(!std::is_same<typename ThisStateTraits::NumericType,typename ThisStateTraits::LeafType>::value)&&
                        ( std::is_same<EmptyLeafType,typename ThisStateTraits::LeafType>::value)                       , int>::type getMapSize() { 
      return get_tuple_map_size<typename ThisStateTraits::LeafsTupleType>();
}

}

template<class TDerived>
class StateMapBaseCRTP {
    public   : using NumericType = typename StateMapBaseCRTPTraits<TDerived>::NumericType;
    public   : using LeafType    = typename StateMapBaseCRTPTraits<TDerived>::LeafType;
    public   : using RootType    = typename StateMapBaseCRTPTraits<TDerived>::RootType;
    public   : static constexpr const int  MapSize = getMapSize<StateMapBaseCRTPTraits<TDerived>>();
    private  : static constexpr const bool IsDynamic = StateMapBaseCRTPTraits<TDerived>::isDynamic;
    
    public   : using MapType     = Eigen::Map<Eigen::Matrix<NumericType, MapSize, 1> >;
    
    public   :       MapType&  data        ()                                { return ((TDerived*)(this))->dataImplCRTP ();            }
    public   : const MapType&  data        ()                          const { return ((TDerived*)(this))->dataImplCRTP ();            }
    public   : template<typename leafType = LeafType, typename std::enable_if< ( !std::is_same<EmptyLeafType,leafType>::value ) >::type* = nullptr> 
	             LeafType& sub         ( const size_t& _i )              { return ((TDerived*)(this))->subImplCRTP  ( _i        ); }
    public   : template<typename leafType = LeafType, typename std::enable_if< ( !std::is_same<EmptyLeafType,leafType>::value ) >::type* = nullptr> 
               const LeafType& sub         ( const size_t& _i )        const { return ((TDerived*)(this))->subImplCRTP  ( _i        ); }
               
    public   : template<size_t _i, typename leafType = LeafType, typename std::enable_if< ( !std::is_same<EmptyLeafType,leafType>::value ) >::type* = nullptr>       
	             LeafType& sub ()       { return ((TDerived*)(this))->template subImplCRTP<_i>( ); }
    public   : template<size_t _i, typename leafType = LeafType, typename std::enable_if< ( !std::is_same<EmptyLeafType,leafType>::value ) >::type* = nullptr>       
	       const LeafType& sub () const { return ((TDerived*)(this))->template subImplCRTP<_i>( ); }
	       
    public   : template<size_t _i, typename leafType = LeafType, typename std::enable_if< ( std::is_same<EmptyLeafType,leafType>::value ) >::type* = nullptr>       
	             typename std::tuple_element<_i, typename StateMapBaseCRTPTraits<TDerived>::LeafsTupleType>::type& sub ()       { return ((TDerived*)(this))->template subImplCRTP<_i>(); }
    public   : template<size_t _i, typename leafType = LeafType, typename std::enable_if< ( std::is_same<EmptyLeafType,leafType>::value ) >::type* = nullptr>       
	       const typename std::tuple_element<_i, typename StateMapBaseCRTPTraits<TDerived>::LeafsTupleType>::type& sub () const { return ((TDerived*)(this))->template subImplCRTP<_i>(); }
    
    public   : template<bool isDynamic = IsDynamic, typename std::enable_if< ( isDynamic ) >::type* = nullptr> 
	             void      subResize   ( const size_t& _size     )       { ((TDerived*)(this))->subResizeImplCRTP   ( _size     ); }
	             
    private  :       void      bindToMemory( const size_t& _idxStart )       { ((TDerived*)(this))->bindToMemoryImplCRTP( _idxStart ); }
    
    template<class Derived2>                                    friend class StateMapBaseCRTP;
    template<class TNumericType2>                               friend class StateMapBaseVirt;
    template<class TNumericType2, class... TLeafTypes2>         friend class StateMapTuple;
    template<class TNumericType2, class TLeafType2>             friend class StateMapVector;
    template<class TNumericType2, class TLeafType2, size_t TN2> friend class StateMapArray;
};

template<class TNumericType>
class StateMapBaseVirt {
    public   : using MapType              = Eigen::Map<Eigen::Matrix<TNumericType, Eigen::Dynamic, 1> >;
    public   : using StateBaseVirtualType = StateMapBaseVirt<TNumericType>;
    public   : StateMapBaseVirt(const std::shared_ptr< std::vector<TNumericType> >& _dataBuffer): dataBuffer_(_dataBuffer) {}
    
    public   :       MapType               data        ()                                { return dataImplVirt();           }
    public   : const MapType               data        ()                          const { return dataImplVirt();           }
    public   :       StateBaseVirtualType& sub         ( const size_t& _i        )       { return subImplVirt(_i);          }
    public   : const StateBaseVirtualType& sub         ( const size_t& _i        ) const { return subImplVirt(_i);          }
    public   : void                        subResize   ( const size_t& _size     )       { subResizeImplVirt   (_size    ); }
    
    private  : void                        bindToMemory( const size_t& _idxStart )       { bindToMemoryImplVirt(_idxStart); }
    
    private  : virtual       MapType               dataImplVirt        ()                                = 0;
    private  : virtual const MapType               dataImplVirt        ()                          const = 0;
    private  : virtual       StateBaseVirtualType& subImplVirt         ( const size_t& _i        )       = 0;
    private  : virtual const StateBaseVirtualType& subImplVirt         ( const size_t& _i        ) const = 0;
    private  : virtual void                        subResizeImplVirt   ( const size_t& _size     )       = 0;
    private  : virtual void                        bindToMemoryImplVirt( const size_t& _idxStart )       = 0;
    
    private  : std::shared_ptr< std::vector<TNumericType> > dataBuffer_;
    
    template<class Derived2>                                    friend class StateMapBaseCRTP;
    template<class TNumericType2>                               friend class StateMapBaseVirt;
    template<class TNumericType2, class... TLeafTypes2>         friend class StateMapTuple;
    template<class TNumericType2, class TLeafType2>             friend class StateMapVector;
    template<class TNumericType2, class TLeafType2, size_t TN2> friend class StateMapArray;
};

}

#endif // STATE_MAP_BASE_HPP