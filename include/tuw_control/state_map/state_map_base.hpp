/***************************************************************************
 *   Software License Agreement (BSD License)                              *
 *   Copyright (C) 2017 by George Todoran <george.todoran@tuwien.ac.at     *
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
#include <boost/align/aligned_allocator.hpp>

namespace tuw {

constexpr const int MapAlignment = Eigen::Aligned16;
template <typename T, size_t N> struct __attribute__((aligned(16))) aligned_array : public std::array<T,N> {};
    
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

template<class TNumericType>
struct LeafTypeContNum{
    using LeafType = TNumericType;
};
template<class TLeafType>
struct LeafTypeContClass{
    using LeafType = typename TLeafType::ImplType;
};

} //namespace <anonymous>

template<class TNumericType>
class DataBufferVector {
    public   : using ContainerType = std::vector<TNumericType, boost::alignment::aligned_allocator<TNumericType, MapAlignment> >;
    
    //special class member functions
    public   : DataBufferVector(std::shared_ptr< ContainerType > _dataBuffer): dataBuffer_(_dataBuffer) {}
    public   : DataBufferVector()                                          = default;
    public   : ~DataBufferVector          ()                               = default;
    private  : DataBufferVector           (const DataBufferVector& _other) = default;
    private  : DataBufferVector& operator=(const DataBufferVector& _other) = default;
    public   : DataBufferVector           (DataBufferVector&&      _other) = default;
    public   : DataBufferVector& operator=(DataBufferVector&&      _other) = default;
    
    private  : std::shared_ptr< ContainerType > dataBuffer_;
    
    //friends
    template<class Derived2>                                    friend class  StateMapBaseCRTP;
    template<class Derived2>                                    friend struct StateMapBaseCRTPTraits;
    template<class TNumericType2>                               friend class  StateMapBaseVirt;
    template<class TNumericType2, class... TLeafTypes2>         friend class  StateMapTuple;
    template<class TNumericType2, class TLeafType2>             friend class  StateMapVector;
    template<class TNumericType2, class TLeafType2, size_t TN2> friend class  StateMapArray;
    template<class TLeafType2>                                  friend struct LeafTypeContClass;
    template<class TNumericType2, int TMapSize2>                friend class  DataBuffer;
};
template<class TNumericType, int TMapSize>
class DataBufferArray {
    
    private  : using ContainerType = aligned_array<TNumericType, TMapSize>;
    
    //special class member functions
    public   : DataBufferArray(std::shared_ptr< ContainerType > _dataBuffer): dataBuffer_(_dataBuffer) {}
    public   : DataBufferArray()                                         = default;
    public   : ~DataBufferArray          ()                              = default;
    private  : DataBufferArray           (const DataBufferArray& _other) = default;
    private  : DataBufferArray& operator=(const DataBufferArray& _other) = default;
    public   : DataBufferArray           (DataBufferArray&&      _other) = default;
    public   : DataBufferArray& operator=(DataBufferArray&&      _other) = default;
    
    private  : std::shared_ptr< ContainerType > dataBuffer_;
    
    
    //friends
    template<class Derived2>                                    friend class  StateMapBaseCRTP;
    template<class Derived2>                                    friend struct StateMapBaseCRTPTraits;
    template<class TNumericType2>                               friend class  StateMapBaseVirt;
    template<class TNumericType2, class... TLeafTypes2>         friend class  StateMapTuple;
    template<class TNumericType2, class TLeafType2>             friend class  StateMapVector;
    template<class TNumericType2, class TLeafType2, size_t TN2> friend class  StateMapArray;
    template<class TLeafType2>                                  friend struct LeafTypeContClass;
    template<class TNumericType2, int TMapSize2>                friend class  DataBuffer;
};

template<class TNumericType, int TMapSize>
class DataBuffer: public std::conditional<TMapSize == Eigen::Dynamic, DataBufferVector<TNumericType>, DataBufferArray<TNumericType,TMapSize> >::type {
    private  : using DataBufferContainerClass = typename std::conditional<TMapSize == Eigen::Dynamic, DataBufferVector<TNumericType>, DataBufferArray<TNumericType,TMapSize> >::type;
    private  : using DataBufferContainerType  = typename DataBufferContainerClass::ContainerType;
    
    //special class member functions
    public   : DataBuffer           (std::shared_ptr<DataBufferContainerType> _dataBuffer) : DataBufferContainerClass( _dataBuffer ) {}
    public   : DataBuffer           ()                         = default;
    public   : ~DataBuffer          ()                         = default;
    private  : DataBuffer           (const DataBuffer& _other) = default;
    private  : DataBuffer& operator=(const DataBuffer& _other) = default;
    public   : DataBuffer           (DataBuffer&&      _other) = default;
    public   : DataBuffer& operator=(DataBuffer&&      _other) = default;
    
    //friends
    template<class Derived2>                                    friend class  StateMapBaseCRTP;
    template<class Derived2>                                    friend struct StateMapBaseCRTPTraits;
    template<class TNumericType2>                               friend class  StateMapBaseVirt;
    template<class TNumericType2, class... TLeafTypes2>         friend class  StateMapTuple;
    template<class TNumericType2, class TLeafType2>             friend class  StateMapVector;
    template<class TNumericType2, class TLeafType2, size_t TN2> friend class  StateMapArray;
    template<class TLeafType2>                                  friend struct LeafTypeContClass;
};

template<class TDerived>
class StateMapBaseCRTP {
    private  : using NumericType                     = typename StateMapBaseCRTPTraits<TDerived>::NumericType;
    private  : using LeafType                        = typename StateMapBaseCRTPTraits<TDerived>::LeafTypeExt;
    private  : using RootType                        = typename StateMapBaseCRTPTraits<TDerived>::RootType;
    public   : static constexpr const int  MapSize   = getMapSize<StateMapBaseCRTPTraits<TDerived>>();
    public   : using MatrixTypeCRTP                  = Eigen::Matrix<NumericType, MapSize, 1>;
    public   : using MapTypeCRTP                     = Eigen::Map<MatrixTypeCRTP, MapAlignment >;
    private  : static constexpr const bool IsDynamic = StateMapBaseCRTPTraits<TDerived>::isDynamic;
    
    
    //special class member functions
    public   : StateMapBaseCRTP           ()                        = default;
    public   : ~StateMapBaseCRTP          ()                        = default;
    private  : StateMapBaseCRTP           (const StateMapBaseCRTP&) = default;
    private  : StateMapBaseCRTP& operator=(const StateMapBaseCRTP&) = default;
    public   : StateMapBaseCRTP           (StateMapBaseCRTP&&)      = default;
    public   : StateMapBaseCRTP& operator=(StateMapBaseCRTP&&)      = default;
    
    
    public   :       MapTypeCRTP&  data        ()                                { return thisDerived().dataImplCRTP ();            }
    public   : const MapTypeCRTP&  data        ()                          const { return thisDerived().dataImplCRTP ();            }
    public   : template<typename leafType = LeafType, typename std::enable_if< ( !std::is_same<EmptyLeafType,leafType>::value ) >::type* = nullptr> 
	             LeafType& sub         ( const size_t& _i )              { return thisDerived().subImplCRTP  ( _i        ); }
    public   : template<typename leafType = LeafType, typename std::enable_if< ( !std::is_same<EmptyLeafType,leafType>::value ) >::type* = nullptr> 
               const LeafType& sub         ( const size_t& _i )        const { return thisDerived().subImplCRTP  ( _i        ); }
               
    public   : template<size_t _i, typename leafType = LeafType, typename std::enable_if< ( !std::is_same<EmptyLeafType,leafType>::value ) >::type* = nullptr>       
	             LeafType& sub ()       { return thisDerived().template subImplCRTP<_i>( ); }
    public   : template<size_t _i, typename leafType = LeafType, typename std::enable_if< ( !std::is_same<EmptyLeafType,leafType>::value ) >::type* = nullptr>       
	       const LeafType& sub () const { return thisDerived().template subImplCRTP<_i>( ); }
	       
    public   : template<size_t _i, typename leafType = LeafType, typename std::enable_if< ( std::is_same<EmptyLeafType,leafType>::value ) >::type* = nullptr>       
	             typename std::tuple_element<_i, typename StateMapBaseCRTPTraits<TDerived>::LeafsTupleTypeExt>::type& sub ()       { return thisDerived().template subImplCRTP<_i>(); }
    public   : template<size_t _i, typename leafType = LeafType, typename std::enable_if< ( std::is_same<EmptyLeafType,leafType>::value ) >::type* = nullptr>       
	       const typename std::tuple_element<_i, typename StateMapBaseCRTPTraits<TDerived>::LeafsTupleTypeExt>::type& sub () const { return thisDerived().template subImplCRTP<_i>(); }
    
    public   : template<bool isDynamic = IsDynamic, typename std::enable_if< ( isDynamic ) >::type* = nullptr> 
	             void      subResize   ( const size_t& _size     )       { thisDerived().subResizeImplCRTP   ( _size     ); }
	             
    public   :       void      bindToMemory( NumericType* _memRef    )       { thisDerived().bindToMemoryImplCRTP( _memRef   ); }
    public   : constexpr const size_t    subSize     ()                          const { return thisDerived().subSizeImplCRTP(); }
    public   : NumericType* const memStartRef ()                          const { return thisDerived().memStartRefImplCRTP(); }
    
    private  :       TDerived& thisDerived()       { return static_cast<      TDerived&>(*this); }
    private  : const TDerived& thisDerived() const { return static_cast<const TDerived&>(*this); }
    
    //friends
    template<class Derived2>                                    friend class  StateMapBaseCRTP;
    template<class Derived2>                                    friend struct StateMapBaseCRTPTraits;
    template<class TNumericType2>                               friend class  StateMapBaseVirt;
    template<class TNumericType2, class... TLeafTypes2>         friend class  StateMapTuple;
    template<class TNumericType2, class TLeafType2>             friend class  StateMapVector;
    template<class TNumericType2, class TLeafType2, size_t TN2> friend class  StateMapArray;
    template<class TLeafType2>                                  friend struct LeafTypeContClass;
};

template<class TDerived>
constexpr const int  StateMapBaseCRTP<TDerived>::MapSize;

template<class TNumericType>
class StateMapBaseVirt {
    private  : using MapTypeVirt          = Eigen::Map<Eigen::Matrix<TNumericType, Eigen::Dynamic, 1>, MapAlignment >;
    private  : using StateBaseVirtualType = StateMapBaseVirt<TNumericType>;
    
    //special class member functions
    public   : StateMapBaseVirt           () : internalCopy_(false) {}
    public   : virtual ~StateMapBaseVirt  ()                        = default;
    private  : StateMapBaseVirt           (const StateMapBaseVirt&) = default;
    private  : StateMapBaseVirt& operator=(const StateMapBaseVirt&) = default;
    public   : StateMapBaseVirt           (StateMapBaseVirt&&)      = default;
    public   : StateMapBaseVirt& operator=(StateMapBaseVirt&&)      = default;
    
    public   :       MapTypeVirt           data        ()                                { return dataImplVirt();           }
    public   : const MapTypeVirt           data        ()                          const { return dataImplVirt();           }
    public   :       StateBaseVirtualType& sub         ( const size_t& _i        )       { return subImplVirt(_i);          }
    public   : const StateBaseVirtualType& sub         ( const size_t& _i        ) const { return subImplVirt(_i);          }
    public   : void                        subResize   ( const size_t& _size     )       { subResizeImplVirt   (_size    ); }
    public   : const size_t&               subSize     ()                          const { return subSizeImplVirt(); }
    public   : TNumericType* const         memStartRef ()                          const { return memStartRefImplVirt(); }
    
    private  : void                        bindToMemory( TNumericType* _memRef   )       { bindToMemoryImplVirt(_memRef);   }
    
    private  : virtual       MapTypeVirt           dataImplVirt        ()                                = 0;
    private  : virtual const MapTypeVirt           dataImplVirt        ()                          const = 0;
    private  : virtual       StateBaseVirtualType& subImplVirt         ( const size_t& _i        )       = 0;
    private  : virtual const StateBaseVirtualType& subImplVirt         ( const size_t& _i        ) const = 0;
    private  : virtual void                        subResizeImplVirt   ( const size_t& _size     )       = 0;
    private  : virtual const size_t                subSizeImplVirt     ()                          const = 0;
    private  : virtual TNumericType* const         memStartRefImplVirt ()                          const = 0;
    private  : virtual void                        bindToMemoryImplVirt( TNumericType* _memRef   )       = 0;
    
    private  : bool internalCopy_;
    
    //friends
    template<class Derived2>                                    friend class  StateMapBaseCRTP;
    template<class Derived2>                                    friend struct StateMapBaseCRTPTraits;
    template<class TNumericType2>                               friend class  StateMapBaseVirt;
    template<class TNumericType2, class... TLeafTypes2>         friend class  StateMapTuple;
    template<class TNumericType2, class TLeafType2>             friend class  StateMapVector;
    template<class TNumericType2, class TLeafType2, size_t TN2> friend class  StateMapArray;
    template<class TLeafType2>                                  friend struct LeafTypeContClass;
};

} // namespace tuw

#endif // STATE_MAP_BASE_HPP
