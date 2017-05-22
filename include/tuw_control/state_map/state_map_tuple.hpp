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

#ifndef STATE_MAP_TUPLE_HPP
#define STATE_MAP_TUPLE_HPP

#include <tuw_control/state_map/state_map_base.hpp>
#include <tuw_control/utils.h>

#include <stdexcept>
#include <vector>
#include <memory>
#include <eigen3/Eigen/Eigen>


namespace tuw {


template<class TNumericType, typename... TLeafTypes>
class StateMapTuple : public StateMapBaseCRTP<StateMapTuple<TNumericType, TLeafTypes...>>, 
                      public StateMapBaseVirt<TNumericType>, 
		      public DataBuffer<TNumericType, StateMapBaseCRTP<StateMapTuple<TNumericType, TLeafTypes...>>::MapSize> {

    private  : using ImplType                 = StateMapTuple<TNumericType, TLeafTypes...>;
    private  : using NumericType              = TNumericType;
    private  : using LeafType                 = EmptyLeafType;
    private  : using StateBaseCRTPType        = StateMapBaseCRTP<ImplType>;
    private  : using RootType                 = typename StateMapBaseCRTPTraits<ImplType>::RootType;
    public   : using MapTypeCRTP              = typename StateMapBaseCRTP<StateMapTuple<NumericType, TLeafTypes...>>::MapTypeCRTP;
    private  : using MapTypeVirt              = typename StateMapBaseVirt<NumericType>::MapTypeVirt;
    private  : using StateBaseVirtualType     = typename StateMapBaseVirt<NumericType>::StateBaseVirtualType;
    private  : using DataBufferContainterType = typename DataBuffer<TNumericType, StateBaseCRTPType::MapSize>::DataBufferContainerType;
    private  : static constexpr const bool HasNumericLeaf   = std::is_same<LeafType,NumericType>::value;
    
    private  : MapTypeCRTP                                               map_;
    private  : TNumericType*                                             memStartRef_;
    private  : size_t                                                    mapElementSize_;
    private  : std::tuple <TLeafTypes...>                                subs_;
    private  : std::array <StateBaseVirtualType*, sizeof...(TLeafTypes)> subsBase_;
    private  : StateBaseVirtualType*                                     root_;
    
    public   :  template< bool numericLeaf = HasNumericLeaf, 
	                  int mapSize = StateBaseCRTPType::MapSize,
	                  typename std::enable_if<!numericLeaf && (mapSize != Eigen::Dynamic)>::type* = nullptr >
		StateMapTuple() : 
		    StateBaseCRTPType(),
		    StateMapBaseVirt<NumericType>(),
		    DataBuffer<TNumericType, StateBaseCRTPType::MapSize>(std::make_shared< DataBufferContainterType >()),
		    map_(nullptr), memStartRef_(nullptr), mapElementSize_(StateBaseCRTPType::MapSize), subs_(std::make_tuple(TLeafTypes(this, this->dataBuffer_)...)), root_( this ) {
			size_t i = 0; for_each_tuple( subs_, [this, &i](auto& subI) { subsBase_[i++] = &subI; } );
			this->bindToMemory(&this->dataBuffer_->at(0));
		}
    public   :  template< bool numericLeaf = HasNumericLeaf, 
	                  int mapSize = StateBaseCRTPType::MapSize,
			  typename std::enable_if<!numericLeaf && (mapSize == Eigen::Dynamic)>::type* = nullptr >
		StateMapTuple() : 
		    StateBaseCRTPType(),
		    StateMapBaseVirt<NumericType>(),
		    DataBuffer<TNumericType, StateBaseCRTPType::MapSize>(std::make_shared< DataBufferContainterType >()),
		    map_(nullptr,0), memStartRef_(nullptr), mapElementSize_(0), subs_(std::make_tuple(TLeafTypes(this, this->dataBuffer_)...)), root_( this ) {
			size_t i = 0; for_each_tuple( subs_, [this, &i](auto& subI) { subsBase_[i++] = &subI; mapElementSize_ += subI.mapElementSize_; } );
			if(mapElementSize_ > 0) { this->dataBuffer_->resize(mapElementSize_); this->bindToMemory(&this->dataBuffer_->at(0)); }
		}
    
    public   :  template< bool numericLeaf = HasNumericLeaf, 
	                  int mapSize = StateBaseCRTPType::MapSize, 
			  typename TDataBuffer, 
			  typename std::enable_if<!numericLeaf && (mapSize != Eigen::Dynamic)>::type* = nullptr >
		StateMapTuple(RootType* _root, std::shared_ptr<TDataBuffer>& _dataBuffer) : 
		    StateBaseCRTPType(),
		    StateMapBaseVirt<NumericType>(),
		    DataBuffer<TNumericType, StateBaseCRTPType::MapSize>(),
		    map_(nullptr), memStartRef_(nullptr), mapElementSize_(StateBaseCRTPType::MapSize), subs_(std::make_tuple(TLeafTypes(_root, _dataBuffer)...)), root_( _root ) {
			size_t i = 0; for_each_tuple( subs_, [this, &i](auto& subI) { subsBase_[i++] = &subI; } );
		}
    public   :  template< bool numericLeaf = HasNumericLeaf, 
	                  int mapSize = StateBaseCRTPType::MapSize, 
			  typename TDataBuffer, 
			  typename std::enable_if<!numericLeaf && (mapSize == Eigen::Dynamic)>::type* = nullptr >
		StateMapTuple(RootType* _root, std::shared_ptr<TDataBuffer>& _dataBuffer) : 
		    StateBaseCRTPType(),
		    StateMapBaseVirt<NumericType>(),
		    DataBuffer<TNumericType, StateBaseCRTPType::MapSize>(),
		    map_(nullptr,0), memStartRef_(nullptr), mapElementSize_(0), subs_(std::make_tuple(TLeafTypes(_root, _dataBuffer)...)), root_( _root ) {
			size_t i = 0; for_each_tuple( subs_, [this, &i](auto& subI) { subsBase_[i++] = &subI; mapElementSize_ += subI.mapElementSize_; } );
		}
    public   : virtual ~StateMapTuple  ()                     = default;
    public   : StateMapTuple(const StateMapTuple& _rhs) : 
	DataBuffer<TNumericType, StateBaseCRTPType::MapSize>(_rhs.dataBuffer_),
	map_(_rhs.map_),
	memStartRef_(nullptr),
	mapElementSize_(_rhs.mapElementSize_),
	subs_(_rhs.subs_),
	root_(_rhs.root_) {	    
	    if ( !_rhs.root_->internalCopy_ ) { throw std::runtime_error("Copy-constructor not allowed"); } 
	    size_t i = 0;  for_each_tuple( subs_, [this, &i](auto& subI) { subsBase_[i++] = &subI; } );
    }
    
    private  :  void copyRhsData(const StateMapTuple& _rhs) { 
		   for_each_2_tuples(_rhs.subs_, subs_, [](auto& _subIRhs, auto& _subI){ _subI.copyRhsData(_subIRhs); }); 
	    }
    public   : StateMapTuple& operator=(const StateMapTuple& _rhs) {
	if ( this != &_rhs ) {
	    if(!root_->internalCopy_) {
		copyRhsData(_rhs);
		this->data() = _rhs.data();
	    } else {
		mapElementSize_ = _rhs.mapElementSize_;
		subs_           = _rhs.subs_;
		size_t i = 0; for_each_tuple( subs_, [this, &i](auto& subI) { subsBase_[i++] = &subI; } );
	    }
	}
	return *this;
    }
    public   : StateMapTuple           (StateMapTuple&&)      = default;
    public   : StateMapTuple& operator=(StateMapTuple&&)      = default;
    
    public   : using StateMapBaseCRTP<StateMapTuple<TNumericType, TLeafTypes...>>::data;
    public   : using StateMapBaseCRTP<StateMapTuple<TNumericType, TLeafTypes...>>::sub;
    public   : using StateMapBaseCRTP<StateMapTuple<TNumericType, TLeafTypes...>>::subResize;
    public   : using StateMapBaseCRTP<StateMapTuple<TNumericType, TLeafTypes...>>::subSize;
    public   : using StateMapBaseCRTP<StateMapTuple<TNumericType, TLeafTypes...>>::memStartRef;
    public   : using StateMapBaseCRTP<StateMapTuple<TNumericType, TLeafTypes...>>::bindToMemory;
    
    private  :       MapTypeVirt           dataImplVirt        ()                               override final { return MapTypeVirt(memStartRef_, mapElementSize_); }
    private  : const MapTypeVirt           dataImplVirt        ()                         const override final { return MapTypeVirt(memStartRef_, mapElementSize_); }
    private  :       StateBaseVirtualType& subImplVirt         ( const size_t& _i       )       override final { return subImplVirtDispatch(_i);  }
    private  : const StateBaseVirtualType& subImplVirt         ( const size_t& _i       ) const override final { return subImplVirtDispatch(_i);  }
    private  : void                        subResizeImplVirt   ( const size_t& _size    )       override final { throw std::runtime_error("Cannot resize a tuple"); }
    private  : void                        bindToMemoryImplVirt( TNumericType* _memRef  )       override final { bindToMemoryImplCRTP(_memRef); }
    private  : const size_t                subSizeImplVirt     ()                         const override final { return subSizeImplCRTP(); }
    private  : NumericType* const          memStartRefImplVirt ()                         const override final { return memStartRefImplCRTP(); }
    
    private   : template< bool numericLeaf = HasNumericLeaf, 
	                  typename std::enable_if< ( !numericLeaf ) >::type* = nullptr > 
		StateBaseVirtualType& subImplVirtDispatch ( const size_t& _i ) { return *subsBase_[_i]; }
    private   : template< bool numericLeaf = HasNumericLeaf, 
	                  typename std::enable_if< ( !numericLeaf ) >::type* = nullptr > 
		const StateBaseVirtualType& subImplVirtDispatch ( const size_t& _i ) const { return *subsBase_[_i]; }
    private   : template< bool numericLeaf = HasNumericLeaf, 
	                  typename std::enable_if< (  numericLeaf ) >::type* = nullptr > 
		StateBaseVirtualType& subImplVirtDispatch ( const size_t& _i ) { throw std::runtime_error("Access of numeric Leaf using sub function not allowed"); return *root_; }
    private   : template< bool numericLeaf = HasNumericLeaf, 
	                  typename std::enable_if< (  numericLeaf ) >::type* = nullptr > 
		const StateBaseVirtualType& subImplVirtDispatch ( const size_t& _i ) const { throw std::runtime_error("Access of numeric Leaf using sub function not allowed"); return *root_; }
    private   : template< bool numericLeaf = HasNumericLeaf, 
	                  typename std::enable_if< ( !numericLeaf )  >::type* = nullptr >
		void bindToMemoryImplCRTP( TNumericType* _memRef ) { 
		    size_t mapMemBeginShift(0);
		    memStartRef_ = _memRef;
		    for_each_tuple( subs_, [this, &mapMemBeginShift](auto& subI) { subI.bindToMemory(memStartRef_ + mapMemBeginShift); mapMemBeginShift += subI.mapElementSize_; } );
		    mapElementSize_     = mapMemBeginShift;
		    bindMap();
		}
    private   : template< bool numericLeaf = HasNumericLeaf, 
	                  typename std::enable_if< (  numericLeaf ) >::type* = nullptr > 
		void bindToMemoryImplCRTP( TNumericType* _memRef ) { 
		    memStartRef_ = _memRef;
		    bindMap();
		}
    private   : template< bool dynamicMap = StateBaseCRTPType::MapSize == -1, 
	                  typename std::enable_if< (  dynamicMap ) >::type* = nullptr > 
		void bindMap() { new (&map_) MapTypeCRTP(memStartRef_, mapElementSize_); }
    private   : template< bool dynamicMap = StateBaseCRTPType::MapSize == -1, 
	                  typename std::enable_if< ( !dynamicMap ) >::type* = nullptr > 
		void bindMap() { new (&map_) MapTypeCRTP(memStartRef_); }
		
    private   :                           MapTypeCRTP&                                                      dataImplCRTP ()       { return map_; }
    private   :                     const MapTypeCRTP&                                                      dataImplCRTP () const { return map_; }
    private   : template<size_t _i>       typename std::tuple_element<_i, std::tuple<TLeafTypes...>>::type& subImplCRTP  ()       { return std::get<_i>(this->subs_); }
    private   : template<size_t _i> const typename std::tuple_element<_i, std::tuple<TLeafTypes...>>::type& subImplCRTP  () const { return std::get<_i>(this->subs_); }
    private   : constexpr const size_t       subSizeImplCRTP()                 const { return sizeof...(TLeafTypes); }
    
    private   :                     NumericType* const memStartRefImplCRTP()             const { return memStartRef_; }
    
    //friends
    template<class Derived2>                                    friend class  StateMapBaseCRTP;
    template<class Derived2>                                    friend struct StateMapBaseCRTPTraits;
    template<class TNumericType2>                               friend class  StateMapBaseVirt;
    template<class TNumericType2, class... TLeafTypes2>         friend class  StateMapTuple;
    template<class TNumericType2, class TLeafType2>             friend class  StateMapVector;
    template<class TNumericType2, class TLeafType2, size_t TN2> friend class  StateMapArray;
    template<class TLeafType2>                                  friend struct LeafTypeContClass;
};

namespace {

template<class TNumericType, class... TLeafTypes>
struct StateMapBaseCRTPTraits<StateMapTuple<TNumericType, TLeafTypes...>> {
    using NumericType                     = TNumericType;
    using LeafType                        = EmptyLeafType;
    using LeafTypeExt                     = EmptyLeafType;
    using LeafsTupleType                  = std::tuple<typename TLeafTypes::ImplType...>;
    using LeafsTupleTypeExt               = std::tuple<TLeafTypes...>;
    using RootType                        = StateMapBaseVirt<TNumericType>;
    static constexpr const int  subSize   = sizeof...(TLeafTypes);
    static constexpr const bool isDynamic = false;
};

} //namespace <anonymous>

}

#endif // STATE_MAP_TUPLE_HPP