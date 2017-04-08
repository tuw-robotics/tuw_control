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

#ifndef STATE_MAP_ARRAY_HPP
#define STATE_MAP_ARRAY_HPP

#include <tuw_control/state_map/state_map_base.hpp>

#include <stdexcept>
#include <vector>
#include <memory>
#include <eigen3/Eigen/Eigen>


namespace tuw {
    
template<class TNumericType, class TLeafType, size_t TN>
class StateMapArray : public StateMapBaseCRTP<StateMapArray<TNumericType, TLeafType, TN>>, 
                      public StateMapBaseVirt<TNumericType>, 
		      public DataBuffer<TNumericType, StateMapBaseCRTP<StateMapArray<TNumericType, TLeafType, TN>>::MapSize> {

    private  : using ImplType                 = StateMapArray<TNumericType, TLeafType, TN>;
    private  : using NumericType              = TNumericType;
    private  : using LeafType                 = TLeafType;
    private  : using StateBaseCRTPType        = StateMapBaseCRTP<ImplType>;
    private  : using RootType                 = typename StateMapBaseCRTPTraits<ImplType>::RootType;
    public   : using MapTypeCRTP              = typename StateMapBaseCRTP<ImplType>::MapTypeCRTP;
    private  : using MapTypeVirt              = typename StateMapBaseVirt<NumericType>::MapTypeVirt;
    private  : using StateBaseVirtualType     = typename StateMapBaseVirt<NumericType>::StateBaseVirtualType;
    private  : using DataBufferContainterType = typename DataBuffer<TNumericType, StateBaseCRTPType::MapSize>::DataBufferContainerType;
    private  : static constexpr const bool HasNumericLeaf = std::is_same<LeafType,NumericType>::value;
    
    private  : MapTypeCRTP                              map_;
    private  : TNumericType*                            memStartRef_;
    private  : size_t                                   mapElementSize_;
    private  : std::array<std::shared_ptr<LeafType>,TN> subs_;
    private  : StateBaseVirtualType*                    root_;
    
    //static, not leaf
    public   :  template< bool numericLeaf = HasNumericLeaf, 
	                  int mapSize = StateBaseCRTPType::MapSize, 
			  typename std::enable_if<!numericLeaf && (mapSize != Eigen::Dynamic)>::type* = nullptr >
		StateMapArray() : 
		    StateBaseCRTPType(),
		    StateMapBaseVirt<NumericType>(),
		    DataBuffer<TNumericType, StateBaseCRTPType::MapSize>(std::make_shared< DataBufferContainterType >()),
		    map_(nullptr), memStartRef_(nullptr), mapElementSize_(StateBaseCRTPType::MapSize), root_( this ) {
			for(auto& subI : subs_){ subI = std::make_shared<LeafType>(this, this->dataBuffer_); }
			this->bindToMemory(this->dataBuffer_->data());
		}
    //dynamic, not leaf
    public   :  template< bool numericLeaf = HasNumericLeaf, 
	                  int mapSize = StateBaseCRTPType::MapSize, 
			  typename std::enable_if<!numericLeaf && (mapSize == Eigen::Dynamic)>::type* = nullptr >
		StateMapArray() : 
		    StateBaseCRTPType(),
		    StateMapBaseVirt<NumericType>(),
		    DataBuffer<TNumericType, StateBaseCRTPType::MapSize>(std::make_shared< DataBufferContainterType >()),
		    map_(nullptr,0), memStartRef_(nullptr), mapElementSize_(0), root_( this ) {
			for(auto& subI : subs_){ subI = std::make_shared<LeafType>(this, this->dataBuffer_); mapElementSize_ += subI->mapElementSize_; }
			if(mapElementSize_ > 0) { this->dataBuffer_->resize(mapElementSize_); this->bindToMemory(this->dataBuffer_->data()); }
		}
    //leaf => static
    public   :  template< bool numericLeaf = HasNumericLeaf, 
	                  int mapSize = StateBaseCRTPType::MapSize, 
			  typename std::enable_if<numericLeaf && (mapSize != -1)>::type* = nullptr >
		StateMapArray() : 
		    StateBaseCRTPType(),
		    StateMapBaseVirt<NumericType>(),
		    DataBuffer<TNumericType, StateBaseCRTPType::MapSize>(std::make_shared< DataBufferContainterType >()),
		    map_(nullptr), memStartRef_(nullptr), mapElementSize_(StateBaseCRTPType::MapSize), root_( this ) {
			this->bindToMemory(this->dataBuffer_->data());
		}
    
    //static, not leaf
    public   :  template< bool numericLeaf = HasNumericLeaf, 
	                  int mapSize = StateBaseCRTPType::MapSize, 
			  typename TDataBuffer, 
			  typename std::enable_if<!numericLeaf && (mapSize != Eigen::Dynamic)>::type* = nullptr >
		StateMapArray(RootType* _root, std::shared_ptr<TDataBuffer>& _dataBuffer) : 
		    StateBaseCRTPType(),
		    StateMapBaseVirt<NumericType>(),
		    DataBuffer<TNumericType, StateBaseCRTPType::MapSize>(),
		    map_(nullptr), memStartRef_(nullptr), mapElementSize_(StateBaseCRTPType::MapSize), root_( _root ) {
			for(auto& subI : subs_){ subI = std::make_shared<LeafType>(_root, _dataBuffer); }
		}
    //dynamic, not leaf
    public   :  template< bool numericLeaf = HasNumericLeaf, 
	                  int mapSize = StateBaseCRTPType::MapSize, 
			  typename TDataBuffer, 
			  typename std::enable_if<!numericLeaf && (mapSize == Eigen::Dynamic)>::type* = nullptr >
		StateMapArray(RootType* _root, std::shared_ptr<TDataBuffer>& _dataBuffer) : 
		    StateBaseCRTPType(),
		    StateMapBaseVirt<NumericType>(),
		    DataBuffer<TNumericType, StateBaseCRTPType::MapSize>(),
		    map_(nullptr,0), memStartRef_(nullptr), mapElementSize_(0), root_( _root ) {
			for(auto& subI : subs_){ subI = std::make_shared<LeafType>(_root, _dataBuffer); mapElementSize_ += subI->mapElementSize_; }
		}
    //leaf => static
    public   :  template< bool numericLeaf = HasNumericLeaf, 
	                  int mapSize = StateBaseCRTPType::MapSize, 
			  typename TDataBuffer, 
			  typename std::enable_if<numericLeaf && (mapSize != Eigen::Dynamic)>::type* = nullptr >
		StateMapArray(RootType* _root, std::shared_ptr<TDataBuffer>& _dataBuffer) : 
		    StateBaseCRTPType(),
		    StateMapBaseVirt<NumericType>(),
		    DataBuffer<TNumericType, StateBaseCRTPType::MapSize>(),
		    map_(nullptr), memStartRef_(nullptr), mapElementSize_(StateBaseCRTPType::MapSize), root_( _root ) {
			
		}
    public   : virtual ~StateMapArray  ()                     = default;
    public   : StateMapArray(const StateMapArray& _rhs) : 
	DataBuffer<TNumericType, StateBaseCRTPType::MapSize>(_rhs.dataBuffer_),
	map_(_rhs.map_),
	memStartRef_(nullptr),
	mapElementSize_(_rhs.mapElementSize_),
	root_(_rhs.root_) {
	    
	    
	    if ( !_rhs.root_->internalCopy_ ) { throw std::runtime_error("Copy-constructor not allowed"); } 
	    else if ( !std::is_same<LeafType,NumericType>::value ) {
		for ( size_t i = 0; i < subs_.size(); ++i ) { subs_[i] = std::make_shared<LeafType>(*_rhs.subs_[i].get()); }
	    }
    }
    
    private  : template<typename SubType = LeafType, typename std::enable_if<!std::is_same<SubType,NumericType>::value>::type* = nullptr > 
	       void copyRhsData(const StateMapArray& _rhs) { for ( size_t i = 0; i < subs_.size(); ++i ) { subs_[i]->copyRhsData(*_rhs.subs_[i]); } }
    private  : template<typename SubType = LeafType, typename std::enable_if< std::is_same<SubType,NumericType>::value>::type* = nullptr > 
	       void copyRhsData(const StateMapArray& _rhs) {}
    public   : StateMapArray& operator=(const StateMapArray& _rhs) {
	if ( this != &_rhs ) {
	    if(!root_->internalCopy_) {
		copyRhsData(_rhs);
		this->data() = _rhs.data();
	    } else {
		mapElementSize_ = _rhs.mapElementSize_;
		if ( !std::is_same<LeafType,NumericType>::value ) {
		    for ( size_t i = 0; i < subs_.size(); ++i ) { subs_[i] = std::make_shared<LeafType>(); *subs_[i].get() = *_rhs.subs_[i].get(); }
		}
	    }
	}
	return *this;
    }
	       
    public   : StateMapArray           (StateMapArray&& _rhs) = default;
    public   : StateMapArray& operator=(StateMapArray&& _rhs) = default;
    
    public   : using StateMapBaseCRTP<StateMapArray<TNumericType, TLeafType, TN>>::data;
    public   : using StateMapBaseCRTP<StateMapArray<TNumericType, TLeafType, TN>>::sub;
    public   : using StateMapBaseCRTP<StateMapArray<TNumericType, TLeafType, TN>>::subResize;
    public   : using StateMapBaseCRTP<StateMapArray<TNumericType, TLeafType, TN>>::subSize;
    public   : using StateMapBaseCRTP<StateMapArray<TNumericType, TLeafType, TN>>::memStartRef;
    private  : using StateMapBaseCRTP<StateMapArray<TNumericType, TLeafType, TN>>::bindToMemory;
    
    private  :       MapTypeVirt           dataImplVirt        ()                               override final { return MapTypeVirt(memStartRef_, mapElementSize_); }
    private  : const MapTypeVirt           dataImplVirt        ()                         const override final { return MapTypeVirt(memStartRef_, mapElementSize_); }
    private  :       StateBaseVirtualType& subImplVirt         ( const size_t& _i       )       override final { return subImplVirtDispatch(_i);  }
    private  : const StateBaseVirtualType& subImplVirt         ( const size_t& _i       ) const override final { return subImplVirtDispatch(_i);  }
    private  : void                        subResizeImplVirt   ( const size_t& _size    )       override final { throw std::runtime_error("Cannot resize an array"); }
    private  : void                        bindToMemoryImplVirt( TNumericType* _memRef  )       override final { bindToMemoryImplCRTP(_memRef); }
    private  : const size_t                subSizeImplVirt     ()                         const override final { return subSizeImplCRTP(); }
    private  : NumericType* const          memStartRefImplVirt ()                         const override final { return memStartRefImplCRTP(); }
    
    private   : template< bool numericLeaf = HasNumericLeaf, 
	                  typename std::enable_if< ( !numericLeaf ) >::type* = nullptr > 
		StateBaseVirtualType& subImplVirtDispatch ( const size_t& _i ) { return subImplCRTP(_i); }
    private   : template< bool numericLeaf = HasNumericLeaf, 
	                  typename std::enable_if< ( !numericLeaf ) >::type* = nullptr > 
		const StateBaseVirtualType& subImplVirtDispatch ( const size_t& _i ) const { return subImplCRTP(_i); }
    private   : template< bool numericLeaf = HasNumericLeaf, 
	                  typename std::enable_if< (  numericLeaf ) >::type* = nullptr > 
		StateBaseVirtualType& subImplVirtDispatch ( const size_t& _i ) { throw std::runtime_error("Access of numeric Leaf using virtual \"sub\" function not allowed"); return *root_; }
    private   : template< bool numericLeaf = HasNumericLeaf, 
	                  typename std::enable_if< (  numericLeaf ) >::type* = nullptr > 
		const StateBaseVirtualType& subImplVirtDispatch ( const size_t& _i ) const { throw std::runtime_error("Access of numeric Leaf using virtual \"sub\" function not allowed"); return *root_; }
    private   : template< bool numericLeaf = HasNumericLeaf, 
	                  typename std::enable_if< ( !numericLeaf )  >::type* = nullptr > 
		void bindToMemoryImplCRTP( TNumericType* _memRef ) { 
		    size_t mapMemBeginShift(0);
		    for(auto& subI : subs_) { subI->bindToMemory(_memRef + mapMemBeginShift); mapMemBeginShift += subI->mapElementSize_; } 
		    memStartRef_ = _memRef;
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
		
    
		
    private   :                           MapTypeCRTP&  dataImplCRTP ()                         { return map_; }
    private   :                     const MapTypeCRTP&  dataImplCRTP ()                   const { return map_; }
    private   :                           LeafType&     subImplCRTP  ( const size_t& _i )       { return *this->subs_[_i]; }
    private   :                     const LeafType&     subImplCRTP  ( const size_t& _i ) const { return *this->subs_[_i]; }
    private   : template<size_t _i>       LeafType&     subImplCRTP  ()                         { return *this->subs_[_i]; }
    private   : template<size_t _i> const LeafType&     subImplCRTP  ()                   const { return *this->subs_[_i]; }
    private   :                     const size_t        subSizeImplCRTP()                 const { return subs_.size(); }
    private   :                     NumericType* const  memStartRefImplCRTP()             const { return memStartRef_; }
    
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

template<class TNumericType, class TLeafType, size_t TN>
struct StateMapBaseCRTPTraits<StateMapArray<TNumericType, TLeafType, TN>> : public std::conditional< std::is_same<TNumericType, TLeafType>::value, 
                                                                                                       LeafTypeContNum<TNumericType>, 
												       LeafTypeContClass<TLeafType>
												     >::type  {
    using NumericType                     = TNumericType;
    using LeafTypeExt                     = TLeafType;
    using LeafsTupleType                  = EmptyLeafType;
    using LeafsTupleTypeExt               = EmptyLeafType;
    using RootType                        = StateMapBaseVirt<TNumericType>;
    static constexpr const int subSize    = TN;
    static constexpr const bool isDynamic = false;
};

} //namespace <anonymous>

}

#endif // STATE_MAP_ARRAY_HPP