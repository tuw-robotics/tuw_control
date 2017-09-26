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

#ifndef STATE_MAP_VECTOR_HPP
#define STATE_MAP_VECTOR_HPP

#include <tuw_control/state_map/state_map_base.hpp>

#include <iostream>
#include <stdexcept>
#include <vector>
#include <memory>
#include <eigen3/Eigen/Eigen>


namespace tuw {

template<class TLeafType>   
class ContainerSubStateMapVector {
    protected: ContainerSubStateMapVector() : subs_() {}
    protected: ContainerSubStateMapVector(size_t _size, const TLeafType& _vals) : subs_(_size, _vals) {}
    protected: std::vector<TLeafType, boost::alignment::aligned_allocator<TLeafType, MapAlignment> > subs_;
};
class ContainerSubStateMapVectorEmpty {};

template<class TNumericType, class TLeafType>
class StateMapVector : public StateMapBaseCRTP<StateMapVector<TNumericType, TLeafType>>, 
                       public StateMapBaseVirt<TNumericType>, 
		       public DataBuffer<TNumericType, StateMapBaseCRTP<StateMapVector<TNumericType, TLeafType>>::MapSize>,
		       public std::conditional<!std::is_same<TLeafType,TNumericType>::value, ContainerSubStateMapVector<TLeafType>, ContainerSubStateMapVectorEmpty>::type {

    private  : using ImplType                 = StateMapVector<TNumericType, TLeafType>;
    private  : using NumericType              = TNumericType;
    private  : using LeafType                 = TLeafType;
    private  : using StateBaseCRTPType        = StateMapBaseCRTP<ImplType>;
    private  : using RootType                 = typename StateMapBaseCRTPTraits<ImplType>::RootType;
    public   : using MapTypeCRTP              = typename StateMapBaseCRTP<ImplType>::MapTypeCRTP;
    private  : using MapTypeVirt              = typename StateMapBaseVirt<NumericType>::MapTypeVirt;
    private  : using StateBaseVirtualType     = typename StateMapBaseVirt<NumericType>::StateBaseVirtualType;
    private  : using DataBufferContainterType = typename DataBuffer<TNumericType, StateBaseCRTPType::MapSize>::DataBufferContainerType;
    private  : static constexpr const bool HasNumericLeaf = std::is_same<LeafType,NumericType>::value;
    
    private  : MapTypeCRTP           map_;
    private  : TNumericType*         memStartRef_;
    private  : size_t                mapElementSize_;
    private  : StateBaseVirtualType* root_;
    
    //special class member functions
    public   :  template< bool numericLeaf = HasNumericLeaf, 
	                  typename std::enable_if< ( !numericLeaf ) >::type* = nullptr > 
		StateMapVector() : 
		    StateBaseCRTPType(),
		    StateMapBaseVirt<NumericType>(),
		    DataBuffer<TNumericType, StateBaseCRTPType::MapSize>(std::make_shared< DataBufferContainterType >()),
		    ContainerSubStateMapVector<TLeafType>(0, LeafType(this, this->dataBuffer_) ),
		    map_(nullptr,0), 
		    memStartRef_(nullptr),
		    mapElementSize_(0),
		    root_( this ) {
		    
		}
    public   :  template< bool numericLeaf = HasNumericLeaf, 
	                  typename std::enable_if< (  numericLeaf ) >::type* = nullptr > 
		StateMapVector(): 
		    StateBaseCRTPType(),
		    StateMapBaseVirt<NumericType>(),
		    DataBuffer<TNumericType, StateBaseCRTPType::MapSize>(std::make_shared< DataBufferContainterType >()),
		    map_(nullptr,0), 
		    memStartRef_(nullptr),
		    mapElementSize_(0), 
		    root_( this ) {
		    
		}
    public   :  StateMapVector(RootType* _root, std::shared_ptr<DataBufferContainterType>& _dataBuffer): 
		    StateBaseCRTPType(),
		    StateMapBaseVirt<NumericType>(),
		    DataBuffer<TNumericType, StateBaseCRTPType::MapSize>(_dataBuffer),
		    map_(nullptr,0), 
		    memStartRef_(nullptr),
		    mapElementSize_(0),
		    root_( _root ) {
		    
		}
    public   : virtual ~StateMapVector  ()                      = default;
    public   : StateMapVector(const StateMapVector& _rhs) : 
	DataBuffer<TNumericType, StateBaseCRTPType::MapSize>(_rhs.dataBuffer_),
	map_(_rhs.map_),
	memStartRef_(nullptr),
	mapElementSize_(_rhs.mapElementSize_),
	root_(_rhs.root_) {
	    if ( !_rhs.root_->internalCopy_ ) { throw std::runtime_error("Copy-constructor not allowed"); } 
	    else  { copyRhsSubs(_rhs); }
    }
    
    private  : template<typename SubType = LeafType, typename std::enable_if<!std::is_same<SubType,NumericType>::value>::type* = nullptr > 
	       void copyRhsData(const StateMapVector& _rhs) { 
		   subResize(_rhs.subs_.size());
		   for ( size_t i = 0; i < this->subs_.size(); ++i ) { this->subs_[i].copyRhsData(_rhs.subs_[i]); }
	    }
    private  : template<typename SubType = LeafType, typename std::enable_if< std::is_same<SubType,NumericType>::value>::type* = nullptr > 
	       void copyRhsData(const StateMapVector& _rhs) { subResize(_rhs.mapElementSize_); }
    private  : template<typename SubType = LeafType, typename std::enable_if< !std::is_same<SubType,NumericType>::value>::type* = nullptr > 
	       void copyRhsSubs(const StateMapVector& _rhs) { for ( size_t i = 0; i < this->subs_.size(); ++i ) { this->subs_[i] = _rhs.subs_[i]; } }
    private  : template<typename SubType = LeafType, typename std::enable_if<  std::is_same<SubType,NumericType>::value>::type* = nullptr > 
	       void copyRhsSubs(const StateMapVector& _rhs) {  }

    public   : StateMapVector& operator=(const StateMapVector& _rhs) {
	if ( this != &_rhs ) {
	    if(!root_->internalCopy_) {
		copyRhsData(_rhs);
		this->data() = _rhs.data();
	    } else {
		mapElementSize_ = _rhs.mapElementSize_;
		copyRhsSubs(_rhs);
	    }
	}
	return *this;
    }
    public   : StateMapVector           (StateMapVector&&) = default;
    public   : StateMapVector& operator=(StateMapVector&&) = default;
    
    public   : using StateMapBaseCRTP<StateMapVector<TNumericType, TLeafType>>::data;
    public   : using StateMapBaseCRTP<StateMapVector<TNumericType, TLeafType>>::sub;
    public   : using StateMapBaseCRTP<StateMapVector<TNumericType, TLeafType>>::subResize;
    public   : using StateMapBaseCRTP<StateMapVector<TNumericType, TLeafType>>::subSize;
    public   : using StateMapBaseCRTP<StateMapVector<TNumericType, TLeafType>>::memStartRef;
    public   : using StateMapBaseCRTP<StateMapVector<TNumericType, TLeafType>>::bindToMemory;
    
    private  :       MapTypeVirt           dataImplVirt        ()                               override final { return dataImplCRTP ();          }
    private  : const MapTypeVirt           dataImplVirt        ()                         const override final { return dataImplCRTP ();          }
    private  :       StateBaseVirtualType& subImplVirt         ( const size_t& _i       )       override final { return subImplVirtDispatch(_i);  }
    private  : const StateBaseVirtualType& subImplVirt         ( const size_t& _i       ) const override final { return subImplVirtDispatch(_i);  }
    private  : void                        subResizeImplVirt   ( const size_t& _size    )       override final { subResizeImplCRTP   (_size    ); }
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
		StateBaseVirtualType& subImplVirtDispatch ( const size_t& _i ) { throw std::runtime_error("Access of numeric Leaf using sub function not allowed"); return *root_; }
    private   : template< bool numericLeaf = HasNumericLeaf, 
	                  typename std::enable_if< (  numericLeaf ) >::type* = nullptr > 
		const StateBaseVirtualType& subImplVirtDispatch ( const size_t& _i ) const { throw std::runtime_error("Access of numeric Leaf using sub function not allowed"); return *root_; }
    
    private   : template< bool numericLeaf = HasNumericLeaf, 
	                  typename std::enable_if< ( !numericLeaf )  >::type* = nullptr > 
		void subResizeImplCRTP(const size_t& _size) {
		    if(_size != this->subs_.size()) {
			const size_t mapElementSizeOld = mapElementSize_;
			const auto distFromStart = std::distance(this->dataBuffer_->data(), memStartRef_);
			root_->internalCopy_ = true;
			this->subs_.resize(_size, LeafType(root_, this->dataBuffer_) );
			root_->internalCopy_ = false;
			mapElementSize_ = 0;
			for (const auto& subI : this->subs_) { mapElementSize_ += subI.mapElementSize_; }
			const auto iteratorStart = this->dataBuffer_->begin() + distFromStart;
			if(mapElementSize_ > mapElementSizeOld) {
			    this->dataBuffer_->insert( iteratorStart+1*(mapElementSizeOld!=0), ( mapElementSize_ - mapElementSizeOld ), 0 );
			} else if (mapElementSize_ < mapElementSizeOld) {
			    this->dataBuffer_->erase( iteratorStart + mapElementSize_, iteratorStart + mapElementSizeOld );
			}
			root_->bindToMemory(this->dataBuffer_->data());
		    }
		}
    private   : template< bool numericLeaf = HasNumericLeaf, 
	                  typename std::enable_if< (  numericLeaf ) >::type* = nullptr > 
		void subResizeImplCRTP(const size_t& _size) {
		    if(_size != mapElementSize_) {
			const size_t mapElementSizeOld = mapElementSize_;
			const auto distFromStart = std::distance(this->dataBuffer_->data(), memStartRef_);
			mapElementSize_ = _size;
			const auto iteratorStart = this->dataBuffer_->begin() + distFromStart;
			if(mapElementSize_ > mapElementSizeOld) {
			    this->dataBuffer_->insert( iteratorStart+1*(mapElementSizeOld!=0), ( mapElementSize_ - mapElementSizeOld ), 0 );
			    root_->bindToMemory(this->dataBuffer_->data());
			} else if (mapElementSize_ < mapElementSizeOld) {
			    this->dataBuffer_->erase( iteratorStart + mapElementSize_, iteratorStart + mapElementSizeOld );
			    root_->bindToMemory(this->dataBuffer_->data());
			}
		    }
		}
    private   : template< bool numericLeaf = HasNumericLeaf, 
	                  typename std::enable_if< ( !numericLeaf )  >::type* = nullptr > 
		void bindToMemoryImplCRTP( TNumericType* _memRef ) { 
		    size_t mapMemBeginShift(0);
		    for(auto& subI : this->subs_) { subI.bindToMemory(_memRef + mapMemBeginShift); mapMemBeginShift += subI.mapElementSize_; } 
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
		
    private   :                           MapTypeCRTP& dataImplCRTP ()                         { return map_; }
    private   :                     const MapTypeCRTP& dataImplCRTP ()                   const { return map_; }
    private   : template< bool numericLeaf = HasNumericLeaf, typename std::enable_if< ( !numericLeaf )  >::type* = nullptr > 
		LeafType&     subImplCRTP  ( const size_t& _i )       { return this->subs_[_i]; }
    private   : template< bool numericLeaf = HasNumericLeaf, typename std::enable_if< ( !numericLeaf )  >::type* = nullptr > 
		const LeafType&     subImplCRTP  ( const size_t& _i ) const { return this->subs_[_i]; }
    private   : template< size_t _i, bool numericLeaf = HasNumericLeaf, typename std::enable_if< ( !numericLeaf )  >::type* = nullptr > 
		LeafType&     subImplCRTP  ()                         { return this->subs_[_i]; }
    private   : template< size_t _i, bool numericLeaf = HasNumericLeaf, typename std::enable_if< ( !numericLeaf )  >::type* = nullptr > 
		const LeafType&     subImplCRTP  ()                   const { return this->subs_[_i]; }
    private   : template< bool numericLeaf = HasNumericLeaf, typename std::enable_if< ( !numericLeaf )  >::type* = nullptr > 
		const size_t        subSizeImplCRTP()                 const { return this->subs_.size(); }
		
    private   : template< bool numericLeaf = HasNumericLeaf, typename std::enable_if< ( numericLeaf )  >::type* = nullptr > 
		LeafType&     subImplCRTP  ( const size_t& _i )       { return this->data()(_i); }
    private   : template< bool numericLeaf = HasNumericLeaf, typename std::enable_if< ( numericLeaf )  >::type* = nullptr > 
		const LeafType&     subImplCRTP  ( const size_t& _i ) const { return this->data()(_i); }
    private   : template< size_t _i, bool numericLeaf = HasNumericLeaf, typename std::enable_if< ( numericLeaf )  >::type* = nullptr > 
		LeafType&     subImplCRTP  ()                         { return this->data()(_i); }
    private   : template< size_t _i, bool numericLeaf = HasNumericLeaf, typename std::enable_if< ( numericLeaf )  >::type* = nullptr > 
		const LeafType&     subImplCRTP  ()                   const { return this->data()(_i); }
    private   : template< bool numericLeaf = HasNumericLeaf, typename std::enable_if< ( numericLeaf )  >::type* = nullptr > 
		const size_t        subSizeImplCRTP()                 const { return mapElementSize_; }
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

template<class TNumericType, class TLeafType>
struct StateMapBaseCRTPTraits<StateMapVector<TNumericType, TLeafType>> : public std::conditional< std::is_same<TNumericType, TLeafType>::value, 
                                                                                                  LeafTypeContNum<TNumericType>, 
												  LeafTypeContClass<TLeafType>
												>::type  {
    using NumericType                     = TNumericType;
    using LeafTypeExt                     = TLeafType;
    using LeafsTupleType                  = EmptyLeafType;
    using LeafsTupleTypeExt               = EmptyLeafType;
    using RootType                        = StateMapBaseVirt<TNumericType>;
    static constexpr const int subSize    = -1;
    static constexpr const bool isDynamic = true;
};

} //namespace <anonymous>

}

#endif // STATE_MAP_VECTOR_HPP
