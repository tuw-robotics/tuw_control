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

#include <stdexcept>
#include <vector>
#include <memory>
#include <eigen3/Eigen/Eigen>


namespace tuw {


template<class TNumericType, class TLeafType>
class StateMapVector : public StateMapBaseCRTP<StateMapVector<TNumericType, TLeafType>>, public StateMapBaseVirt<TNumericType> {
    private  : using NumericType          = TNumericType;
    private  : using LeafType             = TLeafType;
    private  : using StateBaseCRTPType    = StateMapBaseCRTP<StateMapVector<TNumericType, TLeafType>>;
    private  : using RootType             = typename StateMapBaseCRTPTraits<StateMapVector<TNumericType, TLeafType>>::RootType;
    private  : using MapTypeCRTP          = typename StateMapBaseCRTP<StateMapVector<NumericType, LeafType>>::MapType;
    private  : using MapTypeVirt          = typename StateMapBaseVirt<NumericType>::MapType;
    private  : using StateBaseVirtualType = typename StateMapBaseVirt<NumericType>::StateBaseVirtualType;
    private  : static constexpr const bool HasNumericLeaf = std::is_same<LeafType,NumericType>::value;
    
    private  : MapTypeCRTP           map_;
    private  : size_t                dataBufferIdxStart_;
    private  : size_t                mapElementSize_;
    private  : std::vector<LeafType> subs_;
    private  : StateBaseVirtualType* root_;
    private  : StateBaseVirtualType* subPtr_;
    
    public   :  template<bool numericLeaf = HasNumericLeaf, typename std::enable_if< ( !numericLeaf ) >::type* = nullptr> 
	StateMapVector(): 
	StateMapBaseVirt<NumericType>(std::make_shared< std::vector<NumericType> >()), 
	map_(nullptr,0), 
	dataBufferIdxStart_(0),
	mapElementSize_(0), 
	subs_(0,this), 
	root_( this ) {
	
    }
    public   :  template<bool numericLeaf = HasNumericLeaf, typename std::enable_if< (  numericLeaf ) >::type* = nullptr> 
	StateMapVector(): 
	StateMapBaseVirt<NumericType>(std::make_shared< std::vector<NumericType> >()), 
	map_(nullptr,0), 
	dataBufferIdxStart_(0),
	mapElementSize_(0), 
	subs_(0), 
	root_( this ) {
	
    }
    public   : StateMapVector(RootType* _root): 
	StateMapBaseVirt<NumericType>(_root->dataBuffer_), 
	map_(nullptr,0), 
	dataBufferIdxStart_(0),
	mapElementSize_(0),
	root_( _root ) {
	
    }
    
    public   : using StateMapBaseCRTP<StateMapVector<TNumericType, TLeafType>>::data;
    public   : using StateMapBaseCRTP<StateMapVector<TNumericType, TLeafType>>::sub;
    public   : using StateMapBaseCRTP<StateMapVector<TNumericType, TLeafType>>::subResize;
    private  : using StateMapBaseCRTP<StateMapVector<TNumericType, TLeafType>>::bindToMemory;
    
    private  :       MapTypeVirt           dataImplVirt        ()                               override final { return dataImplCRTP ();          }
    private  : const MapTypeVirt           dataImplVirt        ()                         const override final { return dataImplCRTP ();          }
    private  :       StateBaseVirtualType& subImplVirt         ( const size_t& _i       )       override final { return subImplVirtDispatch(_i);  }
    private  : const StateBaseVirtualType& subImplVirt         ( const size_t& _i       ) const override final { return subImplVirtDispatch(_i);  }
    private  : void                        subResizeImplVirt   ( const size_t& _size    )       override final { subResizeImplCRTP   (_size    ); }
    private  : void                        bindToMemoryImplVirt( const size_t& _idxStart)       override final { bindToMemoryImplCRTP(_idxStart); }
    
    private   : template<bool numericLeaf = HasNumericLeaf, typename std::enable_if< ( !numericLeaf ) >::type* = nullptr> 
		StateBaseVirtualType& subImplVirtDispatch ( const size_t& _i ) { return subImplCRTP(_i); }
    private   : template<bool numericLeaf = HasNumericLeaf, typename std::enable_if< ( !numericLeaf ) >::type* = nullptr> 
		const StateBaseVirtualType& subImplVirtDispatch ( const size_t& _i ) const { return subImplCRTP(_i); }
    private   : template<bool numericLeaf = HasNumericLeaf, typename std::enable_if< (  numericLeaf ) >::type* = nullptr> 
		StateBaseVirtualType& subImplVirtDispatch ( const size_t& _i ) { throw std::runtime_error("Access of numeric Leaf using sub function not allowed"); return *root_; }
    private   : template<bool numericLeaf = HasNumericLeaf, typename std::enable_if< (  numericLeaf ) >::type* = nullptr> 
		const StateBaseVirtualType& subImplVirtDispatch ( const size_t& _i ) const { throw std::runtime_error("Access of numeric Leaf using sub function not allowed"); return *root_; }
    
    private   : template<bool numericLeaf = HasNumericLeaf, typename std::enable_if< ( !numericLeaf )  >::type* = nullptr> 
		void subResizeImplCRTP(const size_t& _size) {
		    const size_t mapElementSizeOld = mapElementSize_;
		    subs_.resize(_size, root_);
		    mapElementSize_ = 0;
		    for (const auto& subI : subs_) { mapElementSize_ += subI.mapElementSize_; }
		    if(mapElementSize_ > mapElementSizeOld) {
			this->dataBuffer_->insert( this->dataBuffer_->begin() + dataBufferIdxStart_, ( mapElementSize_ - mapElementSizeOld ), 0 );
			root_->bindToMemory(0);
		    } else if (mapElementSize_ < mapElementSizeOld) {
			this->dataBuffer_->erase( this->dataBuffer_->begin() + dataBufferIdxStart_ + mapElementSize_, this->dataBuffer_->begin() + dataBufferIdxStart_ + mapElementSizeOld );
			root_->bindToMemory(0);
		    }
		}
    private   : template<bool numericLeaf = HasNumericLeaf, typename std::enable_if< (  numericLeaf ) >::type* = nullptr> 
		void subResizeImplCRTP(const size_t& _size) {
		    const size_t mapElementSizeOld = mapElementSize_;
		    mapElementSize_ = _size;
		    if(mapElementSize_ > mapElementSizeOld) {
			this->dataBuffer_->insert( this->dataBuffer_->begin() + dataBufferIdxStart_, ( mapElementSize_ - mapElementSizeOld ), 0 );
			root_->bindToMemory(0);
		    } else if (mapElementSize_ < mapElementSizeOld) {
			this->dataBuffer_->erase( this->dataBuffer_->begin() + dataBufferIdxStart_ + mapElementSize_, this->dataBuffer_->begin() + dataBufferIdxStart_ + mapElementSizeOld );
			root_->bindToMemory(0);
		    }
		}
    private   : template<bool numericLeaf = HasNumericLeaf, typename std::enable_if< ( !numericLeaf )  >::type* = nullptr> 
		void bindToMemoryImplCRTP(const size_t& _idxStart) { 
		    size_t mapMemBeginShift(0);
		    for(auto& subI : subs_) { subI.bindToMemory(_idxStart + mapMemBeginShift); mapMemBeginShift += subI.mapElementSize_; } 
		    dataBufferIdxStart_ = _idxStart;
		    mapElementSize_     = mapMemBeginShift;
		    bindMap();
		}
    private   : template<bool numericLeaf = HasNumericLeaf, typename std::enable_if< (  numericLeaf ) >::type* = nullptr> 
		void bindToMemoryImplCRTP(const size_t& _idxStart) { 
		    dataBufferIdxStart_ = _idxStart;
		    bindMap();
		}
    private   : template<bool dynamicMap = StateBaseCRTPType::MapSize == -1, typename std::enable_if< (  dynamicMap ) >::type* = nullptr> 
		void bindMap() { if ( mapElementSize_ > 0 ) { new (&map_) MapTypeCRTP(&this->dataBuffer_->at(dataBufferIdxStart_), mapElementSize_); } }
    private   : template<bool dynamicMap = StateBaseCRTPType::MapSize == -1, typename std::enable_if< ( !dynamicMap ) >::type* = nullptr> 
		void bindMap() { new (&map_) MapTypeCRTP(&this->dataBuffer_->at(dataBufferIdxStart_)); }
		
    private   :       MapTypeCRTP& dataImplCRTP ()                         { return map_; }
    private   : const MapTypeCRTP& dataImplCRTP ()                   const { return map_; }
    private   :       LeafType&    subImplCRTP  ( const size_t& _i )       { return this->subs_[_i]; }
    private   : const LeafType&    subImplCRTP  ( const size_t& _i ) const { return this->subs_[_i]; }
    private   : template<size_t _i>       LeafType&    subImplCRTP  ()       { return this->subs_[_i]; }
    private   : template<size_t _i> const LeafType&    subImplCRTP  () const { return this->subs_[_i]; }
    
    template<class Derived2>                                    friend class StateMapBaseCRTP;
    template<class TNumericType2>                               friend class StateMapBaseVirt;
    template<class TNumericType2, class... TLeafTypes2>         friend class StateMapTuple;
    template<class TNumericType2, class TLeafType2>             friend class StateMapVector;
    template<class TNumericType2, class TLeafType2, size_t TN2> friend class StateMapArray;
};

namespace {
template<class TNumericType, class TLeafType>
struct StateMapBaseCRTPTraits<StateMapVector<TNumericType, TLeafType>> {
    using NumericType = TNumericType;
    using LeafType    = TLeafType;
    using LeafsTupleType = EmptyLeafType;
    using RootType    = StateMapBaseVirt<TNumericType>;
    static constexpr const int subSize   = -1;
    static constexpr const bool isDynamic = true;
};

}

}

#endif // STATE_MAP_VECTOR_HPP