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
class StateMapTuple : public StateMapBaseCRTP<StateMapTuple<TNumericType, TLeafTypes...>>, public StateMapBaseVirt<TNumericType> {
    private  : using NumericType          = TNumericType;
    private  : using LeafType             = EmptyLeafType;
    private  : using StateBaseCRTPType    = StateMapBaseCRTP<StateMapTuple<TNumericType, TLeafTypes...>>;
    private  : using RootType             = typename StateMapBaseCRTPTraits<StateMapTuple<TNumericType, TLeafTypes...>>::RootType;
    private  : using MapTypeCRTP          = typename StateMapBaseCRTP<StateMapTuple<NumericType, TLeafTypes...>>::MapType;
    private  : using MapTypeVirt          = typename StateMapBaseVirt<NumericType>::MapType;
    private  : using StateBaseVirtualType = typename StateMapBaseVirt<NumericType>::StateBaseVirtualType;
    private  : static constexpr const bool HasNumericLeaf   = std::is_same<LeafType,NumericType>::value;
    
    
    private  : MapTypeCRTP                map_;
    private  : size_t                     dataBufferIdxStart_;
    private  : size_t                     mapElementSize_;
    private  : std::tuple <TLeafTypes...> subs_;
    private  : std::array <StateBaseVirtualType*, sizeof...(TLeafTypes)> subsBase_;
    private  : StateBaseVirtualType*      root_;
    private  : StateBaseVirtualType*      subPtr_;
    
    public   :  template<bool numericLeaf = HasNumericLeaf, int mapSize = StateBaseCRTPType::MapSize>
		StateMapTuple(typename std::enable_if<!numericLeaf && (mapSize != -1)>::type* = nullptr) : 
		    StateBaseVirtualType(std::make_shared< std::vector<NumericType> >()), map_(nullptr), dataBufferIdxStart_(0), mapElementSize_(StateBaseCRTPType::MapSize), subs_(std::make_tuple(TLeafTypes(this)...)), root_( this ) {
			size_t i = 0; for_each_tuple( subs_, [this, &i](auto& subI) { subsBase_[i++] = &subI; } );
			this->dataBuffer_->resize(StateBaseCRTPType::MapSize);
			this->bindToMemory(0);
		    }
    public   :  template<bool numericLeaf = HasNumericLeaf, int mapSize = StateBaseCRTPType::MapSize>
		StateMapTuple(typename std::enable_if<!numericLeaf && (mapSize == -1)>::type* = nullptr) : 
		    StateBaseVirtualType(std::make_shared< std::vector<NumericType> >()), map_(nullptr,0), dataBufferIdxStart_(0), mapElementSize_(0), subs_(std::make_tuple(TLeafTypes(this)...)), root_( this ) {
			size_t i = 0; for_each_tuple( subs_, [this, &i](auto& subI) { subsBase_[i++] = &subI; mapElementSize_ += subI.mapElementSize_; } );
			if(mapElementSize_ > 0) { this->dataBuffer_->resize(mapElementSize_); this->bindToMemory(0); }
		    }
//     public   :  template<bool numericLeaf = HasNumericLeaf, int mapSize = StateBaseCRTPType::MapSize>
// 		StateMapTuple(typename std::enable_if<numericLeaf && (mapSize != -1)>::type* = nullptr) : 
// 		    StateBaseVirtualType(std::make_shared< std::vector<NumericType> >()), map_(nullptr), dataBufferIdxStart_(0), mapElementSize_(StateBaseCRTPType::MapSize), root_( this ) {
// 			
// 		    }
    
    
    public   :  template<bool numericLeaf = HasNumericLeaf, int mapSize = StateBaseCRTPType::MapSize>
		StateMapTuple(RootType* _root, typename std::enable_if<!numericLeaf && (mapSize != -1)>::type* = nullptr) : 
		    StateBaseVirtualType(_root->dataBuffer_), map_(nullptr), dataBufferIdxStart_(0), mapElementSize_(StateBaseCRTPType::MapSize), subs_(std::make_tuple(TLeafTypes(_root)...)), root_( _root ) {
			size_t i = 0; for_each_tuple( subs_, [this, &i](auto& subI) { subsBase_[i++] = &subI; } );
		    }
    public   :  template<bool numericLeaf = HasNumericLeaf, int mapSize = StateBaseCRTPType::MapSize>
		StateMapTuple(RootType* _root, typename std::enable_if<!numericLeaf && (mapSize == -1)>::type* = nullptr) : 
		    StateBaseVirtualType(_root->dataBuffer_), map_(nullptr,0), dataBufferIdxStart_(0), mapElementSize_(0), subs_(std::make_tuple(TLeafTypes(_root)...)), root_( _root ) {
			size_t i = 0; for_each_tuple( subs_, [this, &i](auto& subI) { subsBase_[i++] = &subI; } );
			for(auto& subI : subs_){ subI = std::make_shared<LeafType>(_root); mapElementSize_ += subI->mapElementSize_; }
		    }
//     public   :  template<bool numericLeaf = HasNumericLeaf, int mapSize = StateBaseCRTPType::MapSize>
// 		StateMapTuple(RootType* _root, typename std::enable_if<numericLeaf && (mapSize != -1)>::type* = nullptr) : 
// 		    StateBaseVirtualType(_root->dataBuffer_), map_(nullptr), dataBufferIdxStart_(0), mapElementSize_(StateBaseCRTPType::MapSize), root_( _root ) {
// 			
// 		    }
    
    public   : using StateMapBaseCRTP<StateMapTuple<TNumericType, TLeafTypes...>>::data;
    public   : using StateMapBaseCRTP<StateMapTuple<TNumericType, TLeafTypes...>>::sub;
    public   : using StateMapBaseCRTP<StateMapTuple<TNumericType, TLeafTypes...>>::subResize;
    private  : using StateMapBaseCRTP<StateMapTuple<TNumericType, TLeafTypes...>>::bindToMemory;
    
    private  :       MapTypeVirt           dataImplVirt        ()                               override final { return MapTypeVirt(&this->dataBuffer_->at(dataBufferIdxStart_), mapElementSize_); }
    private  : const MapTypeVirt           dataImplVirt        ()                         const override final { return MapTypeVirt(&this->dataBuffer_->at(dataBufferIdxStart_), mapElementSize_); }
    private  :       StateBaseVirtualType& subImplVirt         ( const size_t& _i       )       override final { return subImplVirtDispatch(_i);  }
    private  : const StateBaseVirtualType& subImplVirt         ( const size_t& _i       ) const override final { return subImplVirtDispatch(_i);  }
    private  : void                        subResizeImplVirt   ( const size_t& _size    )       override final { throw std::runtime_error("Cannot resize an array"); }
    private  : void                        bindToMemoryImplVirt( const size_t& _idxStart)       override final { bindToMemoryImplCRTP(_idxStart); }
    
    private  :       StateBaseVirtualType& subImplVirtDispatch ( const size_t& _i )       { throw std::runtime_error("Cannon use sub(i) when accessing a tuple element from virtual base."); return *root_; }
    private  : const StateBaseVirtualType& subImplVirtDispatch ( const size_t& _i ) const { throw std::runtime_error("Cannon use sub(i) when accessing a tuple element from virtual base."); return *root_; }
    
    private   : template<bool numericLeaf = HasNumericLeaf, typename std::enable_if< ( !numericLeaf ) >::type* = nullptr> 
		StateBaseVirtualType& subImplVirtDispatch ( const size_t& _i ) { return *subsBase_[_i]; }
    private   : template<bool numericLeaf = HasNumericLeaf, typename std::enable_if< ( !numericLeaf ) >::type* = nullptr> 
		const StateBaseVirtualType& subImplVirtDispatch ( const size_t& _i ) const { return *subsBase_[_i]; }
    private   : template<bool numericLeaf = HasNumericLeaf, typename std::enable_if< (  numericLeaf ) >::type* = nullptr> 
		StateBaseVirtualType& subImplVirtDispatch ( const size_t& _i ) { throw std::runtime_error("Access of numeric Leaf using sub function not allowed"); return *root_; }
    private   : template<bool numericLeaf = HasNumericLeaf, typename std::enable_if< (  numericLeaf ) >::type* = nullptr> 
		const StateBaseVirtualType& subImplVirtDispatch ( const size_t& _i ) const { throw std::runtime_error("Access of numeric Leaf using sub function not allowed"); return *root_; }
    private   : template<bool numericLeaf = HasNumericLeaf, typename std::enable_if< ( !numericLeaf )  >::type* = nullptr> 
		void bindToMemoryImplCRTP(const size_t& _idxStart) { 
		    size_t mapMemBeginShift(0);
		    for_each_tuple( subs_, [this, &_idxStart, &mapMemBeginShift](auto& subI) { subI.bindToMemory(_idxStart + mapMemBeginShift); mapMemBeginShift += subI.mapElementSize_; } );
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
    private   : template<size_t _i>       typename std::tuple_element<_i, std::tuple<TLeafTypes...>>::type& subImplCRTP()       { return std::get<_i>(this->subs_); }
    private   : template<size_t _i> const typename std::tuple_element<_i, std::tuple<TLeafTypes...>>::type& subImplCRTP() const { return std::get<_i>(this->subs_); }
    
    template<class Derived2>                                    friend class StateMapBaseCRTP;
    template<class TNumericType2>                               friend class StateMapBaseVirt;
    template<class TNumericType2, class... TLeafTypes2>         friend class StateMapTuple;
    template<class TNumericType2, class TLeafType2>             friend class StateMapVector;
    template<class TNumericType2, class TLeafType2, size_t TN2> friend class StateMapArray; 
};

namespace {

template<class TNumericType, class... TLeafTypes>
struct StateMapBaseCRTPTraits<StateMapTuple<TNumericType, TLeafTypes...>> {
    using NumericType     = TNumericType;
    using LeafType        = EmptyLeafType;
    using LeafsTupleType  = std::tuple<TLeafTypes...>;
    using RootType        = StateMapBaseVirt<TNumericType>;
    static constexpr const int  subSize   = sizeof...(TLeafTypes);
    static constexpr const bool isDynamic = false;
};

}


}

#endif // STATE_MAP_TUPLE_HPP