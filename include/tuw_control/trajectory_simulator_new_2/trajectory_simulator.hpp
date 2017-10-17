/***************************************************************************
 *   Software License Agreement (BSD License)                              *
 *   Copyright (C) 2017 by Horatiu George Todoran <todorangrg@gmail.com>   *
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

#ifndef TRAJECTORY_SIMULATOR2_HPP
#define TRAJECTORY_SIMULATOR2_HPP

#include <tuw_control/state_sim/state_sim.hpp>
#include <tuw_control/param_func_new/param_func_dist.hpp>
#include <tuw_control/costs_evaluator/costs_evaluator.hpp>

#include <functional>
#include <memory>
#include <queue>

namespace tuw {

///@brief Structure containing an evaluation arc and a state pointer.
template<typename TNumType, typename TStateType>
struct LatticePoint {
    using StateSPtr = std::shared_ptr<TStateType>;

    LatticePoint()                                      : arc( -1 ), statePtr(std::make_shared<TStateType>()) {}
    LatticePoint( StateSPtr  _statePtr) : arc( -1 ), statePtr( _statePtr ) {}
    LatticePoint( StateSPtr  _statePtr, const size_t& _latticeIdx) : arc( -1 ), statePtr( _statePtr ), latticeIdx(_latticeIdx) {}
    LatticePoint( TNumType _arc                       ) : arc(_arc), statePtr( nullptr   ) {}
    LatticePoint( TNumType _arc, StateSPtr  _statePtr ) : arc(_arc), statePtr( _statePtr ) {}
    virtual ~LatticePoint() {}

    TNumType    arc;
    StateSPtr   statePtr;
    size_t      latticeIdx;
};


template<class TDerived, typename TNumType, class TSimType, class... TLatticeCostFuncs >
class LatticeTypeBaseCRTP {

public   :
    using NumType         = TNumType;
public   :
    using StateType       = typename TSimType::StateType;
public   :
    using StateSPtr       = std::shared_ptr<StateType>;

public   :
    LatticeTypeBaseCRTP           ()                           = default;
public   :
    virtual ~LatticeTypeBaseCRTP  ()                           = default;
public   :
    LatticeTypeBaseCRTP           (const LatticeTypeBaseCRTP&) = default;
public   :
    LatticeTypeBaseCRTP& operator=(const LatticeTypeBaseCRTP&) = default;
public   :
    LatticeTypeBaseCRTP           (LatticeTypeBaseCRTP&&)      = default;
public   :
    LatticeTypeBaseCRTP& operator=(LatticeTypeBaseCRTP&&)      = default;

public   :
    void computeLatticeStructure(TSimType& _sim, const size_t _latticeIdx) {
        computeLatticeArcs(_sim, latticeArcs_);
        const size_t& latticeSize = latticeArcs_.size();
        if(latticeSize != lattice.size()) {
            lattice.resize( latticeSize, LatticePoint<NumType, StateType>(nullptr, _latticeIdx) );
            latticeVecIterEnd_   = lattice.end();
        }
        latticeVecCacheIter_ = lattice.begin();
        for ( size_t i = 0; i < latticeSize; ++i ) {
            lattice[i].arc = latticeArcs_[i];
        }
    }
public   :
    void         computeLatticeArcs(TSimType& _sim, std::vector<NumType>& _latticeArcs) {
        thisDerived().computeLatticeArcsImpl(_sim, latticeArcs_);
    }
public   :
    virtual void precompute(TSimType& _sim) = 0;
public   :
    template<size_t FuncsNr = 0, size_t TupSize = std::tuple_size<std::tuple<TLatticeCostFuncs...>>::value, typename std::enable_if< (TupSize  > 0) >::type* = nullptr >
    static constexpr const size_t costFuncsNr() { /*const*/
        return std::tuple_size<typename std::tuple_element<FuncsNr, std::tuple<TLatticeCostFuncs...>>::type>::value;
    }
public   :
    template<size_t FuncsNr = 0, size_t TupSize = std::tuple_size<std::tuple<TLatticeCostFuncs...>>::value, typename std::enable_if< (TupSize == 0) >::type* = nullptr >
    static constexpr const size_t costFuncsNr() { /*const*/
        return 0;
    }
public   :
    static constexpr const size_t costFuncsTypesNr() { /*const*/
        return std::tuple_size<std::tuple<TLatticeCostFuncs...>>::value;
    }
public   :
    template<size_t FuncsNr = 0, size_t TupSize = std::tuple_size<std::tuple<TLatticeCostFuncs...>>::value, typename std::enable_if< (TupSize  > 0) >::type* = nullptr >
    void evaluate          (const auto& _x, const size_t& _i, TSimType& _sim, auto& _ansPtr) {
        for_each_tuple( std::get<FuncsNr>(costFuncs_), [this, &_x, &_sim, &_ansPtr, &_i](auto& costFuncI) {
            *_ansPtr = costFuncI.f(_x, _i, _sim, this->thisDerived() );
            _ansPtr++;
        }
                      );
    }
public   :
    template<size_t FuncsNr = 0, size_t TupSize = std::tuple_size<std::tuple<TLatticeCostFuncs...>>::value, typename std::enable_if< (TupSize == 0) >::type* = nullptr >
    void evaluate          (const auto& _x, const size_t& _i, TSimType& _sim, auto& _ansPtr) { }
public   :
    template<size_t FuncsNr = 0, size_t TupSize = std::tuple_size<std::tuple<TLatticeCostFuncs...>>::value, typename std::enable_if< (TupSize  > 0) >::type* = nullptr >
    void evaluateWithGrad (const auto& _x, const size_t& _i, const auto& _gradX, TSimType& _sim, auto& _ansPtr, auto& _ansGradPtr, const size_t& elSize) {
        for_each_tuple( std::get<FuncsNr>(costFuncs_), [this, &_x, &_gradX, &_sim,  &_ansPtr, &_ansGradPtr, &elSize, &_i](auto& costFuncI) {
            *_ansPtr = costFuncI.f(_x, _i, _sim, this->thisDerived() );
            _ansPtr++;
            Eigen::Map<Eigen::Matrix<TNumType, -1,1>, MapAlignment> map(_ansGradPtr, elSize, 1);
            costFuncI.gradF(map, _x, _gradX, _i, _sim, this->thisDerived() );
            _ansGradPtr+=elSize;
        }
                      );
    }
public   :
    template<size_t FuncsNr = 0, size_t TupSize = std::tuple_size<std::tuple<TLatticeCostFuncs...>>::value, typename std::enable_if< (TupSize == 0) >::type* = nullptr >
    void evaluateWithGrad  (const auto& _x, const size_t& _i, const auto& _gradX, TSimType& _sim, auto& _ansPtr, auto& _ansGradPtr, const size_t& elSize) { }

private  :
    TDerived& thisDerived()       {
        return static_cast<      TDerived&>(*this);
    }
private  :
    const TDerived& thisDerived() const {
        return static_cast<const TDerived&>(*this);
    }

public   :
    std::vector<LatticePoint<NumType, StateType>> lattice;
private  :
    std::vector<NumType> latticeArcs_;
private  :
    std::tuple<TLatticeCostFuncs...> costFuncs_;
private  :
    typename std::vector<LatticePoint<NumType, StateType>>::iterator latticeVecCacheIter_;
private  :
    typename std::vector<LatticePoint<NumType, StateType>>::iterator latticeVecIterEnd_;

    template<typename TNumType2, typename TSimType2, bool TUseStateNm2, template<typename, typename> class... TLatticeTypes2> friend class TrajectorySimulator;
    template<typename T, typename... TTuple> friend size_t bindFromPartLatticesToSimLattice(std::tuple<TTuple...>&, std::vector<T>&);
};


template<std::size_t II = 0, class FuncT, typename... Tp>
constexpr inline typename std::enable_if<II == sizeof...(Tp), void>::type
for_each_tuple_class(std::tuple<Tp...> &, FuncT&) { }


template<std::size_t II = 0, class FuncT, typename... Tp>
constexpr inline typename std::enable_if<II < sizeof...(Tp), void>::type
for_each_tuple_class(std::tuple<Tp...>& t, FuncT& f) {
    f[II] = [&t](const auto& _a, const auto& _b, const auto& _c, auto& _d ) {
        std::get<II>(t).computeDArcIdPImpl(_a, _b, _c, _d);
    };
    for_each_tuple_class<II + 1, FuncT, Tp...>(t, f);
}

template<std::size_t IIMax,  std::size_t II = 0>
constexpr inline typename std::enable_if<(II == IIMax), void>::type
get_costs_sizes(auto& partLatI, const size_t& _i, auto& _sizeCostsPerPartLattice, auto& _sizeCostsPerType) { }


template<std::size_t IIMax, std::size_t II = 0>
constexpr inline typename std::enable_if<(II < IIMax), void>::type
get_costs_sizes(auto& partLatI, const size_t& _i, auto& _sizeCostsPerPartLattice, auto& _sizeCostsPerType) {
    size_t partLatICostsTypeJ = partLatI.lattice.size() * partLatI.template costFuncsNr<II>();
    _sizeCostsPerPartLattice[_i] += partLatICostsTypeJ;
    _sizeCostsPerType       [II] += partLatICostsTypeJ;
    get_costs_sizes<IIMax, II + 1>(partLatI, _i, _sizeCostsPerPartLattice, _sizeCostsPerType);
}


template<typename TNumType, typename TSimType, bool TUseStateNm, template<typename, typename> class... TLatticeTypes>
class TrajectorySimulator {

public   :
    using StateSimSPtr    = std::shared_ptr<TSimType>;
public   :
    using StateType       = typename TSimType::StateType;
public   :
    using StateSPtr       = std::shared_ptr<StateType>;
public   :
    using StateForSimType = typename TSimType::StateForSimType;
public   :
    using LatticePointType = LatticePoint<TNumType, StateType>;
public   :
    static constexpr const bool CanComputeStateGrad = !std::is_same<EmptyGradType, typename StateMapBaseTraits<StateType>::StateWithGradNmType>::value;

public   :
    static constexpr const size_t CostFuncsTypesNr = std::tuple_element<0, std::tuple<TLatticeTypes<TNumType, TSimType>...>>::type::costFuncsTypesNr();

public   :
    TrajectorySimulator           () : stateSim_(std::make_shared<TSimType>()) {
        for_each_tuple_class( partialLattices_, correctStateGradFunc );
        for(size_t i = 0; i < gradCostsMap_.size(); ++i) {
            gradCostsMap_[i] = std::make_shared<Eigen::Map<Eigen::Matrix<TNumType,-1,-1, Eigen::RowMajor>, MapAlignment>>(nullptr,0,0);
        }
    }
public   :
    TrajectorySimulator           ( StateSimSPtr& _stateSim ) : stateSim_(_stateSim) {
        for_each_tuple_class( partialLattices_, correctStateGradFunc );
        for(size_t i = 0; i < gradCostsMap_.size(); ++i) {
            gradCostsMap_[i] = std::make_shared<Eigen::Map<Eigen::Matrix<TNumType,-1,-1, Eigen::RowMajor>, MapAlignment>>(nullptr,0,0);
        }
    }


    ///@brief Reference of the state simulator object.
public   :
    StateSimSPtr& stateSim () {
        return stateSim_;
    }
    ///@brief Const reference of the state simulator object.
public   :
    const StateSimSPtr& stateSim () const {
        return stateSim_;
    }
public   :
    LatticePointType& simLatticeI( const size_t& _i)       {
        return simulationLattice_[_i];
    }
public   :
    const LatticePointType& simLatticeI( const size_t& _i) const {
        return simulationLattice_[_i];
    }
public   :
    size_t simLatticeSize() const {
        return simulationLatticeActiveSize_;
    }
public   :
    template< template<typename,typename> class TLatticeType>
    TLatticeType<TNumType, TSimType>& partialLattice () {
        return std::get<TLatticeType<TNumType, TSimType>>(partialLattices_);
    }
public   :
    template< template<typename,typename> class TLatticeType>
    const TLatticeType<TNumType, TSimType>& partialLattice () const {
        return std::get<TLatticeType<TNumType, TSimType>>(partialLattices_);
    }
public   :
    template< size_t TLatticeIdx>       auto& partialLattice ()       {
        return std::get<TLatticeIdx>(partialLattices_);
    }
public   :
    template< size_t TLatticeIdx> const auto& partialLattice () const {
        return std::get<TLatticeIdx>(partialLattices_);
    }

private  :
    using AdvanceFunction = void (TrajectorySimulator<TNumType, TSimType, TUseStateNm, TLatticeTypes...>::*)(const TNumType&);
private  :
    AdvanceFunction advanceFunc;
private  :
    void advanceFuncSimEmpty(const TNumType& _arcNow) { }
private  :
    void advanceFuncSim     (const TNumType& _arcNow) {
        stateSim_->advance( _arcNow );
    }
private  :
    template< bool canComputeStateGrad = CanComputeStateGrad,
              typename std::enable_if< ( canComputeStateGrad ) >::type* = nullptr >
    void advanceFuncSimGrad(const TNumType& _arcNow) {
        stateSim_->advanceWithGrad( _arcNow );
    }

private  :
    bool simulatingWithGrad;
private  :
    template< bool useStateSimNm = TUseStateNm, typename std::enable_if< ( useStateSimNm ) >::type* = nullptr>
    void bindAdvanceFunc()     {
        advanceFunc = &TrajectorySimulator<TNumType, TSimType, TUseStateNm, TLatticeTypes...>::advanceFuncSim;
    }
private  :
    template< bool useStateSimNm = TUseStateNm, typename std::enable_if< ( !useStateSimNm ) >::type* = nullptr>
    void bindAdvanceFunc()     {
        advanceFunc = &TrajectorySimulator<TNumType, TSimType, TUseStateNm, TLatticeTypes...>::advanceFuncSimEmpty;
    }
private  :
    template< bool useStateSimNm = TUseStateNm, typename std::enable_if< ( useStateSimNm ) >::type* = nullptr>
    void bindAdvanceFuncGrad() {
        advanceFunc = &TrajectorySimulator<TNumType, TSimType, TUseStateNm, TLatticeTypes...>::advanceFuncSimGrad;
    }
private  :
    template< bool useStateSimNm = TUseStateNm, typename std::enable_if< ( !useStateSimNm ) >::type* = nullptr>
    void bindAdvanceFuncGrad() {
        advanceFunc = &TrajectorySimulator<TNumType, TSimType, TUseStateNm, TLatticeTypes...>::advanceFuncSimEmpty;
    }

public   :
    void simulateTrajectory (const bool& _saveLatticeStates = false) {
        bindAdvanceFunc();
        simulatingWithGrad = false;
        simulateTrajectoryImpl(_saveLatticeStates);
    }
public   :
    template< bool canComputeStateGrad = CanComputeStateGrad,
              typename std::enable_if< ( canComputeStateGrad ) >::type* = nullptr >
    void simulateTrajectoryWithGrad (const bool& _saveLatticeStates = false) {
        bindAdvanceFuncGrad();
        simulatingWithGrad = true;
        simulateTrajectoryImpl(_saveLatticeStates);
    }

    template<std::size_t IIMax,  std::size_t II = 0>
    constexpr typename std::enable_if<(II == IIMax), void>::type
    eval_all_costs(TNumType*& _cfPtr, TNumType*& _cfGradPtr, const size_t& _optParamSize) { }


    template<std::size_t IIMax, std::size_t II = 0>
    constexpr typename std::enable_if<(II < IIMax), void>::type
    eval_all_costs(TNumType*& _cfPtr, TNumType*& _cfGradPtr, const size_t& _optParamSize) {
        evaluateCosts<II>(_cfPtr, _cfGradPtr, _optParamSize);
        eval_all_costs<IIMax, II + 1>(_cfPtr, _cfGradPtr, _optParamSize);
    }
public   :
    void simulateTrajectoryImpl (const bool& _saveLatticeStates = false) {
        size_t sizeCosts = 0;
        resizeSimLattice(populatePartLattices(sizeCosts));
        simulationLatticeActiveSize_ = bindFromPartLatticesToSimLattice(_saveLatticeStates);
        if(sizeCosts > 0) {
            size_t optParamSize = stateSim_->state().stateGradCf().sub(0).data().size();
            resizeCosts(optParamSize, simulatingWithGrad);
            TNumType* cfPtr     = costs_    .memStartRef();
            TNumType* cfGradPtr = gradCosts_.memStartRef();
            eval_all_costs<CostFuncsTypesNr>(cfPtr, cfGradPtr, optParamSize);
        }

    }
public  :
    void resizeInternal() {
        size_t sizeCosts = 0;
        populatePartLattices(sizeCosts);
        if(sizeCosts > 0) {
            size_t optParamSize = stateSim_->state().stateGradCf().sub(0).data().size();
            resizeCosts(optParamSize, true);
        }
    }
private :
    size_t populatePartLattices(size_t& _sizeCosts) {
        stateSim_->toState0();
        size_t totalPartLatticePtsSize = 0, i = 0;
        sizeCostsPerPartLattice_.fill(0);
        sizeCostsPerType_.fill(0);
        for_each_tuple(partialLattices_,
        [this, &totalPartLatticePtsSize, &i, &_sizeCosts](auto& partLatI) {
            partLatI.computeLatticeStructure(*stateSim_, i);
            totalPartLatticePtsSize += partLatI.lattice.size();
            get_costs_sizes<CostFuncsTypesNr>(partLatI, i, sizeCostsPerPartLattice_, sizeCostsPerType_);
            _sizeCosts +=  sizeCostsPerPartLattice_[i];
            i++;
        }
                      );
        return totalPartLatticePtsSize;
    }
private :
    void resizeCosts(const size_t& _optParamSize, const bool& _simulatingWithGrad) {
        for ( size_t i = 0; i < costs_.subSize(); ++i ) {
            costs_.sub(i).subResize(sizeCostsPerType_[i]);
        }

        if(_simulatingWithGrad) {
            for ( size_t i = 0; i < gradCosts_.subSize(); ++i ) {
                auto& gradCostsI = gradCosts_.sub(i);
                gradCostsI.subResize(sizeCostsPerType_[i]);
                for ( size_t j = 0; j < gradCostsI.subSize(); ++j ) {
                    gradCostsI.sub(j).subResize(_optParamSize);
                }
            }
            for ( size_t i = 0; i < gradCosts_.subSize(); ++i ) {
                auto& gradCostsI = gradCosts_.sub(i);
                for ( size_t j = 0; j < gradCostsI.subSize(); ++j ) {
                    new (gradCostsMap_[i].get()) Eigen::Map<Eigen::Matrix<TNumType,-1,-1, Eigen::RowMajor>, MapAlignment>(gradCostsI.memStartRef(), sizeCostsPerType_[i], _optParamSize );
                }
            }
        }
    }
private :
    template<size_t FuncNr = 0> void evaluateCosts(TNumType*& _cfPtr, TNumType*& _cfGradPtr, const size_t& _optParamSize) {
        for_each_tuple ( partialLattices_,
        [this, &_cfPtr, &_optParamSize, &_cfGradPtr](auto& partLatI) {
            size_t latticeISize = partLatI.lattice.size();
            partLatI.precompute(*stateSim_);
            if ( simulatingWithGrad ) {
                for ( size_t i = 0; i < latticeISize; ++i ) {
                    const auto& latticeI = partLatI.lattice[i];
                    partLatI.template evaluateWithGrad<FuncNr>( latticeI.statePtr->state(), i, latticeI.statePtr->stateGrad(), *stateSim_, _cfPtr, _cfGradPtr, _optParamSize);
                }
            } else {
                for ( size_t i = 0; i < latticeISize; ++i ) {
                    const auto& latticeI = partLatI.lattice[i];
                    partLatI.template evaluate<FuncNr>(*latticeI.statePtr, i, *stateSim_, _cfPtr);
                }
            }
        }
                       );
    }

private :
    void getMinArcLatCacheIdx (typename std::vector<LatticePointType>::iterator*& _itMin, size_t& _latVecIdx) {
        TNumType _arcMin = FLT_MAX;
        _itMin = nullptr;
        for_each_tuple( partialLattices_,
        [&_arcMin, &_itMin, &_latVecIdx](auto& latI) {
            auto& vecIter = latI.latticeVecCacheIter_;
            if ( vecIter!=latI.latticeVecIterEnd_ ) {
                if(vecIter->arc < _arcMin) {
                    _arcMin = vecIter->arc;
                    _itMin = &vecIter;
                    _latVecIdx = std::distance(latI.lattice.begin(), vecIter);
                }
            }
        }
                      );
    }

private  :
    template< bool useStateSimNm = TUseStateNm, typename std::enable_if< ( useStateSimNm ) >::type* = nullptr>
    void setXNmDot(const TNumType& _arcNow) {
        stateSim_->setXNmDot ( _arcNow, PfEaG::NEAR_LAST );
    }
private  :
    template< bool useStateSimNm = TUseStateNm, typename std::enable_if< ( !useStateSimNm ) >::type* = nullptr>
    void setXNmDot(const TNumType& _arcNow) {  }

    size_t bindFromPartLatticesToSimLattice(const bool& _saveLatticeStates = false) {
        int simLatticeIdx_ = -1;
        bool first = true;
        static typename std::vector<LatticePointType>::iterator* itMinPtr;
        size_t latVecIdx;
        getMinArcLatCacheIdx (itMinPtr, latVecIdx);
        for ( size_t i = 0; i < simulationLattice_.size(); ++i ) {
            simulationLattice_[i].arc = -1;
        }
        while ( itMinPtr != nullptr ) {
            const TNumType& arcNow = (*itMinPtr)->arc;
            if      ( first                                           ) {
                (this->*advanceFunc)(arcNow);
                first = false;
            }
            else if ( simulationLattice_[simLatticeIdx_].arc < arcNow ) {
                (this->*advanceFunc)(arcNow);
            }
            ++simLatticeIdx_;
            auto& simLatticeI = simulationLattice_[simLatticeIdx_];
            simLatticeI.arc = arcNow;
            simLatticeI.latticeIdx = (*(*itMinPtr)).latticeIdx;
            (*(*itMinPtr)).statePtr = simLatticeI.statePtr;
            if ( (sizeCostsPerPartLattice_[simLatticeI.latticeIdx] > 0 ) || (_saveLatticeStates)) {
                stateSim_->setXCf (arcNow, PfEaG::NEAR_LAST);
                if ( simulatingWithGrad ) {
                    stateSim_->setGradXCf( arcNow, PfEaG::NEAR_LAST );
                    stateSim_->setXCfDot ( arcNow, PfEaG::NEAR_LAST );
                    setXNmDot( arcNow );
                    copyReqDataFromSimToSimLatticeI(*stateSim_, simLatticeI);
                    correctStateGradFunc[simLatticeI.latticeIdx](*stateSim_, arcNow, latVecIdx, simLatticeI.statePtr->stateGrad() );
                } else {
                    copyReqDataFromSimToSimLatticeI(*stateSim_, simLatticeI);
                }
            }
            (*itMinPtr)++;
            getMinArcLatCacheIdx (itMinPtr, latVecIdx);
        }
        for ( size_t i = ++simLatticeIdx_; i < simulationLattice_.size(); ++i ) {
            simulationLattice_[i].arc = -1;
        }
        return simLatticeIdx_;
    }
private  :
    void resizeSimLattice(const size_t& _totalPartLatticePtsSize) {
        bool forceResize = false;
        bool resize = false;
        if ( ( _totalPartLatticePtsSize > simulationLattice_.size() ) ) {
            forceResize = true;
            resize = true;
        } else if (simulationLattice_.size() > 0) {
            if(stateSim_->state().data().size() != simulationLattice_[0].statePtr->data().size()) {
                forceResize = true;
            }
        } else {
            forceResize = true;
        }
        if(resize)      {
            simulationLattice_.resize(2*_totalPartLatticePtsSize);    ///@warn
        }
        if(forceResize) {
            for(auto& simLatticeI : simulationLattice_) {
                copyStructureFromSimToSimLatticeI(*stateSim_, simLatticeI);
            }
        }
    }


private :
    template< typename TSimType2, typename TLatticePointType, bool canComputeStateGrad = CanComputeStateGrad,
              typename std::enable_if< (!canComputeStateGrad ) >::type* = nullptr >
    void copyReqDataFromSimToSimLatticeI(const TSimType2& _sim, TLatticePointType& _latticePtI) {
        copyDataFromSimToSimLatticeI(_sim, _latticePtI);
    }
private :
    template< typename TSimType2, typename TLatticePointType, bool canComputeStateGrad = CanComputeStateGrad,
              typename std::enable_if< ( canComputeStateGrad ) >::type* = nullptr >
    void copyReqDataFromSimToSimLatticeI(const TSimType2& _sim, TLatticePointType& _latticePtI) {
        copyDataFromSimToSimLatticeI(_sim, _latticePtI);
        if ( simulatingWithGrad ) {
            copyGradDataFromSimToSimLatticeI(_sim, _latticePtI);
        }
    }

private  :
    template<  typename TSimType2, typename TLatticePointType, bool useStateSimNm = TUseStateNm, typename std::enable_if< ( useStateSimNm ) >::type* = nullptr>
    static void copyDataNmFromSimToSimLatticeI(const TSimType2& _sim, TLatticePointType& _latticePtI) {
        auto& latticeState = *_latticePtI.statePtr;
        const auto& simState = _sim.state();
        latticeState.stateNm    ().data() = simState.stateNm    ().data();
    }
private  :
    template< typename TSimType2, typename TLatticePointType,  bool useStateSimNm = TUseStateNm, typename std::enable_if< ( !useStateSimNm ) >::type* = nullptr>
    static void copyDataNmFromSimToSimLatticeI(const TSimType2& _sim, TLatticePointType& _latticePtI) {  }
private  :
    template<  typename TSimType2, typename TLatticePointType, bool useStateSimNm = TUseStateNm, typename std::enable_if< ( useStateSimNm ) >::type* = nullptr>
    static void copyGradDataNmFromSimToSimLatticeI(const TSimType2& _sim, TLatticePointType& _latticePtI) {
        auto& latticeState = *_latticePtI.statePtr;
        const auto& simState = _sim.state();
        latticeState.stateGradNm().data() = simState.stateGradNm().data();
    }
private  :
    template<  typename TSimType2, typename TLatticePointType, bool useStateSimNm = TUseStateNm, typename std::enable_if< ( !useStateSimNm ) >::type* = nullptr>
    static void copyGradDataNmFromSimToSimLatticeI(const TSimType2& _sim, TLatticePointType& _latticePtI) {  }


private :
    template< typename TSimType2, typename TLatticePointType>
    static void copyDataFromSimToSimLatticeI(const TSimType2& _sim, TLatticePointType& _latticePtI) {
        auto& latticeState = *_latticePtI.statePtr;
        const auto& simState = _sim.state();
        latticeState.stateCf    ().data() = simState.stateCf    ().data();
        copyDataNmFromSimToSimLatticeI(_sim, _latticePtI);
    }
private :
    template< typename TSimType2, typename TLatticePointType, bool canComputeStateGrad = CanComputeStateGrad,
              typename std::enable_if< ( canComputeStateGrad ) >::type* = nullptr >
    static void copyGradDataFromSimToSimLatticeI(const TSimType2& _sim, TLatticePointType& _latticePtI) {
        auto& latticeState = *_latticePtI.statePtr;
        const auto& simState = _sim.state();
        latticeState.stateGradCf().data() = simState.stateGradCf().data();
        copyGradDataNmFromSimToSimLatticeI(_sim, _latticePtI);
    }
private :
    template< typename TSimType2, typename TLatticePointType, bool canComputeStateGrad = CanComputeStateGrad,
              typename std::enable_if< (!canComputeStateGrad ) >::type* = nullptr >
    static void copyStructureFromSimToSimLatticeI(const TSimType2& _sim, TLatticePointType& _latticePtI) {
        auto& latticeState = *_latticePtI.statePtr;
        const auto& simState = _sim.state();
        latticeState.stateCf    () = simState.stateCf    ();
        latticeState.stateNm    () = simState.stateNm    ();
    }
private :
    template< typename TSimType2, typename TLatticePointType, bool canComputeStateGrad = CanComputeStateGrad,
              typename std::enable_if< ( canComputeStateGrad ) >::type* = nullptr >
    static void copyStructureFromSimToSimLatticeI(const TSimType2& _sim, TLatticePointType& _latticePtI) {
        auto& latticeState = *_latticePtI.statePtr;
        const auto& simState = _sim.state();
        latticeState.stateCf    () = simState.stateCf    ();
        latticeState.stateNm    () = simState.stateNm    ();
        latticeState.stateGradNm() = simState.stateGradNm();
        latticeState.stateGradCf() = simState.stateGradCf();
        latticeState.stateGrad().bindMat();
    }
    ///@brief State simulator object.
protected:
    StateSimSPtr   stateSim_;
    ///@brief Lattice requesting each simulated trajectory state.
private  :
    std::vector< LatticePointType >    simulationLattice_;
private  :
    size_t                                           simulationLatticeActiveSize_;
    ///@brief Vector containing the ordered sequence of arc parametrizations for each of the used lattices.
public   :
    std::tuple<TLatticeTypes<TNumType, TSimType>...> partialLattices_;
public   :
    std::array<std::function<void( const TSimType&,
                                   const TNumType&,
                                   const size_t&,
                                   typename StateMapBaseTraits<typename StateType::StateMapBaseType>::StateGradType& )>, sizeof...(TLatticeTypes)> correctStateGradFunc;

public   :
    StateMapArray<TNumType, StateMapVector<TNumType, TNumType>                            , CostFuncsTypesNr>  costs_;
public   :
    StateMapArray<TNumType, StateMapVector<TNumType, StateMapVector<TNumType, TNumType>>  , CostFuncsTypesNr>  gradCosts_;
public   :
    std::array<std::shared_ptr<Eigen::Map<Eigen::Matrix<TNumType,-1,-1, Eigen::RowMajor>, MapAlignment>>, CostFuncsTypesNr>  gradCostsMap_;
private  :
    std::array<size_t, sizeof...(TLatticeTypes)> sizeCosts_;
private  :
    std::array<size_t, sizeof...(TLatticeTypes)> sizeCostsPerPartLattice_;
private  :
    std::array<size_t, CostFuncsTypesNr        > sizeCostsPerType_;
};







}

#endif // TRAJECTORY_SIMULATOR_HPP

