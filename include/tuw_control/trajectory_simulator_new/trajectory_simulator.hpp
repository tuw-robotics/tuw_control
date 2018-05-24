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

#ifndef TRAJECTORY_SIMULATOR_HPP
#define TRAJECTORY_SIMULATOR_HPP

#include <tuw_control/state_sim/state_sim.hpp>
#include <tuw_control/param_func_new/param_func_dist.hpp>
#include <tuw_control/costs_evaluator/costs_evaluator.hpp>

#include <functional>
#include <memory>

namespace tuw
{
// class TrajectorySimulator;
// using TrajectorySimulatorSPtr      = std::shared_ptr<TrajectorySimulator>;
// using TrajectorySimulatorConstSPtr = std::shared_ptr<TrajectorySimulator const>;
// using TrajectorySimulatorUPtr      = std::unique_ptr<TrajectorySimulator>;
// using TrajectorySimulatorConstUPtr = std::unique_ptr<TrajectorySimulator const>;

template <typename TNumType, typename TSimType>
class TrajectorySimulator
{
public:
  using StateSimSPtr = std::shared_ptr<TSimType>;

public:
  using StateType = typename TSimType::StateType;

public:
  using StateSPtr = std::shared_ptr<StateType>;

public:
  using StateForSimType = typename TSimType::StateForSimType;

public:
  static constexpr const bool CanComputeStateGrad =
      !std::is_same<EmptyGradType, typename StateMapBaseTraits<StateType>::StateWithGradNmType>::value;

  // enums
  ///@brief Fundamental lattice types.
public:
  enum class BaseSimLatticeType : signed int
  {
    ARC_BG_BK = -3,          ///> begin and end of the simulation lattice
    LATTICE_ARC_EQ_DT = -2,  ///> equal arc lattice
    LATTICE_ARC_EQ_DS = -1,  ///> equal distance lattice
    LATTICE_ENUM_SIZE = 3    ///> size of the enumerated values
  };
  static constexpr const std::size_t extArcLatIdxBegin =
      asInt(BaseSimLatticeType::LATTICE_ENUM_SIZE) + asInt(BaseSimLatticeType::LATTICE_ARC_EQ_DS);

  ///@brief Structure containing an evaluation arc and a state pointer.
public:
  struct LatticePoint
  {
    LatticePoint() : arc(-1), statePtr(nullptr)
    {
    }
    LatticePoint(TNumType _arc) : arc(_arc), statePtr(nullptr)
    {
    }
    LatticePoint(TNumType _arc, StateSPtr& _statePtr) : arc(_arc), statePtr(_statePtr)
    {
    }
    LatticePoint(bool _makePtrShared) : arc(-1), statePtr(std::shared_ptr<StateType>(new StateType))
    {
    }
    virtual ~LatticePoint()
    {
    }
    TNumType arc;
    StateSPtr statePtr;
  };
  using LatticeVec = std::vector<LatticePoint>;
  using LatticeVecSPtr = std::shared_ptr<std::vector<LatticePoint>>;
  using LatticeVecSPtrVec = std::vector<std::shared_ptr<std::vector<LatticePoint>>>;

  ///@brief Structure containing the lattice type afferent to a @param LatticePoint.
public:
  struct LatticePointType : public LatticePoint
  {
    LatticePointType() : LatticePoint(true), latticeType(-5)
    {
    }
    LatticePointType(TNumType _arc, int _latticeType) : LatticePoint(_arc), latticeType(_latticeType)
    {
    }
    LatticePointType(TNumType _arc, int _latticeType, StateSPtr& _statePtr)
      : LatticePoint(_arc, _statePtr), latticeType(_latticeType)
    {
    }
    int latticeType;
  };

public:
  using CostsEvaluatorClass = CostsEvaluatorBase<TrajectorySimulator::LatticeVec>;

  // special class member functions
public:
  TrajectorySimulator(StateSimSPtr& _stateSim)
    : stateSim_(_stateSim)
    , partLattices_(asInt(BaseSimLatticeType::LATTICE_ENUM_SIZE))
    , partLatIdxCache_(asInt(BaseSimLatticeType::LATTICE_ENUM_SIZE))
    , canComputeDistLattice_(stateSim_->paramFuncsDist() != nullptr)
    , dtBase_(-1)
    , dsBase_(-1)
    , scaleDt_(false)
    , scaleDs_(false)
  {
    for (size_t i = 0; i < partLattices_.size(); ++i)
    {
      partLattices_[i] = std::shared_ptr<LatticeVec>(new LatticeVec);
    }
    *(partLattices_[lattTypeIdx(asInt(BaseSimLatticeType::ARC_BG_BK))]) = { LatticePoint(), LatticePoint() };
    simulationLatticeActiveSize_ = 0;
  }

public:
  TrajectorySimulator(StateSimSPtr& _stateSim, std::unique_ptr<CostsEvaluatorClass> _costsEvaluator)
    : TrajectorySimulator(_stateSim)
  {
    costsEvaluator_ = std::move(_costsEvaluator);
    costsEvaluator_->init(partLattices_);
  }

public:
  virtual ~TrajectorySimulator() = default;

public:
  TrajectorySimulator(const TrajectorySimulator&) = default;

public:
  TrajectorySimulator& operator=(const TrajectorySimulator&) = default;

public:
  TrajectorySimulator(TrajectorySimulator&&) = default;

public:
  TrajectorySimulator& operator=(TrajectorySimulator&&) = default;

public:
  void setBoolDtScale(const bool& _doScale)
  {
    scaleDt_ = _doScale;
  }

public:
  void setBoolDsScale(const bool& _doScale)
  {
    scaleDs_ = _doScale;
  }

  ///@brief Reference to arc parametrization interval used for the equal arc-length lattice.
public:
  TNumType& dtBase()
  {
    return dtBase_;
  }
  ///@brief Const reference to arc parametrization interval used for the equal arc-length lattice.
public:
  const TNumType& dt() const
  {
    return dt_;
  }
  ///@brief Reference to arc parametrization interval used for the equal distance lattice.
public:
  TNumType& dsBase()
  {
    return dsBase_;
  }
  ///@brief Const reference to arc parametrization interval used for the equal distance lattice.
public:
  const TNumType& ds() const
  {
    return ds_;
  }
  ///@brief Reference of the state simulator object.
public:
  StateSimSPtr& stateSim()
  {
    return stateSim_;
  }
  ///@brief Const reference of the state simulator object.
public:
  const StateSimSPtr& stateSim() const
  {
    return stateSim_;
  }
  ///@todo documentation
public:
  void setUserDefLattice(const std::vector<std::vector<TNumType*>>& _userDefLattices)
  {
    userDefPartLattices_ = _userDefLattices;
    partLattices_.resize(asInt(BaseSimLatticeType::LATTICE_ENUM_SIZE) + _userDefLattices.size());
    for (size_t i = asInt(BaseSimLatticeType::LATTICE_ENUM_SIZE); i < partLattices_.size(); ++i)
    {
      partLattices_[i] = std::shared_ptr<LatticeVec>(new LatticeVec);
    }
    partLatIdxCache_.resize(partLattices_.size());
    for (size_t i = 0; i < _userDefLattices.size(); ++i)
    {
      partLattices_[lattTypeIdx(i)]->resize(_userDefLattices[i].size());
    }
    updateUserDefLattice();
    if (costsEvaluator_)
    {
      costsEvaluator_->init(partLattices_);
    }
  }

public:
  void updateUserDefLattice()
  {
    partLatIdxCache_.resize(partLattices_.size());
    for (size_t i = 0; i < userDefPartLattices_.size(); ++i)
    {
      for (size_t j = 0; j < userDefPartLattices_[i].size(); ++j)
      {
        partLattices_[lattTypeIdx(i)]->at(j).arc = *userDefPartLattices_[i][j];
      }
    }
  }

  /** @brief Simulates (discrete numerical evaluation) an entire trajectory according to the specified intervals and
   * lattices.
   *  @param _lastValidArc Specifies the last arc at which the previous simulation was still valid. Setting it non-zero
   * is beneficial if already computed (in memory) @ref trajStates()
   *  are guaranteed to not have changed. Thus, it allows the function to simulate only portions of the entire arc
   * parametrization domain.
   */

  bool simulatingWithGrad;

public:
  void simulateTrajectory(TNumType _lastValidArc = 0)
  {
    advanceFunc = &TrajectorySimulator<TNumType, TSimType>::advanceFuncSim;
    simulatingWithGrad = false;
    simulateTrajectoryImpl(_lastValidArc);
  }

public:
  template <bool canComputeStateGrad = CanComputeStateGrad,
            typename std::enable_if<(canComputeStateGrad)>::type* = nullptr>
  void simulateTrajectoryWithGrad(TNumType _lastValidArc = 0)
  {
    advanceFunc = &TrajectorySimulator<TNumType, TSimType>::advanceFuncSimGrad;
    simulatingWithGrad = true;
    simulateTrajectoryImpl(_lastValidArc);
  }

private:
  void simulateTrajectoryImpl(TNumType _lastValidArc)
  {
    firstTime_ = true;
    computeScaleDtDs();
    updateUserDefLattice();

    // resize equal dt lattice
    const TNumType arcParamMax = stateSim_->paramFuncs()->funcsArcEnd();
    size_t simLatticeSize = std::max(0, (int)(ceil(arcParamMax / dt()) + 1));
    partLattices_[lattTypeIdx(asInt(BaseSimLatticeType::LATTICE_ARC_EQ_DT))]->resize(simLatticeSize,
                                                                                     LatticePoint(FLT_MAX));

    // set the dist-extended sim lattice points on the last lattice entries
    if (canComputeDistLattice_ && (ds() > 0))
    {
      static std::vector<TNumType> dsLattice;
      auto& partLatticeDs = partLattices_[lattTypeIdx(asInt(BaseSimLatticeType::LATTICE_ARC_EQ_DS))];
      if (scaleDs_ || scaleDt_)
      {
        stateSim_->paramFuncsDist()->computeS2TLattice(0, ds(), dsLattice);
        partLatticeDs->resize(dsLattice.size());
        for (size_t i = 0; i < dsLattice.size(); ++i)
        {
          partLatticeDs->at(i).arc = dsLattice[i];
        }
      }
      else
      {
        stateSim_->paramFuncsDist()->computeS2TLattice(_lastValidArc, ds(), dsLattice);
        const TNumType& firstDsLattice = dsLattice[0];
        size_t idxFirstInvalidDs = std::max(0, (int)partLatticeDs->size() - 2);
        for (size_t i = 1; i < partLatticeDs->size(); ++i)
        {
          if (partLatticeDs->at(i).arc > firstDsLattice + 1e-3)
          {
            idxFirstInvalidDs = --i;
            break;
          }
        }
        partLatticeDs->resize(idxFirstInvalidDs + dsLattice.size());
        for (size_t i = 0; i < dsLattice.size(); ++i)
        {
          partLatticeDs->at(i + idxFirstInvalidDs).arc = dsLattice[i];
        }
      }
    }
    else
    {
      partLattices_[lattTypeIdx(asInt(BaseSimLatticeType::LATTICE_ARC_EQ_DS))]->clear();
    }

    size_t firstLaticeInvalidIdx = 0;
    if (initSimLatticeState0(_lastValidArc, firstLaticeInvalidIdx))
    {
      populateTrajSimPartLattice(firstLaticeInvalidIdx);
      if (costsEvaluator_)
      {
        costsEvaluator_->evaluateAllCosts();
      }
    }
  }

  ///@brief Performs simulation and populates simulation and partial lattice when only equal dt lattice is enabled.
protected:
  void populatePartSimLatticesDtOnly(const size_t& _firstLaticeInvalidIdx, TNumType _arcParamMax)
  {
    setBeginEndArcsToLattices(0, _arcParamMax);

    size_t itEnd = (int)_arcParamMax / dt() + 1;
    // push back DT lattice points
    for (size_t aPLIdx = _firstLaticeInvalidIdx + 1; aPLIdx < itEnd; ++aPLIdx)
    {
      simAppendToSimPartLat(aPLIdx * dt(), asInt(BaseSimLatticeType::LATTICE_ARC_EQ_DT), aPLIdx);
    }

    // push back final lattice point
    setEndStateToLattices(_arcParamMax);
  }
  ///@brief Performs simulation and populates simulation and partial lattices in the general case of various enabled
  /// lattices.
protected:
  void populatePartSimLatticesGeneral(size_t _firstLaticeInvalidIdx, TNumType _arcParamMax, TNumType _minArcLatticeVal)
  {
    setBeginEndArcsToLattices(0, _arcParamMax);
    initExtLatticeCache(_minArcLatticeVal);

    size_t arcParamLatticeIdx = std::max(0, (int)(_minArcLatticeVal / dt()));
    size_t minArcLatCacheIdx =
        getMinArcLatCacheIdx();  // find lattice type index of next smallest required simulation lattice value
    while ((_minArcLatticeVal = partLattices_[minArcLatCacheIdx]->at(partLatIdxCache_[minArcLatCacheIdx]).arc) <
           _arcParamMax)
    {
      const size_t deltaArcParamLattice = std::max(0, (int)(_minArcLatticeVal / dt()) - (int)arcParamLatticeIdx);
      const size_t simLatticeInjectEnd = ++_firstLaticeInvalidIdx + deltaArcParamLattice;

      for (; _firstLaticeInvalidIdx < simLatticeInjectEnd; ++_firstLaticeInvalidIdx)
      {  // push_back the equal time lattice points before the extended one first
        simAppendToSimPartLat(++arcParamLatticeIdx * dt(), asInt(BaseSimLatticeType::LATTICE_ARC_EQ_DT),
                              arcParamLatticeIdx);
      }
      simAppendToSimPartLat(_minArcLatticeVal, (int)minArcLatCacheIdx - asInt(BaseSimLatticeType::LATTICE_ENUM_SIZE),
                            partLatIdxCache_[minArcLatCacheIdx]);  // push back extended lattice point

      int& idxMinLatticePt = partLatIdxCache_[minArcLatCacheIdx];
      if (idxMinLatticePt + 1 < (int)partLattices_[minArcLatCacheIdx]->size())
      {
        ++idxMinLatticePt;
      }
      minArcLatCacheIdx = getMinArcLatCacheIdx();  // update the minimum extended lattice point
    }
    const TNumType simLatticeInjectEnd = _arcParamMax / dt();
    ++arcParamLatticeIdx;
    for (; arcParamLatticeIdx < simLatticeInjectEnd; ++arcParamLatticeIdx)
    {
      simAppendToSimPartLat(arcParamLatticeIdx * dt(), asInt(BaseSimLatticeType::LATTICE_ARC_EQ_DT),
                            arcParamLatticeIdx);
    }

    setEndStateToLattices(_arcParamMax);
  }

private:
  size_t getMinArcLatCacheIdx() const
  {
    size_t idxMin = 0;
    TNumType minArc = FLT_MAX;
    for (size_t iPart = extArcLatIdxBegin; iPart < partLattices_.size(); ++iPart)
    {
      if (partLattices_[iPart]->empty())
      {
        continue;
      }
      const TNumType& arcI = partLattices_[iPart]->at(partLatIdxCache_[iPart]).arc;
      if (minArc > arcI)
      {
        minArc = arcI;
        idxMin = iPart;
      }
    }
    return idxMin;
  }

  using AdvanceFunction = void (TrajectorySimulator<TNumType, TSimType>::*)(const TNumType&);
  AdvanceFunction advanceFunc;

private:
  void advanceFuncSim(const TNumType& _arcNow)
  {
    stateSim_->advance(_arcNow);
  }

private:
  template <bool canComputeStateGrad = CanComputeStateGrad,
            typename std::enable_if<(canComputeStateGrad)>::type* = nullptr>
  void advanceFuncSimGrad(const TNumType& _arcNow)
  {
    stateSim_->advanceWithGrad(_arcNow);
  }

private:
  auto& dtIdpNm(const size_t _i, const TNumType& _arcNow, const int& _latticePtType) const
  {
    static Eigen::Matrix<TNumType, 1, -1> ans;
    ans.resize(stateSim_->state().stateGradNm().sub(0).data().size());
    ans.setZero();
    if (_arcNow != 0)
    {
      ans(ans.cols() - 1) = (TNumType)1. / (TNumType)(partLattices_[lattTypeIdx(-2)]->size() - 1);
    }
    return ans;
  }

private:
  auto& dtIdpCf(const size_t _i, const TNumType& _arcNow, const int& _latticePtType) const
  {
    static Eigen::Matrix<TNumType, 1, -1> ans;
    ans.resize(stateSim_->state().stateGradNm().sub(0).data().size());
    ans.setZero();
    if (_arcNow == stateSim_->paramFuncs()->funcsArcEnd())
    {
      ans(ans.cols() - 1) = (TNumType)_i / (TNumType)(partLattices_[lattTypeIdx(-2)]->size() - 1);
    }
    return ans;
  }

private:
  template <typename TSimState, typename TState, bool canComputeStateGrad = CanComputeStateGrad,
            typename std::enable_if<(!canComputeStateGrad)>::type* = nullptr>
  static void copyDataFromSimStateToState(const TSimState& _simState, TState& _state)
  {
    _state.stateCf().data() = _simState.stateCf().data();
    _state.stateNm().data() = _simState.stateNm().data();
  }

private:
  template <typename TSimState, typename TState, bool canComputeStateGrad = CanComputeStateGrad,
            typename std::enable_if<(canComputeStateGrad)>::type* = nullptr>
  static void copyDataFromSimStateToState(const TSimState& _simState, TState& _state)
  {
    _state.stateCf().data() = _simState.stateCf().data();
    _state.stateNm().data() = _simState.stateNm().data();
    _state.stateGradNm().data() = _simState.stateGradNm().data();
    _state.stateGradCf().data() = _simState.stateGradCf().data();
  }

private:
  template <typename TSimState, typename TState, bool canComputeStateGrad = CanComputeStateGrad,
            typename std::enable_if<(!canComputeStateGrad)>::type* = nullptr>
  static void copyStructureFromSimStateToState(const TSimState& _simState, TState& _state)
  {
    _state.stateCf() = _simState.stateCf();
    _state.stateNm() = _simState.stateNm();
  }

private:
  template <typename TSimState, typename TState, bool canComputeStateGrad = CanComputeStateGrad,
            typename std::enable_if<(canComputeStateGrad)>::type* = nullptr>
  static void copyStructureFromSimStateToState(const TSimState& _simState, TState& _state)
  {
    _state.stateCf() = _simState.stateCf();
    _state.stateNm() = _simState.stateNm();
    _state.stateGradNm() = _simState.stateGradNm();
    _state.stateGradCf() = _simState.stateGradCf();
  }

private:
  template <typename TSimState, typename TState, bool canComputeStateGrad = CanComputeStateGrad,
            typename std::enable_if<(!canComputeStateGrad)>::type* = nullptr>
  static void copyDataFromStateToSimState(const TState& _state, TSimState& _simState)
  {
    _simState.stateCf().data() = _state.stateCf().data();
    _simState.stateNm().data() = _state.stateNm().data();
  }

private:
  template <typename TSimState, typename TState, bool canComputeStateGrad = CanComputeStateGrad,
            typename std::enable_if<(canComputeStateGrad)>::type* = nullptr>
  static void copyDataFromStateToSimState(const TState& _state, TSimState& _simState)
  {
    _simState.stateCf().data() = _state.stateCf().data();
    _simState.stateNm().data() = _state.stateNm().data();
    _simState.stateGradNm().data() = _state.stateGradNm().data();
    _simState.stateGradCf().data() = _state.stateGradCf().data();
  }

  ///@brief Performs a simulation step (if @param _arcNow is different than last simulated arc) and appends the new arc
  /// and state pointer to the afferent partial lattice point.
protected:
  void simAppendToSimPartLat(const TNumType& _arcNow, const int& _latticePtType, const std::size_t& _minArcLatCacheIdx)
  {
    if ((stateSim_->stateArc() < _arcNow) || firstTime_)
    {
      (this->*advanceFunc)(_arcNow);
      auto& simLatticeI = simulationLattice_[simAppendIdx_];

      auto& stateSimState = stateSim_->state();
      simLatticeI.arc = _arcNow;

      if (simulatingWithGrad)
      {
        stateSim_->setStateCfWithGrad(_arcNow, PfEaG::NEAR_LAST);
        copyDataFromSimStateToState(stateSimState, *simLatticeI.statePtr);
        stateSim_->stateNmDot(stateSim_->state().stateNm(), stateSim_->state().stateCf(), _arcNow, PfEaG::NEAR_LAST);
        stateSim_->stateCfDot();
        const auto& dtIdpVal = dtIdpCf(simAppendIdx_, _arcNow, _latticePtType);
        static Eigen::Matrix<TNumType, -1, -1> fCfdtIdpVal;
        static Eigen::Matrix<TNumType, -1, -1> fNmdtIdpVal;

        fCfdtIdpVal = (stateSim_->stateCfDotCache_.data() * dtIdpVal).transpose();
        fNmdtIdpVal = (stateSim_->stateNmDotCache_.data() * dtIdpVal).transpose();

        auto& stateGradCf = simLatticeI.statePtr->stateGradCf();
        auto& stateGradNm = simLatticeI.statePtr->stateGradNm();
        stateGradCf.data() += Eigen::Map<Eigen::Matrix<TNumType, -1, 1>>(fCfdtIdpVal.data(), fCfdtIdpVal.size());
        stateGradNm.data() += Eigen::Map<Eigen::Matrix<TNumType, -1, 1>>(fNmdtIdpVal.data(), fNmdtIdpVal.size());
      }
      else
      {
        stateSim_->setStateCf(_arcNow, PfEaG::NEAR_LAST);
        copyDataFromSimStateToState(stateSimState, *simLatticeI.statePtr);
      }

      simAppendIdx_++;
      firstTime_ = false;
    }
    appendToPartLat(_arcNow, _latticePtType, _minArcLatCacheIdx);
  }

  //     protected: void   simAppendToSimPartLat          ( const TNumType& _arcNow, const int& _latticePtType, const
  //     std::size_t& _minArcLatCacheIdx) {
  //
  //
  // 	auto& dtIdpVal = dtIdpCf(simAppendIdx_, _arcNow, _latticePtType);
  //
  // 	Eigen::Matrix<TNumType,-1,-1> fNmdtIdpVal;
  // 	static Eigen::Matrix<TNumType,-1,-1> fNmdtIdpValOld;
  // 	if(_arcNow == 0) {
  // 	    fNmdtIdpVal = (stateSim_->stateNmDotCache_.data() * dtIdpVal).transpose();
  // 	    fNmdtIdpValOld = fNmdtIdpVal;
  // 	}
  //
  // 	if( ( stateSim_->stateArc() < _arcNow ) || firstTime_ ) {
  // 	    (this->*advanceFunc)(_arcNow);
  // 	    auto& simLatticeI = simulationLattice_[simAppendIdx_];
  //
  // 	    auto& stateSimState = stateSim_->state();
  // 	    simLatticeI.arc = _arcNow;
  // 	    copyDataFromSimStateToState(stateSimState, *simLatticeI.statePtr);
  //
  //
  // 	    if(simulatingWithGrad) {
  //
  // 		stateSim_->stateCfDot();
  // 		Eigen::Matrix<TNumType,-1,-1> fCfdtIdpVal = (stateSim_->stateCfDotCache_.data() * dtIdpVal).transpose();
  // 		if(_arcNow>0){
  // 		    fNmdtIdpVal = ((stateSim_->state().stateNm().data() -
  // simulationLattice_[simAppendIdx_-1].statePtr->stateNm().data())/(_arcNow-simulationLattice_[simAppendIdx_-1].arc) *
  // dtIdpVal).transpose();
  // 		}
  //
  // 		auto& stateGradCf = simLatticeI.statePtr->stateGradCf();
  // 		auto& stateGradNm = simLatticeI.statePtr->stateGradNm();
  // 		stateGradCf.data() += Eigen::Map<Eigen::Matrix<TNumType,-1,1>>(fCfdtIdpVal.data(), fCfdtIdpVal.size());
  // 		stateSim_->state().stateGrad() =
  // 		stateGradNm.data() += Eigen::Map<Eigen::Matrix<TNumType,-1,1>>(fNmdtIdpVal.data(), fNmdtIdpVal.size());
  //
  // // 		auto& stateSimGradNm = stateSim_->state().stateGradNm();
  // // 		stateSimGradNm.data() += Eigen::Map<Eigen::Matrix<TNumType,-1,1>>(fNmdtIdpVal   .data(), fNmdtIdpVal
  // .size());
  // // 		stateSimGradNm.data() -= Eigen::Map<Eigen::Matrix<TNumType,-1,1>>(fNmdtIdpValOld.data(),
  // fNmdtIdpValOld.size());
  // // 		stateGradNm.data() = stateSimGradNm.data();
  // // 		std::cout<<Eigen::Map<Eigen::Matrix<TNumType,-1,1>>(fNmdtIdpVal   .data(), fNmdtIdpVal
  // .size())-Eigen::Map<Eigen::Matrix<TNumType,-1,1>>(fNmdtIdpValOld.data(), fNmdtIdpValOld.size())<<std::endl;
  //
  // // 		std::cout<<"_arcNow="<<_arcNow<<std::endl;
  // // 		std::cout<<"fNmdtIdpVal="<<std::endl<<fNmdtIdpVal<<std::endl;
  // // 		std::cout<<"fNmdtIdpValOld="<<std::endl<<fNmdtIdpValOld<<std::endl<<std::endl;
  //
  // 		fNmdtIdpValOld = fNmdtIdpVal;
  // 	    }
  //
  //
  //
  // 	    simAppendIdx_++;
  // 	    firstTime_ = false;
  // 	}
  // 	appendToPartLat( _arcNow, _latticePtType, _minArcLatCacheIdx );
  //     }

  ///@brief Appends the new arc and state pointer to the afferent partial lattice point.
protected:
  void appendToPartLat(const TNumType& _arcNow, const int& _latticePtType, const std::size_t& _minArcLatCacheIdx)
  {
    auto& partLatticeNow = partLattices_[lattTypeIdx(_latticePtType)]->at(_minArcLatCacheIdx);
    partLatticeNow.arc = _arcNow;
    partLatticeNow.statePtr = simulationLattice_[simAppendIdx_ - 1].statePtr;
  }
  ///@brief Binds reference to the initial simulated state (at @param _arcBegin) to all partial lattices.
protected:
  void setBeginStateToLattices(const TNumType& _arcBegin)
  {
    simAppendToSimPartLat(_arcBegin, asInt(BaseSimLatticeType::ARC_BG_BK), 0);
    for (size_t i = lattTypeIdx(asInt(BaseSimLatticeType::ARC_BG_BK)); i < partLattices_.size(); ++i)
    {
      if (!partLattices_[i]->empty())
      {
        partLattices_[i]->at(0).statePtr = simulationLattice_[simAppendIdx_ - 1].statePtr;
      }
    }
  }
  ///@brief Binds reference to the final simulated state (at @param _arcEnd) to all partial lattices.
protected:
  void setEndStateToLattices(const TNumType& _arcEnd)
  {
    simAppendToSimPartLat(
        _arcEnd, asInt(BaseSimLatticeType::ARC_BG_BK),
        std::min(partLattices_[lattTypeIdx(asInt(BaseSimLatticeType::ARC_BG_BK))]->size() - 1, (size_t)1));
    simulationLatticeActiveSize_ = simAppendIdx_;
    for (size_t i = lattTypeIdx(asInt(BaseSimLatticeType::ARC_BG_BK)); i < partLattices_.size(); ++i)
    {
      if (!partLattices_[i]->empty())
      {
        auto& partLatI0 = partLattices_[i]->back();
        partLatI0.statePtr = simulationLattice_[simAppendIdx_ - 1].statePtr;
      }
    }
  }
  ///@brief Sets begin and end arcs to all partial lattices on the first and last container positions.
protected:
  void setBeginEndArcsToLattices(const TNumType& _arcBegin, const TNumType& _arcEnd)
  {
    for (size_t i = lattTypeIdx(asInt(BaseSimLatticeType::ARC_BG_BK)); i < partLattices_.size(); ++i)
    {
      if (!partLattices_[i]->empty())
      {
        partLattices_[i]->at(0).arc = _arcBegin;
        partLattices_[i]->back().arc = _arcEnd;
      }
    }
  }

  static bool cmpLatticePt(const TrajectorySimulator::LatticePoint& a, const TrajectorySimulator::LatticePoint& b)
  {
    return a.arc < b.arc;
  }

  ///@brief Initializes the simulation lattice (truncation from the @param _firstLaticeInvalidIdx) and sets the state
  /// simulator inital state (at @param _lastValidArc).
protected:
  bool initSimLatticeState0(const TNumType& _lastValidArc, size_t& _firstLaticeInvalidIdx)
  {
    if ((simulationLattice_.size() > 0) && (_lastValidArc > 0))
    {
      _firstLaticeInvalidIdx = std::upper_bound(simulationLattice_.begin(), simulationLattice_.end(),
                                                LatticePoint(_lastValidArc), cmpLatticePt) -
                               simulationLattice_.begin();
      if (_firstLaticeInvalidIdx >= simulationLattice_.size())
      {
        return false;
      }
    }
    if (_firstLaticeInvalidIdx == 0)
    {
      stateSim_->toState0();
    }
    else if (_firstLaticeInvalidIdx <= simulationLattice_.size())
    {
      auto& lastValidSimState = simulationLattice_[_firstLaticeInvalidIdx - 1];
      stateSim_->toState(*lastValidSimState.statePtr, lastValidSimState.arc);
    }
    return true;
  }
  ///@brief Initializes the cached partial lattices index at the highest arc lower than @param _minArcLatticeVal.
protected:
  void initExtLatticeCache(const TNumType& _minArcLatticeVal)
  {
    partLatIdxCache_.assign(partLattices_.size(), 0);
    for (size_t iPart = extArcLatIdxBegin; iPart < partLattices_.size(); ++iPart)
    {
      if (partLattices_[iPart]->empty())
      {
        partLatIdxCache_[iPart] = -1;
        continue;
      }
      for (; partLatIdxCache_[iPart] < (int)partLattices_[iPart]->size(); ++partLatIdxCache_[iPart])
      {
        if (partLattices_[iPart]->at(partLatIdxCache_[iPart]).arc > _minArcLatticeVal)
        {
          ++partLatIdxCache_[iPart];
          break;
        }
      }
      partLatIdxCache_[iPart] = std::max(0, (int)partLatIdxCache_[iPart] - 1);
    }
  }
  ///@brief Main function that performs resizing, reserving and calls the proper population function.
protected:
  void populateTrajSimPartLattice(const size_t& _firstLaticeInvalidIdx)
  {
    const TNumType arcParamMax = stateSim_->paramFuncs()->funcsArcEnd();
    size_t simLatticeSize = std::max(0, (int)(ceil(arcParamMax / dt()) + 2));

    partLattices_[lattTypeIdx(asInt(BaseSimLatticeType::ARC_BG_BK))]->resize(2);
    // reserve maximum computable lattice points
    for (size_t i = extArcLatIdxBegin; i < partLattices_.size(); ++i)
    {
      simLatticeSize += partLattices_[i]->size();
    }
    simLatticeSize = std::max(simLatticeSize, (size_t)0);
    bool forceResize = false;
    bool resize = false;
    if ((simLatticeSize > simulationLattice_.size()))
    {
      forceResize = true;
      resize = true;
    }
    else if (simulationLattice_.size() > 0)
    {
      if (stateSim_->state0().data().size() != simulationLattice_[0].statePtr->data().size())
      {
        forceResize = true;
      }
    }
    else
    {
      forceResize = true;
    }
    if (resize)
    {
      simulationLattice_.resize(simLatticeSize);
    }
    if (forceResize)
    {
      for (auto& simLatticeI : simulationLattice_)
      {
        copyStructureFromSimStateToState(stateSim_->state0(), *simLatticeI.statePtr);
        simLatticeI.statePtr->stateGrad().bindMat();
      }
    }

    // compute equal time lattice initial value and multiplier; if init, push_back the 0 lattice point
    simAppendIdx_ = _firstLaticeInvalidIdx;
    TNumType minArcLatticeVal = 0;
    if ((_firstLaticeInvalidIdx == 0) || (simulationLattice_.size() == 0))
    {
      setBeginStateToLattices(minArcLatticeVal);
    }
    else
    {
      minArcLatticeVal = simulationLattice_[simAppendIdx_ - 1].arc;
    }

    if (costsEvaluator_)
    {
      costsEvaluator_->resetCostFunctions(CostEvaluatorCostType::F);
      costsEvaluator_->resetCostFunctions(CostEvaluatorCostType::G);
      costsEvaluator_->resetCostFunctions(CostEvaluatorCostType::H);
    }

    if (isEmptyAllExtLattices() && (dt() > 0) && (ds() <= 0))
    {
      populatePartSimLatticesDtOnly(_firstLaticeInvalidIdx, arcParamMax);
    }  // case when only a dt lattice is available
    else
    {
      populatePartSimLatticesGeneral(_firstLaticeInvalidIdx, arcParamMax, minArcLatticeVal);
    }  // general case
  }

  ///@brief Returns true if all extended lattices are empty (the DS lattice as well as user-defined lattices).
protected:
  bool isEmptyAllExtLattices() const
  {
    bool emptyExtArcLat = true;
    for (size_t i = extArcLatIdxBegin; i < partLattices_.size(); ++i)
    {
      if (!partLattices_[i]->empty())
      {
        emptyExtArcLat = false;
        break;
      }
    }
    return emptyExtArcLat;
  }
  ///@brief Converts shifted (int) lattice index to container (size_t) index. @see @param BaseSimLatticeType.
public:
  static size_t lattTypeIdx(int _enumIdx)
  {
    return _enumIdx + asInt(BaseSimLatticeType::LATTICE_ENUM_SIZE);
  }

protected:
  void computeScaleDtDs()
  {
    dt_ = dtBase_;
    if (scaleDt_)
    {
      dt_ *= stateSim_->paramFuncs()->funcsArcEnd();
    }
    ds_ = dsBase_;
    if (scaleDs_)
    {
      stateSim_->paramFuncs()->setEvalArc(stateSim_->paramFuncs()->funcsArcEnd());
      ds_ *= stateSim_->paramFuncsDist()->computeS();
      stateSim_->paramFuncs()->setEvalArc(stateSim_->paramFuncs()->funcsArcBegin());
    }
  }

public:
  LatticePointType& simLatticeI(const size_t& _i)
  {
    return simulationLattice_[_i];
  }

public:
  const LatticePointType& simLatticeI(const size_t& _i) const
  {
    return simulationLattice_[_i];
  }

public:
  size_t simLatticeSize() const
  {
    return simulationLatticeActiveSize_;
  }

  ///@brief State simulator object.
protected:
  StateSimSPtr stateSim_;
  ///@brief Lattice requesting each simulated trajectory state.
private:
  std::vector<LatticePointType> simulationLattice_;

private:
  size_t simulationLatticeActiveSize_;
  ///@brief Vector containing the ordered sequence of arc parametrizations for each of the used lattices.
public:
  LatticeVecSPtrVec partLattices_;
  ///@brief Vector containing cached container indices for each partial lattice related to the the highest arc lower
  /// than the current evaluated arc.
protected:
  std::vector<int> partLatIdxCache_;
  ///@brief Flags if the @ref StateSim object has access to the @ref StateSimDist functionality.
protected:
  bool canComputeDistLattice_;
  ///@brief Arc parametrization interval used for the equal arc-length lattice.
private:
  TNumType dt_;
  ///@brief Arc parametrization interval used for the equal distance lattice.
private:
  TNumType ds_;

private:
  TNumType dtBase_;

private:
  TNumType dsBase_;

protected:
  bool scaleDt_;

protected:
  bool scaleDs_;

private:
  std::vector<std::vector<TNumType*>> userDefPartLattices_;

private:
  size_t simAppendIdx_;

private:
  bool firstTime_;

public:
  std::unique_ptr<CostsEvaluatorClass> costsEvaluator_;

  friend class TrajectorySimGrade;
  //     public   : size_t partLatticeActiveSize_;
};
}

#endif  // TRAJECTORY_SIMULATOR_HPP
