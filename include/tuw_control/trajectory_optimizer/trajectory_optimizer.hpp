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

#ifndef TRAJECTORY_OPTIMIZER_HPP
#define TRAJECTORY_OPTIMIZER_HPP

#include <tuw_control/utils.h>
#include <eigen3/Eigen/Eigen>

#include <float.h>

namespace tuw
{
template <template <class, class, class...> class TLatticeType, template <class> class TCostsJ,
          template <class> class TCostsJG, template <class> class TCostsG, template <class> class TCostsH,
          template <class> class TCostsK>
class EvaluatedLattice
{
public:
  template <class TNumType, class TMapDataType>
  using FuncJ = TLatticeType<TNumType, TMapDataType, TCostsJ<TNumType> >;

public:
  template <class TNumType, class TMapDataType>
  using FuncG = TLatticeType<TNumType, TMapDataType, TCostsG<TNumType>, TCostsK<TNumType>, TCostsJG<TNumType> >;

public:
  template <class TNumType, class TMapDataType>
  using FuncH = TLatticeType<TNumType, TMapDataType, TCostsH<TNumType> >;

public:
  template <class TNumType, class TMapDataType>
  using FuncJGH = TLatticeType<TNumType, TMapDataType, TCostsJ<TNumType>, TCostsG<TNumType>, TCostsH<TNumType>,
                               TCostsK<TNumType>, TCostsJG<TNumType> >;
};

template <class TNumType>
inline static bool isSame(const Eigen::Matrix<TNumType, -1, 1>& _optVar0,
                          const Eigen::Matrix<TNumType, -1, 1>& _optVar1)
{
  const int optVar0Size = _optVar0.size();
  if (optVar0Size != _optVar1.size())
  {
    return false;
  }
  static constexpr const double eps = 2 * FLT_MIN;
  for (int i = 0; i < optVar0Size; ++i)
  {
    if (fabs(_optVar0(i) - _optVar1(i)) > eps)
    {
      return false;
    }
  }
  return true;
}

enum class OptCacheType
{
  LAST1,
  LAST2,
  ITER_START,
  ENUM_SIZE
};

///@todo return references to the cached stuff, do not copy them in the costs lolz :D
/// i.e. you copy ( store ) only when you see new stuff (to cache) but always return the cache data

template <class TNumType, class TTrajSim, class TMyParamType, template <class, class> class TOptVarMap>
class CostsEvaluatorCached : public TTrajSim
{
public:
  CostsEvaluatorCached() : TTrajSim(), idxCacheLast_(0)
  {
    for (size_t i = 0; i < asInt(OptCacheType::ENUM_SIZE); ++i)
    {
      xCacheConstr_[i] = std::shared_ptr<Eigen::Matrix<TNumType, -1, 1> >(new Eigen::Matrix<TNumType, -1, 1>);
      xCacheGradConstr_[i] = std::shared_ptr<Eigen::Matrix<TNumType, -1, 1> >( new Eigen::Matrix<TNumType, -1, 1>);
      for (size_t j = 0; j < TTrajSim::CostFuncsTypesNr; ++j)
      {
        constCache_[i][j] = std::shared_ptr<Eigen::Matrix<TNumType, -1, 1> >( new Eigen::Matrix<TNumType, -1, 1>);
        gradConstrCache_[i][j] = std::shared_ptr<Eigen::Matrix<TNumType, -1, -1> >(new Eigen::Matrix<TNumType, -1, -1>);
      }
    }
  }

public:
  auto& costs()
  {
    return this->costs_;
  }

public:
  auto& gradCosts()
  {
    return this->gradCostsMap_;
  }

public:
  bool evaluateCosts(const Eigen::Matrix<TNumType, -1, 1>& _x, const OptCacheType& _cacheType = OptCacheType::LAST1)
  {
    for (size_t i = 0; i < asInt(OptCacheType::ENUM_SIZE); ++i)
    {
      if (isSame(_x, *xCacheConstr_[i]))
      {
        idxEvalFunc_ = i;
        if (i < 2)
        {
          idxCacheLast_ = i;
        }  // we remember which one was used last time
        return true;
      }
    }

    // 	std::cout<<"computing it Func!"<<std::endl;
    size_t idxCache = asInt(_cacheType);
    if (_cacheType != OptCacheType::ITER_START)
    {
      idxCache = idxCacheLast_ = !idxCacheLast_;
    }
    setOptVar(_x);
    this->simulateTrajectory();
    *xCacheConstr_[idxCache] = _x;
    for (size_t j = 0; j < TTrajSim::CostFuncsTypesNr; ++j)
    {
      *constCache_[idxCache][j] = this->costs_.sub(j).data();
    }
    idxEvalFunc_ = idxCache;
    return false;
  }

public:
  bool evaluateCostsWithGrad(const Eigen::Matrix<TNumType, -1, 1>& _x,
                             const OptCacheType& _cacheType = OptCacheType::LAST1)
  {
    for (size_t i = 0; i < asInt(OptCacheType::ENUM_SIZE); ++i)
    {
      if (isSame(_x, *xCacheGradConstr_[i]))
      {
        idxEvalFunc_ = i;
        idxEvalGrad_ = i;
        if (i < 2)
        {
          idxCacheGradLast_ = i;
        }
        return true;
      }
    }
    // 	std::cout<<"computing it Grad!"<<std::endl;
    size_t idxCache = asInt(_cacheType);
    size_t idxCacheGrad = asInt(_cacheType);
    if (_cacheType != OptCacheType::ITER_START)
    {
      idxCache = idxCacheLast_ = !idxCacheLast_;
      idxCacheGrad = idxCacheGradLast_ = !idxCacheGradLast_;
    }
    setOptVar(_x);
    this->simulateTrajectoryWithGrad();
    *xCacheConstr_[idxCache] = *xCacheGradConstr_[idxCacheGrad] = _x;
    for (size_t j = 0; j < TTrajSim::CostFuncsTypesNr; ++j)
    {
      *constCache_[idxCache][j] = this->costs_.sub(j).data();
      *gradConstrCache_[idxCacheGrad][j] = *this->gradCostsMap_[j];
    }
    idxEvalFunc_ = idxCache;
    idxEvalGrad_ = idxCacheGrad;
    return false;
  }

public:
  const Eigen::Matrix<TNumType, -1, 1>& cachedCosts(const size_t& _i) const
  {
    return *constCache_[idxEvalFunc_][_i];
  }

public:
  const Eigen::Matrix<TNumType, -1, -1>& cachedGradCosts(const size_t& _i) const
  {
    return *gradConstrCache_[idxEvalGrad_][_i];
  }

public:
  void getOptVar(Eigen::Matrix<TNumType, -1, 1>& _optVarExt)
  {
    TOptVarMap<TNumType, TMyParamType>::getOptVar(_optVarExt, *this->stateSim()->paramStruct.get());
  }

private:
  void setOptVar(const Eigen::Matrix<TNumType, -1, 1>& _optVarExt)
  {
    TOptVarMap<TNumType, TMyParamType>::setOptVar(*this->stateSim()->paramStruct.get(), _optVarExt);
  }

private:
  std::array<std::shared_ptr<Eigen::Matrix<TNumType, -1, 1> >, asInt(OptCacheType::ENUM_SIZE)> xCacheConstr_;

private:
  std::array<std::shared_ptr<Eigen::Matrix<TNumType, -1, 1> >, asInt(OptCacheType::ENUM_SIZE)> xCacheGradConstr_;

private:
  std::array<std::array<std::shared_ptr<Eigen::Matrix<TNumType, -1, 1> >, TTrajSim::CostFuncsTypesNr>,
             asInt(OptCacheType::ENUM_SIZE)> constCache_;

private:
  std::array<std::array<std::shared_ptr<Eigen::Matrix<TNumType, -1, -1> >, TTrajSim::CostFuncsTypesNr>,
             asInt(OptCacheType::ENUM_SIZE)> gradConstrCache_;

private:
  size_t idxCacheLast_;

private:
  size_t idxCacheGradLast_;

private:
  size_t idxEvalFunc_;

private:
  size_t idxEvalGrad_;

  template <class TNumType2, class TTrajSimJGH2, class TTrajSimJ2, class TTrajSimG2, class TTrajSimH2,
            class TMyParamType2, template <class, class> class TOptVarMap2>
  friend class TrajectoryOptimizer;
};

template <class TNumType, class TTrajSimJGH, class TTrajSimJ, class TTrajSimG, class TTrajSimH, class TMyParamType,
          template <class, class> class TOptVarMap>
struct TrajectoryOptimizer
{
  TrajectoryOptimizer()
  {
    static constexpr const size_t idxIterStart = asInt(OptCacheType::ITER_START);
    trajSimJ.xCacheConstr_[idxIterStart] = trajSimJGH.xCacheConstr_[idxIterStart];
    trajSimG.xCacheConstr_[idxIterStart] = trajSimJGH.xCacheConstr_[idxIterStart];
    trajSimH.xCacheConstr_[idxIterStart] = trajSimJGH.xCacheConstr_[idxIterStart];

    trajSimJ.xCacheGradConstr_[idxIterStart] = trajSimJGH.xCacheGradConstr_[idxIterStart];
    trajSimG.xCacheGradConstr_[idxIterStart] = trajSimJGH.xCacheGradConstr_[idxIterStart];
    trajSimH.xCacheGradConstr_[idxIterStart] = trajSimJGH.xCacheGradConstr_[idxIterStart];

    trajSimJ.constCache_[idxIterStart][0] = trajSimJGH.constCache_[idxIterStart][0];
    trajSimG.constCache_[idxIterStart][0] = trajSimJGH.constCache_[idxIterStart][1];
    trajSimG.constCache_[idxIterStart][1] = trajSimJGH.constCache_[idxIterStart][3];  // here
    trajSimG.constCache_[idxIterStart][2] = trajSimJGH.constCache_[idxIterStart][4];  // here
    trajSimH.constCache_[idxIterStart][0] = trajSimJGH.constCache_[idxIterStart][2];

    trajSimJ.gradConstrCache_[idxIterStart][0] = trajSimJGH.gradConstrCache_[idxIterStart][0];
    trajSimG.gradConstrCache_[idxIterStart][0] = trajSimJGH.gradConstrCache_[idxIterStart][1];
    trajSimG.gradConstrCache_[idxIterStart][1] = trajSimJGH.gradConstrCache_[idxIterStart][3];  // here
    trajSimG.gradConstrCache_[idxIterStart][2] = trajSimJGH.gradConstrCache_[idxIterStart][4];  // here
    trajSimH.gradConstrCache_[idxIterStart][0] = trajSimJGH.gradConstrCache_[idxIterStart][2];
  }
  CostsEvaluatorCached<TNumType, TTrajSimJ, TMyParamType, TOptVarMap> trajSimJ;
  CostsEvaluatorCached<TNumType, TTrajSimG, TMyParamType, TOptVarMap> trajSimG;
  CostsEvaluatorCached<TNumType, TTrajSimH, TMyParamType, TOptVarMap> trajSimH;
  CostsEvaluatorCached<TNumType, TTrajSimJGH, TMyParamType, TOptVarMap> trajSimJGH;
  void initParamStruct(std::shared_ptr<TMyParamType>& _paramStructPtr)
  {
    trajSimJGH.stateSim()->paramStruct = _paramStructPtr;
    trajSimJ.stateSim()->paramStruct = _paramStructPtr;
    trajSimG.stateSim()->paramStruct = _paramStructPtr;
    trajSimH.stateSim()->paramStruct = _paramStructPtr;
    paramStructPtr = _paramStructPtr;
  }
  std::shared_ptr<TMyParamType> paramStructPtr;
};
}

#endif  // TRAJECTORY_OPTIMIZER_HPP
