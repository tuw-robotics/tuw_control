/***************************************************************************
 *   Software License Agreement (BSD License)                              *
 *   Copyright (C) 2016 by Horatiu George Todoran <todorangrg@gmail.com>   *
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

#ifndef COSTS_EVALUATOR_H
#define COSTS_EVALUATOR_H

#include <vector>
#include <math.h>
#include <float.h>
#include <functional>

#include <tuw_control/costs_evaluator/cost_functions.hpp>

#include <eigen3/Eigen/Core>

#include <iostream>

namespace tuw
{
enum class CostEvaluatorCostType
{
  F,
  H,
  G,
  ENUM_SIZE
};

template <typename Lattice>
class CostsEvaluatorBase
{
  // special class member functions
public:
  CostsEvaluatorBase()
  {
  }

public:
  virtual ~CostsEvaluatorBase() = default;

public:
  CostsEvaluatorBase(const CostsEvaluatorBase&) = default;

public:
  CostsEvaluatorBase& operator=(const CostsEvaluatorBase&) = default;

public:
  CostsEvaluatorBase(CostsEvaluatorBase&&) = default;

public:
  CostsEvaluatorBase& operator=(CostsEvaluatorBase&&) = default;

public:
  virtual void loadCostFunctions() = 0;

public:
  virtual void init(std::vector<std::shared_ptr<Lattice>>& _lattPtr) = 0;

public:
  virtual void resetCostFunctions(const CostEvaluatorCostType& _arrayType) = 0;

public:
  virtual bool evalValidCostStep(const CostEvaluatorCostType& _arrayType, double _arcNow, size_t& _violatingLatIdx,
                                 double& _arcMax) = 0;

protected:
  virtual void computeScalarCost(double& _f) = 0;

protected:
  virtual void computeArrayCost(std::vector<double>& _arr, const CostEvaluatorCostType& _arrayType) = 0;

public:
  void evaluateF()
  {
    computeScalarCost(f);
  }

public:
  void evaluateH()
  {
    computeArrayCost(h, CostEvaluatorCostType::H);
  }

public:
  void evaluateG()
  {
    computeArrayCost(g, CostEvaluatorCostType::G);
  }

public:
  void evaluateAllCosts()
  {
    computeScalarCost(f);
    computeArrayCost(h, CostEvaluatorCostType::H);
    computeArrayCost(g, CostEvaluatorCostType::G);
  }

public:
  bool hIsValid() const
  {
    if ((f > FLT_MAX) || (std::isnan(f)))
    {
      return false;
    }
    for (const auto& hI : h)
    {
      if (hI <= 0)
      {
        return false;
      }
    }
    return true;
  }

public:
  bool gIsValid(const double& _boxBound = 1e-2) const
  {
    for (const auto& gI : g)
    {
      if (fabs(gI) >= _boxBound)
      {
        return false;
      }
    }
    return true;
  }

public:
  bool gIsValid(const size_t _Idx, const double& _boxBound = 1e-2) const
  {
    if (fabs(g[_Idx]) >= _boxBound)
    {
      return false;
    }
    return true;
  }

public:
  double f;

public:
  std::vector<double> h;

public:
  std::vector<double> g;

public:
  std::vector<double> gradF;

public:
  Eigen::MatrixXd gradH;

public:
  Eigen::MatrixXd gradG;
};

template <typename Lattice, typename MapData>
class CostsEvaluator : public CostsEvaluatorBase<Lattice>
{
public:
  CostsEvaluator(std::shared_ptr<MapData>& _mapDataPtr)
    : partialCostsArray_(asInt(CostEvaluatorCostType::ENUM_SIZE)), mapDataPtr_(_mapDataPtr)
  {
  }

public:
  virtual ~CostsEvaluator() = default;

public:
  CostsEvaluator(const CostsEvaluator&) = default;

public:
  CostsEvaluator& operator=(const CostsEvaluator&) = default;

public:
  CostsEvaluator(CostsEvaluator&&) = default;

public:
  CostsEvaluator& operator=(CostsEvaluator&&) = default;

public:
  void init(std::vector<std::shared_ptr<Lattice>>& _lattPtr) override
  {
    for (auto& partialCostsArrayI : partialCostsArray_)
    {
      partialCostsArrayI.clear();
    }
    this->loadCostFunctions();
    for (auto& funcI : partialCostsArray_)
    {
      for (auto& partFuncI : funcI)
      {
        partFuncI->initLatticeMap(_lattPtr, mapDataPtr_);
      }
    }
  }

protected:
  void computeScalarCost(double& _f)
  {
    {
      auto& funcI = partialCostsArray_[asInt(CostEvaluatorCostType::F)];
      for (auto& partFuncI : funcI)
      {
        partFuncI->reset();
        partFuncI->calcCostsFull();
      }
    }

    _f = 0;
    for (auto& partFuncI : partialCostsArray_[asInt(CostEvaluatorCostType::F)])
    {
      for (size_t k = 0; k < partFuncI->costsSize(); ++k)
      {
        _f += partFuncI->cost(k);
      }
    }
  }

protected:
  void computeArrayCost(std::vector<double>& _arr, const CostEvaluatorCostType& _arrayType) override
  {
    {
      auto& funcI = partialCostsArray_[asInt(_arrayType)];
      for (auto& partFuncI : funcI)
      {
        partFuncI->reset();
        partFuncI->calcCostsFull();
      }
    }

    size_t arrSize = 0;
    for (auto& partFuncI : partialCostsArray_[asInt(_arrayType)])
    {
      arrSize += partFuncI->costsSize();
    }
    _arr.resize(arrSize);

    size_t iH = 0, iP;
    iP = 0;
    for (; iP < partialCostsArray_[asInt(_arrayType)].size(); ++iP)
    {
      for (size_t k = 0; k < partialCostsArray_[asInt(_arrayType)][iP]->costsSize(); ++k, ++iH)
      {
        _arr[iH] = partialCostsArray_[asInt(_arrayType)][iP]->cost(k);
      }
    }
  }

public:
  void resetCostFunctions(const CostEvaluatorCostType& _arrayType) override
  {
    {
      auto& funcI = partialCostsArray_[asInt(_arrayType)];
      for (auto& partFuncI : funcI)
      {
        partFuncI->reset();
      }
    }
    firstAfterReset_ = true;
  }

public:
  bool evalValidCostStep(const CostEvaluatorCostType& _arrayType, double _arcNow, size_t& _violatingLatIdx,
                         double& _arcMax) override
  {
    if (firstAfterReset_)
    {
      {
        auto& funcI = partialCostsArray_[asInt(_arrayType)];
        for (auto& partFuncI : funcI)
        {
          partFuncI->reset();
        }
        firstAfterReset_ = false;
      }
    }
    {
      auto& funcI = partialCostsArray_[asInt(_arrayType)];
      for (auto& partFuncI : funcI)
      {
        partFuncI->resetNew();
      }
    }
    double minCost = FLT_MAX;
    double arcMin = FLT_MAX;
    {
      auto& funcI = partialCostsArray_[asInt(_arrayType)];
      for (auto& partFuncI : funcI)
      {
        size_t k = partFuncI->iterIdxPartBegin_;
        partFuncI->calcCosts1KnotStep(_arcNow);
        size_t kend = std::min(k + 1, partFuncI->costsSize());
        for (; k < kend; ++k)
        {
          const double& costK = partFuncI->cost(k);
          if (costK < 0)
          {
            double violLatNew = partFuncI->arcAtLattIdxPrev();
            if ((minCost >= 0) || (violLatNew < arcMin))
            {
              minCost = costK;
              arcMin = violLatNew;
              _violatingLatIdx = partFuncI->knotLatIdx();
              _arcMax = partFuncI->arcAtLattIdxPrev();
            }
          }
        }
      }
    }
    return minCost >= 0;
  }

public:
  std::vector<std::vector<std::unique_ptr<cost_functions::CostsArrayLatBase<Lattice, MapData>>>> partialCostsArray_;

public:
  std::shared_ptr<MapData>& mapDataPtr()
  {
    return mapDataPtr_;
  }

private:
  std::shared_ptr<MapData> mapDataPtr_;

private:
  bool firstAfterReset_;
};
}

#endif  // COSTS_EVALUATOR_H
