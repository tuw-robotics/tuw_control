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

#ifndef TRAJECTORY_SIM_GRADE_H
#define TRAJECTORY_SIM_GRADE_H

#include <tuw_control/trajectory_simulator/trajectory_simulator_precalc.h>
#include <tuw_control/trajectory_simulator/trajectory_simulator_online.h>

#include <tuw_control/costs_evaluator/costs_evaluator.hpp>

#include <functional>
#include <memory>

namespace tuw
{
class TrajectorySimGrade;
using TrajectorySimGradeSPtr = std::shared_ptr<TrajectorySimGrade>;
using TrajectorySimGradeConstSPtr = std::shared_ptr<TrajectorySimGrade const>;
using TrajectorySimGradeUPtr = std::unique_ptr<TrajectorySimGrade>;
using TrajectorySimGradeConstUPtr = std::unique_ptr<TrajectorySimGrade const>;

class TrajectorySimGrade
{
  // special class member functions
public:
  TrajectorySimGrade(StateSimPtr& _stateSim);

public:
  TrajectorySimGrade(StateSimPtr& _stateSim, std::unique_ptr<TrajectorySimulator::CostsEvaluatorClass> _costsEvaluator);

public:
  ~TrajectorySimGrade() = default;

public:
  TrajectorySimGrade(const TrajectorySimGrade&) = default;

public:
  TrajectorySimGrade& operator=(const TrajectorySimGrade&) = default;

public:
  TrajectorySimGrade(TrajectorySimGrade&&) = default;

public:
  TrajectorySimGrade& operator=(TrajectorySimGrade&&) = default;

public:
  void initCostsEvaluator();

public:
  void evaluateTrajectory(const double& _arcBegin = 0);

public:
  void setSimMode(const TrajectorySimulator::SimMode& _simMode);

public:
  const TrajectorySimulator::SimMode& simMode() const;

public:
  TrajectorySimulatorSPtr& trajSim();

private:
  void modifyTrajSimMode();

private:
  TrajectorySimulatorSPtr trajSim_;

private:
  StateSimPtr stateSim_;

private:
  TrajectorySimulator::SimMode simMode_;

private:
  std::vector<std::vector<double*> > userPartLattices_;

  //     public   : std::unique_ptr<CostsEvaluatorClass> costsEvaluator_;
};
}

#endif  // TRAJECTORY_SIM_GRADE_H