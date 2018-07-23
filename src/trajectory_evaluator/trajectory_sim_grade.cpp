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

#include <tuw_control/trajectory_evaluator/trajectory_sim_grade.h>
#include <tuw_control/utils.h>

#include <algorithm>

#include <iostream>

using namespace std;
using namespace tuw;

TrajectorySimGrade::TrajectorySimGrade(StateSimPtr& _stateSim)
  : stateSim_(_stateSim), simMode_(TrajectorySimulator::SimMode::ONLINE)
{
  trajSim_ = make_shared<TrajectorySimulatorOnline>(stateSim_);
}

TrajectorySimGrade::TrajectorySimGrade(StateSimPtr& _stateSim,
                                       unique_ptr<TrajectorySimulator::CostsEvaluatorClass> _costsEvaluator)
  : stateSim_(_stateSim), simMode_(TrajectorySimulator::SimMode::ONLINE)
{
  trajSim_ = make_shared<TrajectorySimulatorOnline>(stateSim_, std::move(_costsEvaluator));
}

void TrajectorySimGrade::modifyTrajSimMode()
{
  double dt = -1, ds = -1;
  std::vector<TrajectorySimulator::LatticePointType> simulationLattice;
  TrajectorySimulator::LatticeVecSPtrVec partLattices;
  bool copy = false;
  if (trajSim_ != nullptr)
  {
    copy = true;
    dt = trajSim_->dtBase();
    ds = trajSim_->dsBase();
    simulationLattice = trajSim_->simLattice();
    partLattices = trajSim_->partLattices_;
    userPartLattices_ = trajSim_->userDefPartLattices_;
  }
  if (simMode_ == TrajectorySimulator::SimMode::ONLINE)
  {
    trajSim_ = make_shared<TrajectorySimulatorOnline>(trajSim_->stateSim(), std::move(trajSim_->costsEvaluator_));
  }
  else if (simMode_ == TrajectorySimulator::SimMode::PRECALC)
  {
    trajSim_ = make_shared<TrajectorySimulatorPrecalc>(trajSim_->stateSim(), std::move(trajSim_->costsEvaluator_));
  }
  if (copy)
  {
    trajSim_->dtBase() = dt;
    trajSim_->dsBase() = ds;
    trajSim_->simLattice() = simulationLattice;
    trajSim_->partLattices_ = partLattices;
    trajSim_->userDefPartLattices_ = userPartLattices_;
    if (trajSim_->costsEvaluator_)
    {
      trajSim_->updateUserDefLattice();
      initCostsEvaluator();
    }
  }
}

void TrajectorySimGrade::initCostsEvaluator()
{
  trajSim_->costsEvaluator_->init(trajSim_->partLattices_);
}

void TrajectorySimGrade::setSimMode(const TrajectorySimulator::SimMode& _simMode)
{
  if (simMode_ != _simMode)
  {
    simMode_ = _simMode;
    modifyTrajSimMode();
  }
}

const TrajectorySimulator::SimMode& TrajectorySimGrade::simMode() const
{
  return simMode_;
}

TrajectorySimulatorSPtr& TrajectorySimGrade::trajSim()
{
  return trajSim_;
}

void TrajectorySimGrade::evaluateTrajectory(const double& _arcBegin)
{
  trajSim_->stateSim()->paramFuncs()->precompute();
  trajSim_->simulateTrajectory(_arcBegin);
}
