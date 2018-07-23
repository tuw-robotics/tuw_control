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

#ifndef TRAJECTORY_SIMULATOR_ONLINE_H
#define TRAJECTORY_SIMULATOR_ONLINE_H

#include <tuw_control/trajectory_simulator/trajectory_simulator.h>

#include <functional>

namespace tuw
{
class TrajectorySimulatorOnline : public TrajectorySimulator
{
  // special class member functions
public:
  TrajectorySimulatorOnline(StateSimPtr _stateSim);

public:
  TrajectorySimulatorOnline(StateSimPtr _stateSim, std::unique_ptr<CostsEvaluatorClass> _costsEvaluator);

public:
  virtual ~TrajectorySimulatorOnline() = default;

public:
  TrajectorySimulatorOnline(const TrajectorySimulatorOnline&) = default;

public:
  TrajectorySimulatorOnline& operator=(const TrajectorySimulatorOnline&) = default;

public:
  TrajectorySimulatorOnline(TrajectorySimulatorOnline&&) = default;

public:
  TrajectorySimulatorOnline& operator=(TrajectorySimulatorOnline&&) = default;

public:
  void simulateTrajectory(double _lastValidArc = 0) override;

private:
  void populatePartSimLatticesGeneral(std::size_t _firstLaticeInvalidIdx, double _arcParamMax,
                                      double _minArcLatticeVal) override;

private:
  void populatePartSimLatticesDtOnly(const size_t& _firstLaticeInvalidIdx, double _arcParamMax) override;

  ///@brief Performs lattices resizing relevant to the general online simulation algorithm.
private:
  void resizeBeginGeneral(const double& _arcParamMax);
  ///@brief Performs lattices resizing relevant to the DT-only online simulation algorithm.
private:
  void resizeBeginDtOnly(const double& _arcParamMax);
  ///@brief Numeric safe incremental rounding of x (finds the smallest multiple of @param _dx larger than @param _x).
private:
  double toNextIntMult(const double& _x, const double& _dx) const;
};
}

#endif  // TRAJECTORY_SIMULATOR_ONLINE_H