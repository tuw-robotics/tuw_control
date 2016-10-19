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

#ifndef TRAJECTORY_SIMULATOR_H
#define TRAJECTORY_SIMULATOR_H

#include <tuw_control/state/state_sim.h>
#include <tuw_control/param_func/param_func_dist.h>
#include <tuw_control/costs_evaluator/costs_evaluator.hpp>

#include <functional>
#include <memory>

namespace tuw {

class TrajectorySimulator;
using TrajectorySimulatorSPtr      = std::shared_ptr<TrajectorySimulator>;
using TrajectorySimulatorConstSPtr = std::shared_ptr<TrajectorySimulator const>;
using TrajectorySimulatorUPtr      = std::unique_ptr<TrajectorySimulator>;
using TrajectorySimulatorConstUPtr = std::unique_ptr<TrajectorySimulator const>;

class TrajectorySimulator {
    
    //enums
    ///@brief Mode of the simulation.
    public   : enum class SimMode {
	ONLINE ,///> Online evaluation. In this mode, every trajectory compuation step is performed incrementally for each lattice point: choosing the lattices order, simulation, grading (efficient when expected to interrupt simulation prematurely)
	PRECALC ///> Precomputed evaluation. In this mode, every sub-problem is executed entirely for all lattice points: choosing the lattices order, simulation, grading (efficient when full trajectory computation is desired)
    };
    ///@brief Fundamental lattice types.
    public   : enum class BaseSimLatticeType : signed int {
	ARC_BG_BK = -3,///> begin and end of the simulation lattice
	LATTICE_ARC_EQ_DT = -2,///> equal arc lattice
	LATTICE_ARC_EQ_DS = -1,///> equal distance lattice
	LATTICE_ENUM_SIZE =  3 ///> size of the enumerated values
    };
    static constexpr const std::size_t extArcLatIdxBegin = asInt(BaseSimLatticeType::LATTICE_ENUM_SIZE) + asInt(BaseSimLatticeType::LATTICE_ARC_EQ_DS);
    
    ///@brief Structure containing an evaluation arc and a state pointer.
    public   : struct LatticePoint { 
	LatticePoint(                                   ) : arc( -1 ), statePtr( nullptr ) {}
	LatticePoint( double _arc                       ) : arc(_arc), statePtr( nullptr ) {}
	LatticePoint( double _arc, StateSPtr& _statePtr ) : arc(_arc), statePtr(_statePtr) {}
	virtual ~LatticePoint(){}
	double      arc;
	StateSPtr   statePtr;
    };
    
    using LatticeVec        =                                std::vector<LatticePoint>;
    using LatticeVecSPtr    =              std::shared_ptr < std::vector<LatticePoint> >;
    using LatticeVecSPtrVec = std::vector< std::shared_ptr < std::vector<LatticePoint> > >;
    
    ///@brief Structure containing the lattice type afferent to a @param LatticePoint.
    public   : struct LatticePointType : public LatticePoint { 
	LatticePointType() : LatticePoint(), latticeType(-5) {}
	LatticePointType( double _arc, int _latticeType ) : LatticePoint(_arc), latticeType(_latticeType) {}
	LatticePointType( double _arc, int _latticeType, StateSPtr& _statePtr ) : LatticePoint(_arc, _statePtr), latticeType(_latticeType) {}
	int latticeType;
    };
    
    public   : using CostsEvaluatorClass = CostsEvaluatorBase<TrajectorySimulator::LatticeVec>;
    
    //special class member functions
    public   : TrajectorySimulator           ( StateSimPtr _stateSim );
    public   : TrajectorySimulator           ( StateSimPtr _stateSim, std::unique_ptr<CostsEvaluatorClass> _costsEvaluator );
    public   : virtual ~TrajectorySimulator  ()                           = default;
    public   : TrajectorySimulator           (const TrajectorySimulator&) = default;
    public   : TrajectorySimulator& operator=(const TrajectorySimulator&) = default;
    public   : TrajectorySimulator           (TrajectorySimulator&&)      = default;
    public   : TrajectorySimulator& operator=(TrajectorySimulator&&)      = default;
    
    
    public   : void setBoolDtScale ( const bool& _doScale );
    public   : void setBoolDsScale ( const bool& _doScale );
    
    ///@brief Reference to arc parametrization interval used for the equal arc-length lattice.
    public   :       double& dtBase ();
    ///@brief Const reference to arc parametrization interval used for the equal arc-length lattice.
    public   : const double& dt     () const;
    ///@brief Reference to arc parametrization interval used for the equal distance lattice.
    public   :       double& dsBase ();
    ///@brief Const reference to arc parametrization interval used for the equal distance lattice.
    public   : const double& ds     () const;
    ///@brief Reference of the state simulator object.
    public   :       StateSimPtr stateSim ();
    ///@brief Const reference of the state simulator object.
    public   : const StateSimPtr stateSim () const;
    ///@brief Reference to the lattice that requested each simulated trajectory state.
    public   :       std::vector<LatticePointType>& simLattice ();
    ///@brief Const reference to the lattice that requested each simulated trajectory state.
    public   : const std::vector<LatticePointType>& simLattice () const;
    ///@todo documentation
    public   : void setUserDefLattice   ( const std::vector< std::vector< double* > >& _userDefLattices );
    
    public   : void updateUserDefLattice(  );
    
    //pure virtual function
    /** @brief Simulates (discrete numerical evaluation) an entire trajectory according to the specified intervals and lattices.
     *  @param _lastValidArc Specifies the last arc at which the previous simulation was still valid. Setting it non-zero is beneficial if already computed (in memory) @ref trajStates()
     *  are guaranteed to not have changed. Thus, it allows the function to simulate only portions of the entire arc parametrization domain.
     */
    public   : virtual void simulateTrajectory ( double _lastValidArc = 0 ) = 0;
    
    ///@brief Performs simulation and populates simulation and partial lattice when only equal dt lattice is enabled.
    protected: virtual void populatePartSimLatticesDtOnly  ( const size_t& _firstLaticeInvalidIdx, double _arcParamMax );
    ///@brief Performs simulation and populates simulation and partial lattices in the general case of various enabled lattices.
    protected: virtual void populatePartSimLatticesGeneral (       size_t  _firstLaticeInvalidIdx, double _arcParamMax, double _minArcLatticeVal ) = 0; 
    
    ///@brief Performs a simulation step (if @param _arcNow is different than last simulated arc) and appends the new arc and state pointer to the afferent partial lattice point.
    protected: void   simAppendToSimPartLat          ( const double& _arcNow, const int& _latticePtType, const std::size_t& _minArcLatCacheIdx);
    ///@brief Appends the new arc and state pointer to the afferent partial lattice point.
    protected: void   appendToPartLat                ( const double& _arcNow, const int& _latticePtType, const std::size_t& _minArcLatCacheIdx);
    ///@brief Binds reference to the initial simulated state (at @param _arcBegin) to all partial lattices.
    protected: void   setBeginStateToLattices        ( const double& _arcBegin );
    ///@brief Binds reference to the final simulated state (at @param _arcEnd) to all partial lattices.
    protected: void   setEndStateToLattices          ( const double& _arcEnd   );
    ///@brief Sets begin and end arcs to all partial lattices on the first and last container positions.
    protected: void   setBeginEndArcsToLattices      ( const double& _arcBegin, const double& _arcEnd );
    
    ///@brief Initializes the simulation lattice (truncation from the @param _firstLaticeInvalidIdx) and sets the state simulator inital state (at @param _lastValidArc).
    protected: bool   initSimLatticeState0           ( const double& _lastValidArc, size_t& _firstLaticeInvalidIdx );
    ///@brief Initializes the cached partial lattices index at the highest arc lower than @param _minArcLatticeVal.
    protected: void   initExtLatticeCache            ( const double& _minArcLatticeVal );
    ///@brief Main function that performs resizing, reserving and calls the proper population function.
    protected: void   populateTrajSimPartLattice     ( const size_t& _firstLaticeInvalidIdx );
    
    ///@brief Returns true if all extended lattices are empty (the DS lattice as well as user-defined lattices).
    protected: bool   isEmptyAllExtLattices          () const;
    ///@brief Converts shifted (int) lattice index to container (size_t) index. @see @param BaseSimLatticeType.
    public   : static size_t lattTypeIdx             ( int _enumIdx );
    
    protected: void computeScaleDtDs();
    
    ///@brief State simulator object.
    protected: StateSimPtr   stateSim_;
    ///@brief Lattice requesting each simulated trajectory state.
    public   : std::vector< LatticePointType > simulationLattice_;
    ///@brief Vector containing the ordered sequence of arc parametrizations for each of the used lattices.
    public   : LatticeVecSPtrVec               partLattices_;
    ///@brief Vector containing cached container indices for each partial lattice related to the the highest arc lower than the current evaluated arc.
    protected: std::vector<        int       > partLatIdxCache_;
    ///@brief Flags if the @ref StateSim object has access to the @ref StateSimDist functionality.
    protected: bool canComputeDistLattice_;
    ///@brief Arc parametrization interval used for the equal arc-length lattice.
    private  : double dt_;
    ///@brief Arc parametrization interval used for the equal distance lattice.
    private  : double ds_;
    
    private  : double dtBase_;
    private  : double dsBase_;
    protected: bool   scaleDt_;
    protected: bool   scaleDs_;
    
    private  : std::vector<std::vector<double*> > userDefPartLattices_;
    

    public   : std::unique_ptr<CostsEvaluatorClass> costsEvaluator_;
    
    
    friend class TrajectorySimGrade;
//     public   : size_t partLatticeActiveSize_;
};

}

#endif // TRAJECTORY_SIMULATOR_H