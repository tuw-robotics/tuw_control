/***************************************************************************
 *   Software License Agreement (BSD License)                              *
 *   Copyright (C) 2015 by Horatiu George Todoran <todorangrg@gmail.com>   *
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
#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <float.h>
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <cfloat>
#include <tuw_control/utils.h>

#include <iostream>

namespace tuw {

/*!@class KalmanFilter
 * @brief Minimal cass implementing the Extended Kalman Filter algorithm
 * @tparam NumType Numerical type of the underlying variables (matrices, vectors)
 * @tparam XDim    Dimension of the state vector. Value -1 relates to dynamic size state vector
 */
template<typename NumType, int XDim>
class KalmanFilter {
    
    //special class member functions
    public   : KalmanFilter           ()                    = default;
    public   : virtual ~KalmanFilter  ()                    = default;
    public   : KalmanFilter           (const KalmanFilter&) = default;
    public   : KalmanFilter& operator=(const KalmanFilter&) = default;
    public   : KalmanFilter           (KalmanFilter&&)      = default;
    public   : KalmanFilter& operator=(KalmanFilter&&)      = default;
    
    public   : using NumericalType = NumType;          ///< Numerical type
    public   : static constexpr const int  xDim = XDim;///< %State vector size
    
    /** @brief Initializes state and covariance of filter.
     *  @param _x0     Initial state vector
     *  @param _Sigma0 Initial state covariance matrix
     */
    public   : void init   ( const Eigen::Matrix<NumType, XDim, 1>& _x0, const Eigen::Matrix<NumType, XDim, XDim>& _Sigma0 ) {
	x_     = _x0;
	Sigma_ = _Sigma0;
    }
    
    /** @brief Preforms Kalman prediction step.
     *  @param _f     %State transition function
     *  @param _Phi   %State transition matrix
     *  @param _Q     Process noise matrix (mapped in state-space)
     */
    public   : void predict( const Eigen::Matrix<NumType, XDim, 1   >& _f, 
	                     const Eigen::Matrix<NumType, XDim, XDim>& _Phi, 
			     const Eigen::Matrix<NumType, XDim, XDim>& _Q ) {
	x_     = _f;
	Sigma_ = _Phi * Sigma_ * _Phi.transpose() + _Q;
    }
    
    /** @brief Preforms Kalman update step.
     *  @param _deltah  Measurement error vector
     *  @param _C       %State-to-measurement matrix
     *  @param _R       Measurement noise matrix (mapped in measurement-space)
     */
    public   : template<int UpdateDim> void update ( const Eigen::Matrix<NumType, UpdateDim, 1        >& _deltah, 
						     const Eigen::Matrix<NumType, UpdateDim, XDim     >& _C, 
						     const Eigen::Matrix<NumType, UpdateDim, UpdateDim>& _R ) {
	
	Eigen::Matrix<NumType, UpdateDim, UpdateDim> S = _C * Sigma_ * _C.transpose() + _R;
	Eigen::Matrix<NumType, XDim     , UpdateDim> K = Sigma_ * _C.transpose() * S.inverse();
	x_     += K * _deltah;
	Sigma_  = ( Eigen::Matrix<NumType, XDim, XDim>::Identity( Sigma_.rows(), Sigma_.cols() ) - K * _C ) * Sigma_;
    }
    
    ///@brief %State vector const access.
    public   : const Eigen::Matrix<NumType, XDim, 1    >& x     () const { return x_;     }
    ///@brief %State covariance matrix const access.
    public   : const Eigen::Matrix<NumType, XDim, XDim >& Sigma () const { return Sigma_; }
    
    protected: Eigen::Matrix<NumType, XDim, 1   > x_;    ///< %State vector
    
    protected: Eigen::Matrix<NumType, XDim, XDim> Sigma_;///< %State covariance matrix
};
template<typename NumType, int XDim>
constexpr const int  KalmanFilter<NumType, XDim>::xDim;

/*!@class KalmanFilterPredictInterface
 * @brief Interface for simplified manipulation of specialized (Extended) Kalman Filter prediction part.
 * @tparam NumType Numerical type of the underlying variables (matrices, vectors)
 * @tparam XDim    Dimension of the state vector. Value -1 relates to dynamic size state vector
 * @tparam UDim    Dimension of the input vector. Value 0 relates to an input-less filter
 */
template<typename NumType, int XDim, size_t UDim, typename ParamType>
class KalmanFilterPredictInterface : public KalmanFilter<NumType, XDim> {
    
    //special class member functions
    public   : KalmanFilterPredictInterface           (ParamType& _params) : KalmanFilter<NumType, XDim>(), params_(_params) {}
    public   : virtual ~KalmanFilterPredictInterface  ()                                    = default;
    public   : KalmanFilterPredictInterface           (const KalmanFilterPredictInterface&) = default;
    public   : KalmanFilterPredictInterface& operator=(const KalmanFilterPredictInterface&) = default;
    public   : KalmanFilterPredictInterface           (KalmanFilterPredictInterface&&)      = default;
    public   : KalmanFilterPredictInterface& operator=(KalmanFilterPredictInterface&&)      = default;
    
    public   : using ParamsType = ParamType;///< Parameter type
    
    /** @brief Initializes the state vector and triggers initialization of @ref Sigma_.
     * 
     *  For dynamic state vector size, @ref Sigma_ is being properly resized (as quadratic matrix) but its content is *not* initialized.
     * 
     *  @param _x0 Initial state
     * 
     *  @see @ref computeSigmaInit for the interface of the state covariance matrix initialization.
     */
    public   : void init    ( const Eigen::Matrix<NumType, XDim, 1>& _x0 ) { 
	this->x_ = _x0; 
	this->Sigma_.resize( _x0.rows(), _x0.rows() ); computeSigmaInit(); 
    }
    
    /** @brief Performs the prediction step.
     * 
     *  This function performs no resizing on the internal filter variables and is thus intended for external use only for compile-time constant filter state size.
     * 
     *  The order in which the implementation-specific functions are called is: @ref precompute, @ref computePhi, @ref computef, @ref computeQ.
     *  Note that @ref computePhi is called before @ref computef and thus @ref Phi_ can be used in @ref computef (for linear prediction models).
     * 
     *  @param _u  Inputs vector
     *  @param _Ta Time duration since last prediction step
     * 
     *  @see @ref precompute, @ref computePhi, @ref computef, @ref computeQ
     */
    public   : template<int i = XDim, typename std::enable_if< (i >=  0) >::type* = nullptr> void predict ( const Eigen::Matrix<NumType, UDim, 1>& _u,  const double& _Ta   ) { 
	precompute(_Ta);
	computePhi(); 
	computef  (_u);
	computeQ  (); 
	KalmanFilter<NumType, XDim>::predict( f_, Phi_, Q_ ); 
    }
    
    /** @brief Performs the prediction step with resizing.
     * 
     *  This function performs resizing on the internal filter variables and later calls its its static counterpart. It is intended for external use for dynamic filter state vector size.
     * 
     *  For dynamic state vector size, @ref f_, @ref Phi_ and @ref Q_ are being properly resized but their content is *not* initialized.
     * 
     *  @see @ref predict version for constant state vector size.
     */
    public   : template<int i = XDim, typename std::enable_if< (i == -1) >::type* = nullptr> void predict ( const Eigen::Matrix<NumType, UDim, 1>& _u,  const double& _Ta   ) { 
	f_    .resize( this->x_.rows(),               1 ); 
	Phi_  .resize( this->x_.rows(), this->x_.rows() ); 
	Q_    .resize( this->x_.rows(), this->x_.rows() ); 
	predict<0>(_u, _Ta);
    }
    
    public   : template<int i = UDim, typename std::enable_if< (i == 0) >::type* = nullptr> void predict ( const double& _Ta ) { 
	static Eigen::Matrix<double, 0, 1> uEmpty_;
	f_    .resize( this->x_.rows(),               1 ); 
	Phi_  .resize( this->x_.rows(), this->x_.rows() ); 
	Q_    .resize( this->x_.rows(), this->x_.rows() ); 
	predict(uEmpty_, _Ta);
    }
    
    /// @brief Parameters const acces.
    public   : const ParamType& param () const { return params_; }
    
    protected: Eigen::Matrix<NumType, XDim, 1   > f_;  ///<  %State transition function vector
    protected: Eigen::Matrix<NumType, XDim, XDim> Phi_;///<  %State transition function partial derivative (Jacobian) with respect to the state variables
    protected: Eigen::Matrix<NumType, XDim, XDim> Q_;  ///<  Process noise matrix (mapped in state-space)
    protected: ParamType& params_;///< Filter parameters.
    
    /// @brief Interface for precomputation function called at the beginning of the prediction step.
    protected: virtual void precompute      ( const double& _Ta ) = 0;
    /// @brief Interface for initialization of the state covariance matrix @ref Sigma_.
    protected: virtual void computeSigmaInit() = 0;
    /// @brief Interface for computation of the state transition matrix @ref Phi_.
    protected: virtual void computePhi      () = 0;
    /// @brief Interface for computation of the state transition function vector @ref f_.
    protected: virtual void computef        ( const Eigen::Matrix<NumType, UDim, 1   >& _u ) = 0;
    /// @brief Interface for computation of the state process noise matrix @ref Q_.
    protected: virtual void computeQ        () = 0;
};


/*!@class KalmanFilterUpdateInterface
 * @brief Interface for simplified manipulation of specialized (Extended) Kalman Filter updates. To be used with @ref KalmanFilterPredictInterface.
 * @tparam KFPredType Kalman prediction class (extended from @ref KalmanFilterPredictInterface)
 * @tparam HDim       Dimension of the measurement vector. Value -1 relates to dynamic size measurement vector
 */
template<typename KFPredType, int HDim>
class KalmanFilterUpdateInterface {
    //special class member functions
    public   : KalmanFilterUpdateInterface           ()                                   = default;
    public   : virtual ~KalmanFilterUpdateInterface  ()                                   = default;
    public   : KalmanFilterUpdateInterface           (const KalmanFilterUpdateInterface&) = default;
    public   : KalmanFilterUpdateInterface& operator=(const KalmanFilterUpdateInterface&) = default;
    public   : KalmanFilterUpdateInterface           (KalmanFilterUpdateInterface&&)      = default;
    public   : KalmanFilterUpdateInterface& operator=(KalmanFilterUpdateInterface&&)      = default;
    
    public   : static constexpr const int hDim = HDim;///< Measurement vector size
    
    public   : const Eigen::Matrix<typename KFPredType::NumericalType, HDim, 1                >& deltah () const  { return deltah_; }///< Const access of measurement function error vector @ref h_
    public   : const Eigen::Matrix<typename KFPredType::NumericalType, HDim, KFPredType::xDim >& H      () const  { return H_     ; }///< Const access of predicted measurement matrix @ref H_
    public   : const Eigen::Matrix<typename KFPredType::NumericalType, HDim, HDim             >& R      () const  { return R_     ; }///< Const access of measurement noise matrix @ref R_
    
    protected: Eigen::Matrix<typename KFPredType::NumericalType, HDim, 1                > deltah_; ///< Measurement function error vector
    protected: Eigen::Matrix<typename KFPredType::NumericalType, HDim, KFPredType::xDim > H_; ///< Predicted measurement partial derivative (Jacobian) with respect to the state vector
    protected: Eigen::Matrix<typename KFPredType::NumericalType, HDim, HDim             > R_; ///< Measurement noise matrix (mapped in measurement-space)
    
    /// @brief Interface for precomputation function called at the beginning of the update step.
    protected: virtual void precompute     (const KFPredType* _kf) = 0;
    /// @brief Interface for computation of the predicted measurement matrix @ref H_.
    protected: virtual void computeH       (const KFPredType* _kf) = 0;
    /// @brief Interface for computation of the measurement function error vector @ref deltah_.
    protected: virtual void computeDeltah  (const KFPredType* _kf, const Eigen::Matrix<typename KFPredType::NumericalType, hDim, 1>& _zObs) = 0;
    /// @brief Interface for computation of the measurement noise matrix @ref R_.
    protected: virtual void computeR       (const KFPredType* _kf) = 0;
    
    template<typename KFPredTypeI, typename...  KFUpdateType > friend class KalmanFilterInterface;
};
template<typename KFPredType, int HDim>
constexpr const int  KalmanFilterUpdateInterface<KFPredType, HDim>::hDim;

/*!@class KalmanFilterInterface
 * @brief Interface for simplified manipulation of specialized (Extended) Kalman Filter implementations. 
 * @tparam KFPredType   Kalman prediction class (extended from @ref KalmanFilterPredictInterface)
 * @tparam KFUpdateType Parameter pack of implemented @ref KalmanFilterUpdateInterface classes that are used to update the filter
 * As building blocks, a @ref KalmanFilterPredictInterface and multilple @ref KalmanFilterUpdateInterface have to be defined.
 */
template<typename KFPredType, typename...  KFUpdateType >
class KalmanFilterInterface : public KFPredType {
    //special class member functions
    public   : KalmanFilterInterface           (typename KFPredType::ParamsType& _params) : KFPredType(_params) {}
    public   : virtual ~KalmanFilterInterface  ()                             = default;
    public   : KalmanFilterInterface           (const KalmanFilterInterface&) = default;
    public   : KalmanFilterInterface& operator=(const KalmanFilterInterface&) = default;
    public   : KalmanFilterInterface           (KalmanFilterInterface&&)      = default;
    public   : KalmanFilterInterface& operator=(KalmanFilterInterface&&)      = default;
    
    /** @brief Performs the update step defined by the template argument @ref KFUpdateTypeI (defaults to first updater class).
     * 
     *  This function performs no resizing on the internal filter variables and is thus intended for external use only for compile-time constant filter state size.
     * 
     *  The order in which the implementation-specific functions are called is: @ref KalmanFilterUpdateInterface::precompute, @ref KalmanFilterUpdateInterface::computeH, @ref KalmanFilterUpdateInterface::computeh, @ref KalmanFilterUpdateInterface::computeR.
     *  Note that @ref computeH is called before @ref KalmanFilterUpdateInterface::computeh and thus @ref KalmanFilterUpdateInterface::H_ can be used in @ref KalmanFilterUpdateInterface::computeh (for linear prediction models).
     * 
     *  @param _u  Inputs vector.
     *  @param _Ta Time duration since last prediction step.
     * 
     *  @see @ref KalmanFilterUpdateInterface::precompute, @ref KalmanFilterUpdateInterface::computeH, @ref KalmanFilterUpdateInterface::computeh, @ref KalmanFilterUpdateInterface::computeR
     */
    public   : template<typename KFUpdateTypeI = typename std::tuple_element<0, std::tuple< KFUpdateType... >>::type, int i = KFUpdateTypeI::hDim, typename std::enable_if< (i >=  0) >::type* = nullptr> 
	       void update  ( const Eigen::Matrix<typename KFPredType::NumericalType, KFUpdateTypeI::hDim, 1>& _zObs ) { 
	auto& updI = std::get<KFUpdateTypeI>(updaters_); 
	updI.precompute(this);
	updI.computeH      (this); 
	updI.computeDeltah (this, _zObs);  
	updI.computeR      (this); 
	KFPredType::update( updI.deltah(), updI.H(),  updI.R() );
    }
    
    /** @brief Performs the update step  by the template argument @ref KFUpdateTypeI (defaults to first updater class) with resizing.
     * 
     *  This function performs resizing on the internal filter variables and later calls its its static counterpart. It is intended for external use for dynamic filter measurement vector size.
     * 
     *  For dynamic measurement vector size, @ref KalmanFilterUpdateInterface::h_, @ref KalmanFilterUpdateInterface::H_ and @ref KalmanFilterUpdateInterface::R_ are being properly resized but their content is *not* initialized.
     * 
     *  @see @ref update for constant measurement vector size
     */
    public   : template<typename KFUpdateTypeI = typename std::tuple_element<0, std::tuple< KFUpdateType... >>::type, int i = KFUpdateTypeI::hDim, typename std::enable_if< (i == -1) >::type* = nullptr> 
	       void update  ( const Eigen::Matrix<typename KFPredType::NumericalType, KFUpdateTypeI::hDim, 1>& _zObs ) { 
	auto& updI = std::get<KFUpdateTypeI>(updaters_); 
	updI.deltah_.resize(_zObs.rows(), 1               ); 
	updI.H_     .resize(_zObs.rows(), this->x_.rows() ); 
	updI.R_     .resize(_zObs.rows(),    _zObs.rows() ); 
	update<KFUpdateTypeI, 0>(_zObs);
    }
    private  : std::tuple< KFUpdateType... > updaters_;///< Container
};

}

#endif // KALMAN_FILTER_H
