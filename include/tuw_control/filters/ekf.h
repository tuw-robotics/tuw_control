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
#ifndef EKFBASE_H
#define EKFBASE_H

#include <opencv2/opencv.hpp>

namespace tuw {

class EKFBase {
public:
    
    //special class member functions
    public   : EKFBase           (const std::size_t xSize, const std::size_t uSize, const std::size_t zSize);
    public   : virtual ~EKFBase  ()               = default;
    public   : EKFBase           (const EKFBase&) = default;
    public   : EKFBase& operator=(const EKFBase&) = default;
    public   : EKFBase           (EKFBase&&)      = default;
    public   : EKFBase& operator=(EKFBase&&)      = default;
    
    
    ///@brief Initializes state and covariance of filter.
    public   : void init   ( const cv::Mat& x0, const cv::Mat& Sigma0 );
    
    /** @brief Preforms Kalman prediction step.
     *  @param f__xu New predicted state
     *  @param Phi   State transition matrix
     *  @param Q     Input noise matrix (computed in state-space)
     */
    public   : void predict( const cv::Mat& f__xu, const cv::Mat& Phi, const cv::Mat& Q );
    /** @brief Preforms Kalman update step.
     *  @param zHat Expected measurements
     *  @param z    Measurements
     *  @param C    State-to-measurement matrix
     *  @param R    Measurement noise matrix (computed in measurement-space)
     */
    public   : void update ( const cv::Mat& zHat , const cv::Mat& z  , const cv::Mat& C, const cv::Mat& R );
    
    public   : const std::size_t xSize() const { return xSize_; }///>state size
    public   : const std::size_t uSize() const { return uSize_; }///>input size
    public   : const std::size_t zSize() const { return zSize_; }///>measurement size

    protected: cv::Mat Sigma_;///>state covariance matrix
    protected: cv::Mat x_;   ;///>state

    private  : std::size_t xSize_;///>state size
    private  : std::size_t uSize_;///>input size
    private  : std::size_t zSize_;///>measurement size
    private  : cv::Mat     MatEyexSize;///>Eye matrix of state size
};

}

#endif // EKFBASE_H
