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
#include <tuw_control/filters/ekf.h>
#include <tuw_control/filters/kalman_filter.hpp>


using namespace cv;
using namespace std;
using namespace tuw;

EKFBase::EKFBase( const std::size_t xSize, const size_t uSize, const size_t zSize ): xSize_(xSize), uSize_(uSize), zSize_(zSize) {
    MatEyexSize    = Mat::eye(xSize_  , xSize_  , CV_64F);
    x_ = Mat::zeros(xSize_,1,CV_64F);
    
    
//     Paramss testParams;
//     KFTest kfTest(testParams);
//     Eigen::Matrix<double, 0, 0> testU;
//     kfTest.predict(testU,0);
//     Eigen::Matrix<double, 3, 1> zUpd1;
//     kfTest.update<KFUpdate1>(zUpd1);
//     kfTest.resize();
//     
//     
//     KFConstTest kfConstTest(testParams);
//     kfConstTest.predict(testU,0);
//     kfConstTest.update<KFConstUpdate1>(zUpd1);
//     kfConstTest.resize();
}

void EKFBase::init ( const cv::Mat& x0, const cv::Mat& Sigma0 ) {
    x0    .copyTo(x_);
    Sigma0.copyTo(Sigma_);
}

void EKFBase::predict( const cv::Mat& f__xu, const cv::Mat& Phi, const cv::Mat& Q ) {
    f__xu.copyTo( x_.rowRange(0,f__xu.rows).col(0) );
    
    Mat SigmaRed = Sigma_.rowRange(0,f__xu.rows).colRange(0,f__xu.rows);
    
    Mat(Phi * SigmaRed * Phi.t() + Q).copyTo(SigmaRed);
}

void EKFBase::update ( const cv::Mat& zHat, const cv::Mat& z, const cv::Mat& C, const cv::Mat& R ) {
    Mat SigmaRed = Sigma_.rowRange(0,C.cols).colRange(0,C.cols);
    
    Mat S = C * SigmaRed * C.t() + R;
    Mat K = SigmaRed * C.t() * S.inv(DECOMP_SVD);
    x_.rowRange(0,C.cols).col(0) += K * (z - zHat );
    Mat((MatEyexSize.rowRange(0,C.cols).colRange(0,C.cols) - K * C) * SigmaRed).copyTo(SigmaRed);
}
