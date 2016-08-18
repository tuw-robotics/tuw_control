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

#ifndef INTEGRATOR_HPP
#define INTEGRATOR_HPP

#include <float.h>
#include <memory>

namespace tuw {

/*!@class Integrator
 * @todo document tthe Kahan summation algorithm
 * 
 */
class Integrator {
    
    //special class member functions
    public   : Integrator           ()                  = default;
    public   : virtual ~Integrator  ()                  = default;
    public   : Integrator           (const Integrator&) = default;
    public   : Integrator& operator=(const Integrator&) = default;
    public   : Integrator           (Integrator&&)      = default;
    public   : Integrator& operator=(Integrator&&)      = default;
    
    public   : virtual void     reset ( const double& _x0 )       { x_ = _x0; comp_ = 0; }
    
    public   : const double& intOutput()                    const { return x_; }
    public   : void          integrate( const double& _x  )       {
	double y = _x - comp_;
        double t = x_ + y;    
        comp_ = (t - x_) - y;
        x_ = t;  
    }
    
    private  : double x_;
    private  : double comp_;
};



}

#endif // STATE_FEEDBACK_HPP