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

#ifndef MODEL_STAMPS_H
#define MODEL_STAMPS_H

#include <memory>

namespace tuw {

class ModelStamps;
using ModelStampsPtr      = std::shared_ptr<ModelStamps>;
using ModelStampsConstPtr = std::shared_ptr<ModelStamps const>;
class ModelStamps {
    
    //enums
    ///@brief Stamp type.
    public: enum class StampType { 
	TIME, 
	DIST 
    };
    
    //default class functions
    public: ModelStamps           () : s_ ( 0 ), t_ ( 0 ) {}
    public: virtual ~ModelStamps  ()                   = default;
    public: ModelStamps           (const ModelStamps&) = default;
    public: ModelStamps& operator=(const ModelStamps&) = default;
    public: ModelStamps           (ModelStamps&&)      = default;
    public: ModelStamps& operator=(ModelStamps&&)      = default;
    
    ///@brief Arc length parameter reference.
    public: double&       s ()       { return s_; }
    ///@brief Const arc length parameter reference.
    public: const double& s () const { return s_; }
    ///@brief Temporal parameter reference.
    public: double&       t ()       { return t_; }
    ///@brief Const temporal parameter reference.
    public: const double& t () const { return t_; }
    
    ///@brief Arc length parameter.
    protected: double s_;
    ///@brief Temporal parameter.
    protected: double t_;
};

}

#endif // MODEL_STAMPS_H