/***************************************************************************
 *   Software License Agreement (BSD License)                              *
 *   Copyright (C) 2016 by Markus Bader <markus.bader@tuwien.ac.at         *
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

#ifndef STATE_MAP_BASE_TYPES_H
#define STATE_MAP_BASE_TYPES_H


#include <tuw_control/state_map/state_map.h>

namespace tuw {

// \brief Point structure consisting of x and y value
struct Point : public AdditionalMethods<Point> {
    double x;
    double y;

    Point() = default;

    Point ( double x_, double y_ )
        : x ( x_ ),
          y ( y_ ) {

    }

    using AdditionalMethods::operator[];
    double& operator[] ( size_t pos ) {
        return at ( pos, x, y );
    }

    size_t size() const {
        return tuw::size ( x, y );
    }
};

// \brief Rectangle structure consisting of two points
struct Rect : public AdditionalMethods<Rect> {
    Point lowerLeftPoint;
    Point upperRightPoint;

    Rect() = default;

    Rect ( Point lowerLeftPoint_, Point upperRightPoint_ )
        : lowerLeftPoint ( lowerLeftPoint_ ),
          upperRightPoint ( upperRightPoint_ ) {

    }

    using AdditionalMethods::operator[];
    double& operator[] ( size_t pos ) {
        return at ( pos, lowerLeftPoint, upperRightPoint );
    }

    size_t size() const {
        return tuw::size ( lowerLeftPoint, upperRightPoint );
    }
};

// \brief Rectangle structure with additional dynamic vector (just to show an example)
struct RectWithVector : public AdditionalMethods<RectWithVector> {
    Rect rectangle;
    std::vector<double> doubleVector;

    RectWithVector() = default;

    RectWithVector ( Rect rectangle_, std::vector<double> doubleVector_ )
        : rectangle ( rectangle_ ),
          doubleVector ( doubleVector_ ) {

    }

    using AdditionalMethods::operator[];
    double& operator[] ( size_t pos ) {
        return at ( pos, rectangle, doubleVector );
    }

    size_t size() const {
        return tuw::size ( rectangle, doubleVector );
    }
};


}

#endif // STATE_MAP_BASE_TYPES_H