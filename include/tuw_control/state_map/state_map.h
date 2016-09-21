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

#ifndef STATE_MAP_H
#define STATE_MAP_H


#include <iostream>
#include <stdexcept>
#include <vector>

namespace tuw {

/// \brief    Allows access to doubles, and the number of doubles in an array like object in a unified way.
/// \details  The array like object must implement operator[] and the size() method.
/// \ElementT Type of the array like object.
template<typename ElementT>
struct ElementTraits {
    static double& at ( ElementT& element, size_t pos ) {
        return element[pos];
    }

    static size_t size ( const ElementT& element ) {
        return element.size();
    }
};

/// \brief Specialization for doubles.
template<>
struct ElementTraits<double> {
    static double& at ( double& element, size_t pos ) {
        return element;
    }

    static size_t size ( const double& element ) {
        return 1;
    }
};

///////////////////////////////////////////////////////////////////////////////

/// \brief Ends the recursion of the vardiatic template - for description see below.
template<typename LastElementT>
double& at ( size_t pos, LastElementT& lastElement ) {
    auto sizeOfFirstElement = ElementTraits<LastElementT>::size ( lastElement );

    if ( pos < sizeOfFirstElement )
        return ElementTraits<LastElementT>::at ( lastElement, pos );
    else
        throw new std::out_of_range ( "pos is out of range" );
}

/// \brief Returns a reference to the n-th double of an array like object which is theoretically
///        concatenated from the given list of elements of array like objects.
template<typename FirstElementT, typename ...RemainingElementTs>
double& at ( size_t pos, FirstElementT& firstElement, RemainingElementTs&... remainingElements ) {
    auto sizeOfFirstElement = ElementTraits<FirstElementT>::size ( firstElement );

    if ( pos < sizeOfFirstElement )
        return ElementTraits<FirstElementT>::at ( firstElement, pos );
    else
        return at<RemainingElementTs...> ( pos - sizeOfFirstElement, remainingElements... );
}

///////////////////////////////////////////////////////////////////////////////

/// \brief Ends the recursion of the vardiatic template - for description see below.
template<typename LastElementT>
size_t size ( const LastElementT& lastElement ) {
    return ElementTraits<LastElementT>::size ( lastElement );
}

/// \brief Returns the size of an array like object which is theoretically
///        concatenated from the given list of elements of array like objects.
template<typename FirstElementT, typename ...RemainingElementTs>
size_t size ( const FirstElementT& firstElement, const RemainingElementTs&... remainingElements ) {
    return ElementTraits<FirstElementT>::size ( firstElement ) + size<RemainingElementTs...> ( remainingElements... );
}

///////////////////////////////////////////////////////////////////////////////

template<typename StructT>
class AdditionalMethods {
public:
    const double& operator[] ( size_t pos ) const {
        return const_cast<StructT*> ( static_cast<const StructT*> ( this ) )->operator[] ( pos );
    }

    bool operator== ( const StructT& other ) const {
        size_t thisSize = static_cast<const StructT*> ( this )->size();
        size_t otherSize = other.size();

        if ( thisSize != otherSize )
            return false;

        for ( size_t i = 0; i < thisSize; i++ )
            if ( this->operator[] ( i ) != other[i] )
                return false;

        return true;
    }

    bool operator!= ( const StructT& other ) const {
        return !this->operator== ( other );
    }

    //other methods, like to / from Eigen-vector, ...
};

}

#endif // STATE_MAP_H