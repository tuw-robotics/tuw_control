/***************************************************************************
 *   Software License Agreement (BSD License)                              *
 *   Copyright (C) 2016 by Horatiu George ran <todorangrg@gmail.com>   *
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

#include <tuw_control/utils.h>
#include <tuw_control/state/state_array.hpp>
#include <tuw_control/state/state_nested_set.h>
#include <tuw_control/state/state_nested_vector.h>
#include <tuw_control/state/state_nested_array.h>

#include <tuw_control/leaf_access_tree/leaf_access_tree.hpp>

#include <iostream>

using namespace tuw;

///@brief Enum defining the semantics of the Iws wheel state.
enum class StateWheel
{
  Steer,
  Revol,
  ENUM_SIZE
};
///@brief OneWheelType with access via varialbe and enums
class OneWheelType : public StateArrayScoped<StateWheel>
{
public:
  using StateArrayScoped::StateArrayScoped;
  OneWheelType(double steering, double revolute) : StateArrayScoped<StateWheel>()
  {
    steer() = steering, revol() = revolute;
  }
  const double &steer() const
  {
    return values_[0];
  }
  const double &revol() const
  {
    return values_[1];
  }
  double &steer()
  {
    return values_[0];
  }
  double &revol()
  {
    return values_[1];
  }
};

class StateWhInpType : public StateNestedVector<OneWheelType>
{
public:
  using StateNestedVector::StateNestedVector;
  const OneWheelType &wheel(size_t i) const
  {
    return *states_[i];
  }
  OneWheelType &wheel(size_t i)
  {
    return *states_[i];
  }
};

using StateWhInpPtrType = std::shared_ptr<StateWhInpType>;

int main(int argc, char **argv)
{
  std::cout << "iws utils test" << std::endl;
  StateWhInpPtrType tuw_i2ws_wheel_state = std::shared_ptr<StateWhInpType>(new StateWhInpType);
  tuw_i2ws_wheel_state->resize(2);
  tuw_i2ws_wheel_state->wheel(0).steer() = 10;
  tuw_i2ws_wheel_state->wheel(0).revol() = 11;
  tuw_i2ws_wheel_state->wheel(1).steer() = 12;
  tuw_i2ws_wheel_state->wheel(1).revol() = 13;
  /// printing the state
  std::cout << "tuw      : " << *tuw_i2ws_wheel_state << std::endl;

  /// convert to stl vector
  std::vector<double> values;
  tuw_i2ws_wheel_state->toSTLVec(values);

  /// printing the classical struct
  std::cout << "tuw      : [" << *tuw_i2ws_wheel_state->state(0) << ", " << *tuw_i2ws_wheel_state->state(1) << "]"
            << std::endl;

  /// accessing varibles
  std::cout << "tuw      tuw_i2ws_wheel_state->stateScoped ( 1 )->value<StateWheel::Steer>() = "
            << tuw_i2ws_wheel_state->stateScoped(1)->value<StateWheel::Steer>() << std::endl;

  /// accessing varibles
  std::cout << "tuw      ( *tuw_i2ws_wheel_state ) [1].steer()     = " << (*tuw_i2ws_wheel_state)[1].steer()
            << std::endl;
  /// accessing varibles
  std::cout << "tuw      tuw_i2ws_wheel_state->wheel ( 1 ).steer() = " << tuw_i2ws_wheel_state->wheel(1).steer()
            << std::endl;

  using LeafArr3 = LeafAccessTreeBase<double, double, 3>;
  using LeafVec = LeafAccessTreeBase<double, double, -1>;

  LeafAccessTreeBase<double, std::tuple<LeafArr3, LeafVec>> state0;
  LeafAccessTreeBaseVirt &state0Virt = state0;
  //     LeafAccessTreeBaseVirt& subState0Virt = state0.sub(0);
  //     LeafAccessTreeBaseVirt& subState1Virt = state0.sub<0>();
  Eigen::VectorXd xx;
  xx.resize(5);
  xx(0) = 0;
  xx(1) = 1;
  xx(2) = 2;
  xx(3) = 3;
  xx(4) = 4;

  std::cout << std::endl;
  std::cout << xx.block(2, 0, 2, 1) << std::endl
            << std::endl;

  Eigen::VectorXd yy(3);
  yy(0) = 9;
  yy(1) = 8;
  yy(2) = 7;

  // xx.resize(6);
  auto _blockRef = xx.block(2, 0, 2, 1);
  //_blockRef.resize(3);
  //_blockRef = yy;

  // xx.block(2,0,2,1) = yy;
  std::cout << _blockRef << std::endl
            << std::endl;
  std::cout << xx << std::endl
            << std::endl;

  typedef Eigen::Map<Eigen::VectorXd> MapType;

  MapType xMap0(&xx(0), xx.rows());
  MapType xMap2(&xx(2), xx.rows() - 3);
  MapType xMap4(&xx(4), xx.rows() - 4);

  std::cout << "xx=" << std::endl
            << xx << std::endl
            << std::endl;
  std::cout << "xMap0=" << std::endl
            << xMap0 << std::endl
            << std::endl;
  std::cout << "xMap2=" << std::endl
            << xMap2 << std::endl
            << std::endl;
  std::cout << "xMap4=" << std::endl
            << xMap4 << std::endl
            << std::endl;

  xx.conservativeResize(8);
  double *p;

  new (&xMap2) MapType(&xx(2), xMap2.outerStride() + 3);
  new (&xMap4) MapType(&xMap2(2), xMap4.outerStride());
  double *p2;
  // xMap2.cast_to_pointer_type(p2);
  // xMap4.cast_to_pointer_type(p); p =&xx(2)+7;+xMap2.outerStride();
  // xMap4.

  std::cout << "xx=" << std::endl
            << xx << std::endl
            << std::endl;
  std::cout << "xMap0=" << std::endl
            << xMap0 << std::endl
            << std::endl;
  std::cout << "xMap2=" << std::endl
            << xMap2 << std::endl
            << std::endl;
  std::cout << "xMap4=" << std::endl
            << xMap4 << std::endl
            << std::endl;
  //     x.resize(5);
  //     xMap.cast_to_pointer_type(p);
  //     p = &x(0);

  //     std::cout<<x<<std::endl<<std::endl;

  //     MatrixType m1(n_dims), m2(n_dims);
  //     m1.setRandom();
  //     m2.setRandom();
  //     float *p = &m2(0);  // get the address storing the data for m2
  //     MapType m2map(p,m2.size());   // m2map shares data with m2
  //     MapTypeConst m2mapconst(p,m2.size());  // a read-only accessor for m2
  //     cout << "m1: " << m1 << endl;
  //     cout << "m2: " << m2 << endl;
  //     cout << "Squared euclidean distance: " << (m1-m2).squaredNorm() << endl;
  //     cout << "Squared euclidean distance, using map: " <<
  //     (m1-m2map).squaredNorm() << endl;
  //     m2map(3) = 7;   // this will change m2, since they share the same array
  //     cout << "Updated m2: " << m2 << endl;
  //     cout << "m2 coefficient 2, constant accessor: " << m2mapconst(2) << endl;
  //     /* m2mapconst(2) = 5; */   // this yields a compile-time error
}
