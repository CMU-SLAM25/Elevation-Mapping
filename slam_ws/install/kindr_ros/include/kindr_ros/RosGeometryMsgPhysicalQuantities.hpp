/*
 * Copyright (c) 2014, Peter Fankhauser, Christian Gehring, Hannes Sommer, Paul Furgale, Remo Diethelm
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Autonomous Systems Lab, ETH Zurich nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL Peter Fankhauser, Christian Gehring, Hannes Sommer, Paul Furgale,
 * Remo Diethelm BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
*/

#ifndef KINDR_ROS__ROSGEOMETRYMSGPHYSICALQUANTITIES_HPP_
#define KINDR_ROS__ROSGEOMETRYMSGPHYSICALQUANTITIES_HPP_


// kindr
#include <kindr/Core>

// ros
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point32.hpp>


namespace kindr_ros
{


template<enum kindr::PhysicalType PhysicalType_, typename PrimType_>
inline static void convertFromRosGeometryMsg(
  const geometry_msgs::msg::Vector3 & geometryVector3Msg,
  kindr::Vector<PhysicalType_, PrimType_, 3> & vector)
{
  vector.x() = static_cast<PrimType_>(geometryVector3Msg.x);
  vector.y() = static_cast<PrimType_>(geometryVector3Msg.y);
  vector.z() = static_cast<PrimType_>(geometryVector3Msg.z);
}

template<enum kindr::PhysicalType PhysicalType_, typename PrimType_>
inline static void convertToRosGeometryMsg(
  const kindr::Vector<PhysicalType_, PrimType_, 3> & vector,
  geometry_msgs::msg::Vector3 & geometryVector3Msg)
{
  geometryVector3Msg.x = static_cast<double>(vector.x());
  geometryVector3Msg.y = static_cast<double>(vector.y());
  geometryVector3Msg.z = static_cast<double>(vector.z());
}

template<typename PrimType_>
inline static void convertFromRosGeometryMsg(
  const geometry_msgs::msg::Point32 & geometryPoint32Msg,
  kindr::Position<PrimType_, 3> & position)
{
  position.x() = static_cast<PrimType_>(geometryPoint32Msg.x);
  position.y() = static_cast<PrimType_>(geometryPoint32Msg.y);
  position.z() = static_cast<PrimType_>(geometryPoint32Msg.z);
}

template<typename PrimType_>
inline static void convertToRosGeometryMsg(
  const kindr::Position<PrimType_, 3> & position,
  geometry_msgs::msg::Point32 & geometryPoint32Msg)
{
  geometryPoint32Msg.x = static_cast<float>(position.x());
  geometryPoint32Msg.y = static_cast<float>(position.y());
  geometryPoint32Msg.z = static_cast<float>(position.z());
}

template<typename PrimType_>
inline static void convertFromRosGeometryMsg(
  const geometry_msgs::msg::Point & geometryPointMsg,
  kindr::Position<PrimType_, 3> & position)
{
  position.x() = static_cast<PrimType_>(geometryPointMsg.x);
  position.y() = static_cast<PrimType_>(geometryPointMsg.y);
  position.z() = static_cast<PrimType_>(geometryPointMsg.z);
}

template<typename PrimType_>
inline static void convertToRosGeometryMsg(
  const kindr::Position<PrimType_, 3> & position,
  geometry_msgs::msg::Point & geometryPointMsg)
{
  geometryPointMsg.x = static_cast<double>(position.x());
  geometryPointMsg.y = static_cast<double>(position.y());
  geometryPointMsg.z = static_cast<double>(position.z());
}


}  // namespace kindr_ros

#endif  // KINDR_ROS__ROSGEOMETRYMSGPHYSICALQUANTITIES_HPP_
