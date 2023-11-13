/**
 * \file      measurementTools.h
 * \author    Arnaud Demont
 * \date       2023
 * \brief      Library for an easened handling of contacts and sensors in general.
 *
 * \details
 *
 *
 */

#pragma once

#include "mc_state_observation/measurements/IMU.h"
#include <vector>
namespace mc_state_observation::measurements
{

// allowed odometry types
enum class OdometryType
{
  Odometry6d,
  Flat,
  None
};

// IMUs can be handled using only a vector containing the IMU objects.
typedef std::vector<IMU> ImuList;

} // namespace mc_state_observation::measurements
