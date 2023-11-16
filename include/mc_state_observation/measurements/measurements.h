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

// map allowing to get the VelocityUpdate value associated to the given string
inline std::unordered_map<std::string, OdometryType> mapStrOdometryType_ = {{"Odometry6d", OdometryType::Odometry6d},
                                                                            {"Flat", OdometryType::Flat},
                                                                            {"None", OdometryType::None}};

/// @brief Gives the given OdometryType the value corresponding to the given string.
/// @details Allows to set the odometry type directly from a string, most likely obtained from a configuration file.
/// @param str The string naming the desired odometry type
/// @param odometryType The OdometryType we want to modify
inline void stringToOdometryType(const std::string & str, OdometryType & odometryType)
{
  odometryType = mapStrOdometryType_.at(str);
}
// IMUs can be handled using only a vector containing the IMU objects.
typedef std::vector<IMU> ImuList;

} // namespace mc_state_observation::measurements
