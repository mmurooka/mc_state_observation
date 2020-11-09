#pragma once

#include <mc_observers/BodySensorObserver.h>

#include <state-observation/dynamical-system/imu-dynamical-system.hpp>
#include <state-observation/observer/extended-kalman-filter.hpp>

namespace mc_state_observation
{

struct AttitudeObserver : public mc_observers::Observer
{
  AttitudeObserver(const std::string & type, double dt);

  void configure(const mc_control::MCController & ctl, const mc_rtc::Configuration &) override;

  void reset(const mc_control::MCController & ctl) override;

  bool run(const mc_control::MCController & ctl) override;

  void update(mc_control::MCController & ctl) override;

protected:
  /*! \brief Add observer from logger
   *
   * @param category Category in which to log this observer
   */
  void addToLogger(const mc_control::MCController &, mc_rtc::Logger &, const std::string & category) override;

  /*! \brief Remove observer from logger
   *
   * @param category Category in which this observer entries are logged
   */
  void removeFromLogger(mc_rtc::Logger &, const std::string & category) override;

  /*! \brief Add observer information the GUI.
   *
   * @param category Category in which to add this observer
   */
  void addToGUI(const mc_control::MCController &,
                mc_rtc::gui::StateBuilder &,
                const std::vector<std::string> & /* category */) override;

protected:
  /// @{
  std::string robot_ = ""; ///< Name of robot to which the IMU sensor belongs
  std::string imuSensor_ = ""; ///< Name of the sensor used for IMU readings
  std::string updateSensor_ = ""; ///< Name of the sensor to update with the results (default: imuSensor_)
  std::string datastoreName_ = ""; ///< Name on the datastore (default name())
  bool m_compensateMode = false;
  double m_acceleroCovariance = 0.003;
  double m_gyroCovariance = 1e-10;
  double m_orientationAccCov = 0.003;
  double m_linearAccCov = 1e-13;
  double m_stateCov = 3e-14;
  double m_stateInitCov = 1e-8;
  Eigen::Matrix3d m_offset = Eigen::Matrix3d::Identity();
  /// @}

  /// Sizes of the states for the state, the measurement, and the input vector
  const unsigned stateSize_ = 18;
  const unsigned measurementSize_ = 6;
  const unsigned inputSize_ = 6;

  double lastStateInitCovariance_;

  /// initialization of the extended Kalman filter
  stateObservation::ExtendedKalmanFilter filter_;

  /// initalization of the functor
  stateObservation::IMUDynamicalSystem imuFunctor_;

  stateObservation::Matrix q_;
  stateObservation::Matrix r_;

  stateObservation::Vector uk_;
  stateObservation::Vector xk_;

  stateObservation::Matrix3 Kpt_, Kdt_;
  stateObservation::Matrix3 Kpo_, Kdo_;

  Eigen::Matrix3d m_orientation = Eigen::Matrix3d::Identity(); ///< Result
  Eigen::Vector3d m_rpy = Eigen::Vector3d::Zero(); ///< Result
};

} // namespace mc_state_observation
