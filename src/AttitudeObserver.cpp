#include <mc_control/MCController.h>
#include <mc_observers/ObserverMacros.h>
#include <AttitudeObserver.h>

namespace mc_state_observation
{

namespace so = stateObservation;

AttitudeObserver::AttitudeObserver(const std::string & type, double dt)
: mc_observers::Observer(type, dt), filter_(stateSize_, measurementSize_, inputSize_, false),
  q_(so::Matrix::Identity(stateSize_, stateSize_) * m_stateCov),
  r_(so::Matrix::Identity(measurementSize_, measurementSize_) * m_acceleroCovariance), uk_(inputSize_), xk_(stateSize_)
{
  q_(9, 9) = q_(10, 10) = q_(11, 11) = m_orientationAccCov;
  q_(6, 6) = q_(7, 7) = q_(8, 8) = m_linearAccCov;
  r_(3, 3) = r_(4, 4) = r_(5, 5) = m_gyroCovariance;

  /// initialization of the extended Kalman filter
  imuFunctor_.setSamplingPeriod(dt_);
  filter_.setFunctor(&imuFunctor_);

  filter_.setQ(q_);
  filter_.setR(r_);
  xk_.setZero();
  uk_.setZero();
  filter_.setState(xk_, 0);
  filter_.setStateCovariance(so::Matrix::Identity(stateSize_, stateSize_) * m_stateInitCov);

  lastStateInitCovariance_ = m_stateInitCov;

  Kpt_ << -20, 0, 0, 0, -20, 0, 0, 0, -20;
  Kdt_ << -10, 0, 0, 0, -10, 0, 0, 0, -10;
  Kpo_ << -0.0, 0, 0, 0, -0.0, 0, 0, 0, -10;
  Kdo_ << -0.0, 0, 0, 0, -0.0, 0, 0, 0, -10;
}

void AttitudeObserver::configure(const mc_control::MCController & ctl, const mc_rtc::Configuration & config)
{
  config("compensateMode", m_compensateMode);
  config("offset", m_offset);
  config("acc_cov", m_acceleroCovariance);
  config("gyr_cov", m_gyroCovariance);
  config("ori_acc_cov", m_orientationAccCov);
  config("lin_acc_cov", m_linearAccCov);
  config("state_cov", m_stateCov);
  config("state_init_cov", m_stateInitCov);
  robot_ = config("robot", ctl.robot().name());
  imuSensor_ = config("imuSensor", ctl.robot().bodySensor().name());
  if(config.has("updateSensor"))
  {
    updateSensor_ = static_cast<std::string>(config("updateSensor"));
  }
  else
  {
    updateSensor_ = imuSensor_;
  }
  datastoreName_ = config("datastoreName", name());
}

void AttitudeObserver::reset(const mc_control::MCController & /*ctl*/) {}

bool AttitudeObserver::run(const mc_control::MCController & ctl)
{
  using indexes = so::kine::indexes<so::kine::rotationVector>;

  imuFunctor_.setSamplingPeriod(dt_);

  q_.noalias() = so::Matrix::Identity(stateSize_, stateSize_) * m_stateCov;
  r_.noalias() = so::Matrix::Identity(measurementSize_, measurementSize_) * m_acceleroCovariance;
  q_(9, 9) = q_(10, 10) = q_(11, 11) = m_orientationAccCov;
  q_(6, 6) = q_(7, 7) = q_(8, 8) = m_linearAccCov;
  r_(3, 3) = r_(4, 4) = r_(5, 5) = m_gyroCovariance;

  filter_.setQ(q_);
  filter_.setR(r_);

  if(lastStateInitCovariance_ != m_stateInitCov) /// if the value of the state Init Covariance has changed
  {
    filter_.setStateCovariance(so::Matrix::Identity(stateSize_, stateSize_) * m_stateInitCov);
    lastStateInitCovariance_ = m_stateInitCov;
  }

  const mc_rbdyn::BodySensor & imu = ctl.robot(robot_).bodySensor(imuSensor_);
  const Eigen::Vector3d & accIn = imu.linearAcceleration();
  const Eigen::Vector3d & rateIn = imu.angularVelocity();

  // processing
  so::Vector6 measurement;
  if(m_compensateMode)
  {
    if(ctl.datastore().has(datastoreName_ + "::accRef"))
    {
      const Eigen::Vector3d & accRef = ctl.datastore().get<Eigen::Vector3d>(datastoreName_ + "::accRef");
      measurement.segment<3>(0) = accIn - accRef;
    }
    else
    {
      // XXX should this be zero
      measurement[0] = measurement[1] = measurement[2] = 0;
      return false;
    }
  }
  else
  {
    measurement.segment<3>(0) = accIn;
  }
  measurement.segment<3>(3) = rateIn;

  auto time = filter_.getCurrentTime();

  /// damped linear and angular spring
  uk_.head<3>() = Kpt_ * xk_.segment<3>(indexes::pos) + Kdt_ * xk_.segment<3>(indexes::linVel);
  uk_.tail<3>() = Kpo_ * xk_.segment<3>(indexes::ori) + Kdo_ * xk_.segment<3>(indexes::angVel);

  filter_.setInput(uk_, time);

  filter_.setMeasurement(measurement, time + 1);

  /// set the derivation step for the finite difference method
  so::Vector dx = filter_.stateVectorConstant(1) * 1e-8;

  so::Matrix a = filter_.getAMatrixFD(dx);
  so::Matrix c = filter_.getCMatrixFD(dx);

  filter_.setA(a);
  filter_.setC(c);

  /// get the estimation and give it to the array
  xk_ = filter_.getEstimatedState(time + 1);

  so::Vector3 orientation(xk_.segment<3>(indexes::ori));
  so::Matrix3 mat(m_offset * so::kine::rotationVectorToRotationMatrix(orientation));
  so::Vector3 euler(so::kine::rotationMatrixToRollPitchYaw(mat));

  // result
  m_orientation = mat;
  m_rpy = euler;

  return true;
}

void AttitudeObserver::update(mc_control::MCController & ctl)
{
  auto & sensor = ctl.robot(robot_).bodySensor(updateSensor_);
  sensor.orientation(Eigen::Quaterniond{m_orientation});
}

void AttitudeObserver::addToLogger(const mc_control::MCController &,
                                   mc_rtc::Logger & logger,
                                   const std::string & category)
{
  logger.addLogEntry(category + "_orientation", [this]() -> sva::PTransformd { return m_orientation.inverse(); });
  logger.addLogEntry(category + "_covariance_state", [this]() { return m_stateCov; });
  logger.addLogEntry(category + "_covariance_ori_acc", [this]() { return m_orientationAccCov; });
  logger.addLogEntry(category + "_covariance_acc", [this]() { return m_acceleroCovariance; });
  logger.addLogEntry(category + "_covariance_gyr", [this]() { return m_gyroCovariance; });
}

void AttitudeObserver::removeFromLogger(mc_rtc::Logger & logger, const std::string & category)
{
  logger.removeLogEntry(category + "_covariance_state");
  logger.removeLogEntry(category + "_covariance_ori_acc");
  logger.removeLogEntry(category + "_covariance_acc");
  logger.removeLogEntry(category + "_covariance_gyr");
}

void AttitudeObserver::addToGUI(const mc_control::MCController & ctl,
                                mc_rtc::gui::StateBuilder & gui,
                                const std::vector<std::string> & category)
{
  Observer::addToGUI(ctl, gui, category);
}

} // namespace mc_state_observation
EXPORT_OBSERVER_MODULE("Attitude", mc_state_observation::AttitudeObserver)
