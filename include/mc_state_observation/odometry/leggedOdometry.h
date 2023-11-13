#pragma once

#include <mc_state_observation/measurements/measurementsTools.h>

#include <state-observation/dynamics-estimators/kinetics-observer.hpp>

namespace mc_state_observation::odometry
{

/**
 * Interface for the implementation of legged odometry. This odometry is based on the tracking of successive contacts
 * for the estimation of the pose of the floating base of the robot.

 * The tilt cannot be estimated from this method (but the yaw can), it has to be estimated beforehand by another
 * observer.
 * One can decide to perform flat or 6D odometry. The flat odometry considers that the robot walks on a flat
 * ground and corrects the estimated height accordingly, it is preferable in this use case.
 *
 * The odometry manager must be initialized once all the configuration parameters are retrieved using the init function,
 * and called on every iteration with \ref LeggedOdometryManager::run(const mc_control::MCController & ctl,
 * mc_rtc::Logger & logger, sva::PTransformd & pose, sva::MotionVecd & vel, sva::MotionVecd & acc).
 **/

///////////////////////////////////////////////////////////////////////
/// ------------------------------Contacts-----------------------------
///////////////////////////////////////////////////////////////////////
// Enhancement of the class ContactWithSensor with the reference of the contact in the world and the force measured by
// the associated sensor
class LoContactWithSensor : public measurements::ContactWithSensor
{
public:
  LoContactWithSensor(int id, std::string forceSensorName) : measurements::ContactWithSensor(id, forceSensorName) {}

  LoContactWithSensor(int id, const std::string & forceSensorName, const std::string & surfaceName)
  : measurements::ContactWithSensor(id, forceSensorName, surfaceName)
  {
  }

public:
  // reference of the contact in the world
  stateObservation::kine::Kinematics worldRefKine_;
  // indicates whether the contact can be used for the orientation odometry or not
  bool useForOrientation_ = false;
  // current estimation of the kinematics of the floating base in the world, obtained from the reference pose of the
  // contact in the world
  stateObservation::kine::Kinematics currentWorldFbPose_;
  // current estimation of the kinematics of the contact in the world
  stateObservation::kine::Kinematics currentWorldKine_;
};

/// @brief Structure that implements all the necessary functions to perform legged odometry.
/// @details Handles the odometry from the contacts detection to the final pose estimation of the floating base. Also
/// allows to compute the pose of an anchor frame linked to the robot.
struct LeggedOdometryManager
{
public:
  using ContactsManager = measurements::ContactsManager<LoContactWithSensor>;
  enum VelocityUpdate
  {
    noUpdate,
    finiteDiff,
    fromUpstream
  };

public:
  LeggedOdometryManager() {}

protected:
  ///////////////////////////////////////////////////////////////////////
  /// ------------------------Contacts Manager---------------------------
  ///////////////////////////////////////////////////////////////////////

  /// @brief Adaptation of the structure ContactsManager to the legged odometry, using personalized contacts classes.
  struct LeggedOdometryContactsManager : public ContactsManager
  {
  protected:
    // comparison function that sorts the contacts based on their measured force.
    struct sortByForce
    {
      inline bool operator()(const LoContactWithSensor & contact1, const LoContactWithSensor & contact2) const
      {
        return (contact1.forceNorm_ < contact2.forceNorm_);
      }
    };

  public:
    // list of contacts used for the orientation odometry. At most two contacts can be used for this estimation, and
    // contacts at hands are not considered. The contacts with the highest measured force are used.
    std::set<std::reference_wrapper<LoContactWithSensor>, sortByForce> oriOdometryContacts_;
  };

public:
  /// @brief Initializer for the odometry manager.
  /// @details Version for the contact detection using a thresholding on the contact force sensors measurements or by
  /// direct input from the solver.
  /// @param ctl Controller
  /// @param robotName Name of the robot
  /// @param odometryName Name of the odometry, used in logs and in the gui.
  /// @param odometryType Indicates if the desired odometry must be a flat or a 6D odometry.
  /// @param withYawEstimation Indicates if the orientation must be estimated by this odometry.
  /// @param velocityUpdate Indicates if we want to update the velocity and what method it must be updated with.
  /// @param verbose
  /// @param withModeSwitchInGui If true, adds the possiblity to switch between 6d and flat odometry from the gui.
  /// Should be set to false if this feature is implemented in the estimator using this library.
  void init(const mc_control::MCController & ctl,
            const std::string & robotName,
            const std::string & odometryName,
            measurements::OdometryType odometryType,
            const bool withYawEstimation,
            VelocityUpdate velocityUpdate,
            const bool verbose,
            const bool withModeSwitchInGui = false);

  /// @brief Initialization for a detection based on contact surfaces
  /// @param ctl Controller
  /// @param robotName name of the robot
  /// @param contactsDetection mean of detection for the contacts
  /// @param surfacesForContactDetection list of possible contact surfaces
  /// @param contactsSensorDisabledInit list of the force sensors that must be disabled on initialization.
  /// @param contactDetectionThreshold threshold on the measured force for the contact detection
  void initDetection(const mc_control::MCController & ctl,
                     const std::string & robotName,
                     const ContactsManager::ContactsDetection contactsDetection,
                     const std::vector<std::string> & surfacesForContactDetection,
                     const std::vector<std::string> & contactsSensorDisabledInit,
                     const double contactDetectionThreshold);

  /// @brief initialization for a detection based on a threshold on the measured contact forces or for contacts given by
  /// the controller
  /// @param ctl Controller
  /// @param robotName name of the robot
  /// @param contactsDetection mean of detection for the contacts
  /// @param contactsSensorDisabledInit list of the force sensors that must be disabled on initialization.
  /// @param contactDetectionThreshold threshold on the measured force for the contact detection
  /// @param forceSensorsToOmit list of force sensors that cannot be used for the contacts detection
  void initDetection(const mc_control::MCController & ctl,
                     const std::string & robotName,
                     const ContactsManager::ContactsDetection contactsDetection,
                     const std::vector<std::string> & contactsSensorDisabledInit,
                     const double contactDetectionThreshold,
                     const std::vector<std::string> & forceSensorsToOmit);

  /// @brief @copybrief run(const mc_control::MCController & ctl, mc_rtc::Logger &, sva::PTransformd &, const
  /// stateObservation::Matrix3 &). This version uses the tilt estimated by the upstream observers.
  /// @param ctl Controller
  /// @param pose The pose of the floating base in the world that we want to update
  /// @param logger Logger
  void run(const mc_control::MCController & ctl, mc_rtc::Logger & logger, sva::PTransformd & pose);

  /// @brief Core function runing the odometry.
  /// @param ctl Controller
  /// @param logger Logger
  /// @param pose The pose of the floating base in the world that we want to update
  /// @param tilt The floating base's tilt (only the yaw is estimated).
  void run(const mc_control::MCController & ctl,
           mc_rtc::Logger & logger,
           sva::PTransformd & pose,
           const stateObservation::Matrix3 & tilt);

  /// @brief @copybrief run(const mc_control::MCController &, mc_rtc::Logger &, sva::PTransformd &, const
  /// stateObservation::Matrix3 &, sva::MotionVecd &). This version uses the tilt estimated by the upstream observers.
  /// @param ctl Controller
  /// @param pose The pose of the floating base in the world that we want to update
  /// @param logger Logger
  /// @param vel The 6D velocity of the floating base in the world that we want to update. \velocityUpdate_ must be
  /// different from noUpdate, otherwise, it will not be updated.
  void run(const mc_control::MCController & ctl,
           mc_rtc::Logger & logger,
           sva::PTransformd & pose,
           sva::MotionVecd & vel);

  /// @brief Core function runing the odometry.
  /// @param ctl Controller
  /// @param logger Logger
  /// @param pose The pose of the floating base in the world that we want to update
  /// @param tilt The floating base's tilt (only the yaw is estimated).
  /// @param vel The 6D velocity of the floating base in the world that we want to update. \velocityUpdate_ must be
  /// different from noUpdate, otherwise, it will not be updated.
  void run(const mc_control::MCController & ctl,
           mc_rtc::Logger & logger,
           sva::PTransformd & pose,
           const stateObservation::Matrix3 & tilt,
           sva::MotionVecd & vel);

  /// @brief @copybrief run(const mc_control::MCController & ctl, mc_rtc::Logger &,  sva::PTransformd &, const
  /// stateObservation::Matrix3 &, sva::MotionVecd &, sva::MotionVecd &) run(const mc_control::MCController &,
  /// mc_rtc::Logger &, sva::PTransformd &, sva::MotionVecd &, sva::MotionVecd &, stateObservation::Matrix3). This
  /// version uses the tilt estimated by the upstream observers.
  /// @param ctl Controller
  /// @param logger Logger
  /// @param pose The pose of the floating base in the world that we want to update
  /// @param vel The 6D velocity of the floating base in the world that we want to update. \velocityUpdate_ must be
  /// different from noUpdate, otherwise, it will not be updated.
  /// @param acc The acceleration of the floating base in the world that we want to update
  void run(const mc_control::MCController & ctl,
           mc_rtc::Logger & logger,
           sva::PTransformd & pose,
           sva::MotionVecd & vel,
           sva::MotionVecd & acc);

  /// @brief Core function runing the odometry.
  /// @param ctl Controller
  /// @param logger Logger
  /// @param pose The pose of the floating base in the world that we want to update
  /// @param tilt The floating base's tilt (only the yaw is estimated).
  /// @param vel The 6D velocity of the floating base in the world that we want to update. \velocityUpdate_ must be
  /// different from noUpdate, otherwise, it will not be updated.
  /// @param acc The acceleration of the floating base in the world that we want to update
  void run(const mc_control::MCController & ctl,
           mc_rtc::Logger & logger,
           sva::PTransformd & pose,
           const stateObservation::Matrix3 & tilt,
           sva::MotionVecd & vel,
           sva::MotionVecd & acc);

  /// @brief Returns the pose of the odometry robot's anchor frame based on the current floating base and encoders.
  /// @details The anchor frame can can from 2 sources:
  /// - 1: contacts are detected and can be used to compute the anchor frame.
  /// - 2: no contact is detected, the robot is hanging. If we still need an anchor frame for the tilt estimation we
  /// arbitrarily use the frame of the bodySensor used by the estimator.
  /// @param ctl controller
  /// @param bodySensorName name of the body sensor.
  stateObservation::kine::Kinematics & getAnchorFramePose(const mc_control::MCController & ctl,
                                                          const std::string & bodySensorName);

  /// @brief Changes the type of the odometry
  /// @param newOdometryType The new type of odometry to use.
  void setOdometryType(const std::string & newOdometryType);

  /// @brief Changes the type of the odometry.
  /// @details Version meant to be called by the observer using the odometry.
  /// @param newOdometryType The new type of odometry to use.
  void setOdometryType(const measurements::OdometryType newOdometryType);

  /// @brief Getter for the odometry robot used for the estimation.
  mc_rbdyn::Robot & odometryRobot() { return odometryRobot_->robot("odometryRobot"); }

  /// @brief Getter for the contacts manager.
  LeggedOdometryContactsManager & contactsManager() { return contactsManager_; }

private:
  /// @brief Core function runing the odometry.
  /// @param ctl Controller
  /// @param logger Logger
  /// @param pose The pose of the floating base in the world that we want to update
  /// @param tilt The floating base's tilt, estimated either by the estimator using this library or by an upstream
  /// estimator.
  /// @param vel The 6D velocity of the floating base in the world that we want to update. \velocityUpdate_ must be
  /// different from noUpdate, otherwise, it will not be updated.
  /// @param acc The acceleration of the floating base in the world that we want to update
  void runPvt(const mc_control::MCController & ctl,
              mc_rtc::Logger & logger,
              sva::PTransformd & pose,
              const stateObservation::Matrix3 * tilt = nullptr,
              sva::MotionVecd * vel = nullptr,
              sva::MotionVecd * acc = nullptr);

  /// @brief Updates the floating base kinematics given as argument by the observer.
  /// @details Beware, only the pose is updated by the odometry, the 6D velocity (except if not updated by an upstream
  /// observer) and acceleration update only performs a transformation from the real robot to our newly estimated
  /// robot. If you want to update the acceleration of the floating base, you need to add an observer computing them
  /// beforehand.
  /// @param pose The pose of the floating base in the world that we want to update
  /// @param vel The 6D velocity of the floating base in the world that we want to update. \velocityUpdate_ must be
  /// different from noUpdate, otherwise, it will not be updated.
  /// @param acc The acceleration of the floating base in the world that we want to update. This acceleration must
  /// come from an upstream observer.
  void updateFbKinematicsPvt(sva::PTransformd & pose, sva::MotionVecd * vel = nullptr, sva::MotionVecd * acc = nullptr);

protected:
  /// @brief Updates the joints configuration of the odometry robot. Has to be called at the beginning of each
  /// iteration.
  /// @param ctl Controller
  void updateJointsConfiguration(const mc_control::MCController & ctl);

  /// @brief Updates the pose of the contacts and estimates the floating base from them.
  /// @param ctl Controller.
  /// @param logger Logger.
  /// @param tilt The floating base's tilt (only the yaw is estimated).
  /// @param vel The 6D velocity of the floating base in the world that we want to update. \velocityUpdate_ must be
  /// different from noUpdate, otherwise, it will not be updated.
  /// @param acc The floating base's tilt (only the yaw is estimated).
  void updateFbAndContacts(const mc_control::MCController & ctl,
                           mc_rtc::Logger & logger,
                           const stateObservation::Matrix3 & tilt,
                           sva::MotionVecd * vel = nullptr,
                           sva::MotionVecd * acc = nullptr);

  /// @brief If the contacts respect the conditions, computes the pose of the floating base for each set contact.
  /// @details Combines the reference pose of the contact in the world and the transformation from the contact to the
  /// frame.
  /// @param ctl Controller.
  /// @param posUpdatable Indicates if the position can be updated using contacts
  /// @param oriUpdatable Indicates if the orientation can be updated using contacts
  /// @param sumForces_position Sum of the measured force of all the contacts that will be used for the position
  /// estimation
  /// @param sumForces_orientation Sum of the measured force of all the contacts that will be used for the orientation
  /// estimation
  void getFbFromContacts(const mc_control::MCController & ctl,
                         bool & posUpdatable,
                         bool & oriUpdatable,
                         double & sumForces_position,
                         double & sumForces_orientation);

  /// @brief Updates the floating base kinematics given as argument by the observer.
  /// @details Must be called after \ref updateFbAndContacts(const mc_control::MCController & ctl, mc_rtc::Logger &,
  /// const stateObservation::Matrix3 &, sva::MotionVecd *, sva::MotionVecd *).
  /// Beware, only the pose is updated by the odometry. The 6D velocity (except if not updated by an upstream observer)
  /// is obtained using finite differences or by expressing the one given in input in the new robot frame. The
  /// acceleration can only be updated if estimated by an upstream estimator.
  /// @param ctl Controller.
  /// @param vel The 6D velocity of the floating base in the world that we want to update. \velocityUpdate_ must be
  /// different from noUpdate, otherwise, it will not be updated.
  /// @param acc The floating base's tilt (only the yaw is estimated).
  void updateOdometryRobot(const mc_control::MCController & ctl,
                           sva::MotionVecd * vel = nullptr,
                           sva::MotionVecd * acc = nullptr);

  /// @brief Computes the reference kinematics of the newly set contact in the world.
  /// @param contact The new contact
  /// @param measurementsRobot The robot containing the contact's force sensor
  void setNewContact(LoContactWithSensor & contact, const mc_rbdyn::Robot & measurementsRobot);

  /// @brief Computes the kinematics of the contact attached to the odometry robot in the world frame.
  /// @param contact Contact of which we want to compute the kinematics
  /// @param fs The force sensor associated to the contact
  /// @return stateObservation::kine::Kinematics &
  const stateObservation::kine::Kinematics & getCurrentContactKinematics(LoContactWithSensor & contact,
                                                                         const mc_rbdyn::ForceSensor & fs);
  /// @brief Select which contacts to use for the orientation odometry
  /// @details The two contacts with the highest measured force are selected. The contacts at hands are ignored because
  /// their orientation is less trustable.
  void selectForOrientationOdometry();

  /// @brief Add the log entries corresponding to the contact.
  /// @param logger
  /// @param contactName
  void addContactLogEntries(mc_rtc::Logger & logger, const LoContactWithSensor & contact);

  /// @brief Remove the log entries corresponding to the contact.
  /// @param logger
  /// @param contactName
  void removeContactLogEntries(mc_rtc::Logger & logger, const LoContactWithSensor & contact);

public:
  // Indicates if the mode of computation of the anchor frame changed. Might me needed by the estimator (ex;
  // TiltObserver)
  bool prevAnchorFromContacts_ = true;
  // Indicates if the desired odometry must be a flat or a 6D odometry.
  using OdometryType = measurements::OdometryType;
  measurements::OdometryType odometryType_;

  VelocityUpdate velocityUpdate_ = noUpdate;

protected:
  // Name of the odometry, used in logs and in the gui.
  std::string odometryName_;
  // Name of the robot
  std::string robotName_;

  // indicates whether we want to update the yaw using this method or not
  bool withYawEstimation_ = true;
  // tracked pose of the floating base
  sva::PTransformd fbPose_ = sva::PTransformd::Identity();

protected:
  // contacts manager used by this odometry manager
  LeggedOdometryContactsManager contactsManager_;
  // odometry robot that is updated by the legged odometry and can then update the real robot if required.
  std::shared_ptr<mc_rbdyn::Robots> odometryRobot_;
  // pose of the anchor frame of the robot in the world
  stateObservation::kine::Kinematics worldAnchorPose_;
};

} // namespace mc_state_observation::odometry
