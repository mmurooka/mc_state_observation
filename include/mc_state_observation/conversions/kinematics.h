#include <mc_rtc/log/Logger.h>
#include <SpaceVecAlg/SpaceVecAlg>
#include <state-observation/tools/rigid-body-kinematics.hpp>

/**
 * Conversion framework between the sva representation of kinematics (PTransform for pose, MotionVec for velocities and
 * accelerations) and the one used in rigid-body-kinematics (Kinematics).
 * The Kinematics object allows a user-friendly representation of the kinematics of a frame within
 * another and offers all the necessary operations on kinematics : composition, inversion, etc.
 *
 * Equivalences : PTransform = {position + Orientation}
 *                 MotionVec = {linVel + angVel} or {linAcc + angAcc}
 **/

namespace mc_state_observation::kinematicsTools
{

///////////////////////////////////////////////////////////////////////
/// -------------------Sva to Kinematics conversion--------------------
///////////////////////////////////////////////////////////////////////

/// @brief Creates a Kinematics object from a PTransformd object that contains the position and the orientation of a
/// frame within another.
/// @param pTransform The pose of the frame within the other frame, stored as a sva PTransform object.
/// @param zeroKine Defines the kinematic variables to initialize to zero. For example some compositions need to set a
/// zero velocity to obtain the one of the resulting kinematic.
stateObservation::kine::Kinematics poseFromSva(const sva::PTransformd & pTransform,
                                               stateObservation::kine::Kinematics::Flags::Byte zeroKine);

/// @brief Creates a Kinematics object from a PTransformd object that contains the position and the orientation of a
/// frame A within another frame B, and from a MotionVecd object that contains the associated velocities.
/// @details The motion vectors given by mc_rtc can be expressed in the global frame B (example:
/// rbd::MultiBodyConfig::bodyVelW), but some are expressed in the local frame A (example:
/// rbd::MultiBodyConfig::bodyAccB), this information must be passed as a parameter.
/// @param pTransform The pose of the frame within the other frame, stored as a sva PTransform object.
/// @param vel The velocity of the frame A inside B.
/// @param velIsGlobal If true, the velocity vectors are expressed in the global frame (B), if false, they are expressed
/// in the local frame (A).
stateObservation::kine::Kinematics poseAndVelFromSva(const sva::PTransformd & pTransform,
                                                     const sva::MotionVecd & vel,
                                                     bool velIsGlobal = true);

/// @brief Creates a Kinematics object from a PTransformd object that contains the position and the orientation of a
/// frame A within another frame B, and from two MotionVecd object that contain the associated velocities and
/// accelerations.
/// @details The motion vectors given by mc_rtc can be expressed in the global frame B (example:
/// rbd::MultiBodyConfig::bodyVelW), but some are expressed in the local frame A (example:
/// rbd::MultiBodyConfig::bodyAccB), this information must be passed as a parameter.
/// @param pTransform The pose of the frame within the other frame, stored as a sva PTransform object.
/// @param vel The linear and angular velocities of the frame A inside B.
/// @param acc The linear and angular accelerations of the frame A inside B.
/// @param velIsGlobal If true, the velocity vectors are expressed in the global frame (B), if false, they are expressed
/// in the local frame (A).
/// @param accIsGlobal If true, the acceleration vectors are expressed in the global frame (B), if false, they are
/// expressed in the local frame (A).
stateObservation::kine::Kinematics kinematicsFromSva(const sva::PTransformd & pTransform,
                                                     const sva::MotionVecd & vel,
                                                     const sva::MotionVecd & acc,
                                                     bool velIsGlobal = true,
                                                     bool accIsGlobal = true);

/// @brief Adds the velocity variables of a frame A within a frame B contained in a MotionVectord object
/// to the corresponding Kinematics object.
/// @details The motion vectors given by mc_rtc can be expressed in the global frame B (example:
/// rbd::MultiBodyConfig::bodyVelW), but some are expressed in the local frame A (example:
/// rbd::MultiBodyConfig::bodyAccB), this information must be passed as a parameter.
/// @param kine The Kinematics object to enhance with the velocity / acceleration variables.
/// @param vel The linear and angular velocities of the frame A inside B.
/// @param velIsGlobal If true, the velocity vectors are expressed in the global frame (B), if false, they are expressed
/// in the local frame (A).
stateObservation::kine::Kinematics & addVelocities(stateObservation::kine::Kinematics & kine,
                                                   const sva::MotionVecd & vel,
                                                   bool velIsGlobal = true);
/// @brief Adds the velocity variables of a frame A within a frame B contained in a MotionVectord object
/// to the corresponding Kinematics object.
/// @details The motion vectors given by mc_rtc can be expressed in the global frame B (example:
/// rbd::MultiBodyConfig::bodyVelW), but some are expressed in the local frame A (example:
/// rbd::MultiBodyConfig::bodyAccB), this information must be passed as a parameter.
/// @param kine The Kinematics object to enhance with the velocity / acceleration variables.
/// @param vel The linear and angular velocities of the frame A inside B.
/// @param acc The linear and angular accelerations of the frame A inside B.
/// @param velIsGlobal If true, the velocity vectors are expressed in the global frame (B), if false, they are expressed
/// in the local frame (A).
/// @param accIsGlobal If true, the acceleration vectors are expressed in the global frame (B), if false, they are
/// expressed in the local frame (A).
stateObservation::kine::Kinematics & addVelsAndAccs(stateObservation::kine::Kinematics & kine,
                                                    const sva::MotionVecd & vel,
                                                    const sva::MotionVecd & acc,
                                                    bool velIsGlobal = true,
                                                    bool accIsGlobal = true);

///////////////////////////////////////////////////////////////////////
/// -------------------Kinematics to SVA conversion--------------------
///////////////////////////////////////////////////////////////////////

sva::PTransformd pTransformFromKinematics(const stateObservation::kine::Kinematics & kine);

///////////////////////////////////////////////////////////////////////
/// -------------------------Logging functions-------------------------
///////////////////////////////////////////////////////////////////////

void addToLogger(const stateObservation::kine::Kinematics & kine, mc_rtc::Logger & logger, const std::string & prefix);

void removeFromLogger(mc_rtc::Logger & logger, const std::string & prefix);

} // namespace mc_state_observation::kinematicsTools