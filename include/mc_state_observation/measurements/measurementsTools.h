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

#include <mc_control/MCController.h>
#include <mc_observers/Observer.h>
#include <mc_rbdyn/Robot.h>
#include <mc_rtc/log/Logger.h>
#include <mc_rtc/logging.h>

#include <mc_state_observation/measurements/ContactWithSensor.h>
#include <mc_state_observation/measurements/IMU.h>
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

/// @brief Structure that implements all the necessary functions to manage the map of contacts. Handles their detection
/// and updates the list of the detected contacts, newly removed contacts, etc., to apply the appropriate functions on
/// them.
/// @details The template allows to define other kinds of contacts and thus add custom parameters to them.
/// @tparam ContactWithSensorT Contacts associated to a sensor.
template<typename ContactWithSensorT>
struct ContactsManager
{
public:
  enum ContactsDetection
  {
    Solver,
    Surfaces,
    Sensors,
    Undefined
  };
  typedef std::set<int> ContactsSet;

public:
  ContactsManager()
  {
    static_assert(std::is_base_of_v<ContactWithSensor, ContactWithSensorT>,
                  "The template class for the contacts with sensors must inherit from the ContactWithSensor class");
  }

protected:
  /// @brief Inserts a contact to the map of contacts.
  /// @details Version for contacts that are associated to both a force sensor and a contact surface. The contact will
  /// be named with the name of the force sensor.
  /// @param forceSensorName The name of the force sensor.
  /// @param surface The name of the surface that will be used also to name the contact.
  /// @return ContactWithSensorT &
  inline ContactWithSensorT & addContactToManager(const std::string & forceSensorName, const std::string surface)
  {
    ContactWithSensorT contact = ContactWithSensorT(idx_, forceSensorName, surface);

    listContacts_.insert(std::make_pair(forceSensorName, contact));
    insertOrder_.push_back(forceSensorName);
    idx_++;

    return listContacts_.at(forceSensorName);
  }
  /// @brief Insert a contact to the map of contacts.
  /// @details Version for contacts that are associated to a force sensor but to no surface.
  /// @param name The name of the contact (= name of the sensor)
  /// @return ContactWithSensorT &
  inline ContactWithSensorT & addContactToManager(const std::string & forceSensorName)
  {
    ContactWithSensorT contact = ContactWithSensorT(idx_, forceSensorName);

    listContacts_.insert(std::make_pair(forceSensorName, contact));
    insertOrder_.push_back(forceSensorName);
    idx_++;

    return listContacts_.at(forceSensorName);
  }

  // pointer to the function that will be used for the contact detection depending on the chosen method
  void (ContactsManager::*contactsFinder_)(const mc_control::MCController &, const std::string &) = 0;

  /// @brief Updates the list @contactsFound_ of currently set contacts directly from the controller.
  /// @details Called by \ref findContacts(const mc_control::MCController & ctl) if @contactsDetection_ is equal to
  /// "Solver". The contacts are given by the controller directly (then thresholded based on the measured force).
  void findContactsFromSolver(const mc_control::MCController & ctl, const std::string & robotName);
  /// @brief Updates the list @contactsFound_ of currently set contacts from the surfaces given by the user.
  /// @details Called by \ref findContacts(const mc_control::MCController & ctl) if @contactsDetection_ is equal to
  /// "Surfaces". The contacts are obtained by thresholded based the force measured by the associated force sensor).
  void findContactsFromSurfaces(const mc_control::MCController & ctl, const std::string & robotName);
  /// @brief Updates the list @contactsFound_ of currently set contacts from a thresholding of the measured forces.
  /// @details Called by \ref findContacts(const mc_control::MCController & ctl) if @contactsDetection_ is equal to
  /// "Sensors". The contacts are not required to be given by the controller (the detection is based on a
  /// thresholding of the measured force).
  void findContactsFromSensors(const mc_control::MCController & ctl, const std::string & robotName);
  /// @brief Updates the list of contacts to inform whether they are newly set, removed, etc.
  void updateContacts();

  /// @brief Adds the contact to the GUI to enable or disable it easily.
  /// @param ctl The controller.
  /// @param name Name of the contact.
  /// addContactToGui(const mc_control::MCController &, const std::string &).
  void addContactToGui(const mc_control::MCController & ctl, const std::string & surface);

  /// @brief Returns the desired list of contacts as a string object
  std::string set_to_string(const ContactsSet & contactSet);

public:
  // initialization of the odometry
  void init(const std::string & observerName, const bool verbose = true);

  /// @brief Initialization for a detection based on contact surfaces
  /// @param ctl Controller
  /// @param robotName name of the robot
  /// @param contactsDetection mean of detection for the contacts
  /// @param surfacesForContactDetection list of possible contact surfaces
  /// @param contactsSensorDisabledInit list of the force sensors that must be disabled on initialization.
  /// @param contactDetectionThreshold threshold on the measured force for the contact detection
  void initDetection(const mc_control::MCController & ctl,
                     const std::string & robotName,
                     ContactsDetection contactsDetection,
                     const std::vector<std::string> & surfacesForContactDetection,
                     const std::vector<std::string> & contactsSensorDisabledInit,
                     double contactDetectionThreshold);

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
                     ContactsDetection contactsDetection,
                     const std::vector<std::string> & contactsSensorDisabledInit,
                     double contactDetectionThreshold,
                     const std::vector<std::string> & forceSensorsToOmit);

  /// @brief Updates the list of currently set contacts and returns it.
  /// @return std::set<FoundContactsListType> &
  const ContactsSet & updateContacts(const mc_control::MCController & ctl, const std::string & robotName);

  /// @brief Accessor for the a contact associated to a sensor contained in the map
  ///
  /// @param name The name of the contact to access
  /// @return contactsWithSensorT&
  inline ContactWithSensorT & contact(const std::string & name) { return listContacts_.at(name); }

  /// @brief Accessor for the a contact associated to a sensor contained in the map
  ///
  /// @param idx The index of the contact to access
  /// @return ContactWithSensor&
  inline ContactWithSensorT & contact(const int idx) { return listContacts_.at(getNameFromIdx(idx)); }

  /// @brief Get the map of all the contacts
  ///
  /// @return std::unordered_map<std::string, contactsWithSensorT>&
  inline std::unordered_map<std::string, ContactWithSensorT> & contacts() { return listContacts_; }

  /// @brief Get the list of all the contacts.
  /// @return const std::vector<std::string> &
  inline const std::vector<std::string> & getList() { return insertOrder_; }

  /// @brief Get the name of a contact given its index
  /// @param idx The index of the contact
  /// @return const std::string &
  inline const std::string & getNameFromIdx(const int idx) { return insertOrder_.at(idx); }

  /// @brief Get the index of a contact given its name
  /// @param name The name of the contact
  /// @return const int &
  inline const int & getIdxFromName(const std::string & name) { return listContacts_.at(name).getID(); }

  /// @brief Get the list of the currently set contacts.
  /// @return const std::vector<std::string> &
  inline const ContactsSet & contactsFound() { return contactsFound_; }

  inline const ContactsSet & removedContacts() { return removedContacts_; }

  inline const ContactsDetection & getContactsDetection() { return contactsDetectionMethod_; }

protected:
  // map of contacts used by the manager.
  // unordered map containing all the contacts
  std::unordered_map<std::string, ContactWithSensorT> listContacts_;
  // List of the contacts used to access their indexes quickly
  std::vector<std::string> insertOrder_;
  // Index generator, incremented everytime a new contact is created
  int idx_ = 0;

protected:
  // method used to detect the contacts
  ContactsDetection contactsDetectionMethod_ = Undefined;
  double contactDetectionThreshold_;

  // list of the current contacts
  ContactsSet contactsFound_;
  // list of contacts that were set on last iteration
  ContactsSet oldContacts_;
  // list of the contacts that just got removed
  ContactsSet removedContacts_;

  // list of surfaces used for contacts detection if @contactsDetection_ is set to "Surfaces"
  std::vector<std::string> surfacesForContactDetection_;
  // list of sensors that must not be used from the start of the observer
  std::vector<std::string> contactsSensorDisabledInit_;

  // name of the observer using this contacts manager.
  std::string observerName_;

  bool verbose_ = true;
};

} // namespace mc_state_observation::measurements

#include <mc_state_observation/measurements/measurementsTools.hpp>
