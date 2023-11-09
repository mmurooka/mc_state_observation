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

namespace mc_state_observation::measurements
{

/// @brief List of IMUs.
/// @details Facilitates the handling of IMUs.
struct MapIMUs
{
public:
  /// @brief Get the index of the IMU given its name.
  /// @param name The name of the IMU
  /// @return const int &
  inline const int & getNumFromName(const std::string & name) { return mapIMUs_.find(name)->second.id(); }
  /// @brief Get the name of the IMU given its index.
  /// @param num_ The index of the IMU
  /// @return const std::string &
  inline const std::string & getNameFromNum(const int & num) { return insertOrder_.at(num); }

  /// @brief Get the list of all the IMUs.
  /// @return const std::vector<std::string> &
  inline const std::vector<std::string> & getList() { return insertOrder_; }

  /// @brief Checks if the required IMU exists.
  /// @param name The name of the IMU
  /// @return bool
  inline bool hasElement(const std::string & name) { return checkAlreadyExists(name); }

  /// @brief Inserts an IMU to the map.
  /// @param name The name of the IMU
  inline void insertIMU(std::string name)
  {
    if(checkAlreadyExists(name)) return;
    insertOrder_.push_back(name);
    mapIMUs_.insert(std::make_pair(name, IMU(num_, name)));
    num_++;
  }

  /// @brief Accessor for an IMU in the list.
  /// @param name The name of the IMU
  /// @return bool
  inline IMU & operator()(std::string name)
  {
    BOOST_ASSERT(checkAlreadyExists(name) && "The requested sensor doesn't exist");
    return mapIMUs_.at(name);
  }

private:
  /// @brief Checks if the required IMU exists.
  /// @param name The name of the IMU
  /// @return bool
  inline bool checkAlreadyExists(const std::string & name) { return mapIMUs_.find(name) != mapIMUs_.end(); }

private:
  // list of all the IMUs.
  std::vector<std::string> insertOrder_;
  // map associating all the IMUs to their names.
  std::unordered_map<std::string, IMU> mapIMUs_;
  // Index generator, incremented everytime a new IMU is added
  int num_ = 0;
};

/**
 * Contacts manager for observers implemented in mc_rtc.
 *
 * This contact manager handles the detection of contacts with three different methods, using contact surfaces, contacts
 * directly given by the controller, or a thresholding on the measured contact force.
 *
 * On each iteration, the manager updates the list of current contacts and of removed contacts.
 **/

/// @brief Map of contacts containing the list of all the contacts and functions facilitating their handling.
/// @details The template allows to define other kinds of contacts and thus add custom parameters to them. Warning! This
/// class has been tested only on contacts with sensors
/// @tparam ContactWithSensorT Contacts associated to a sensor.
template<typename ContactWithSensorT>
struct MapContacts
{
public:
  MapContacts()
  {
    BOOST_ASSERT((std::is_base_of<ContactWithSensor, ContactWithSensorT>::value)
                 && "The template class for the contacts with sensors must inherit from the ContactWithSensor class");
  }

public:
  /// @brief Accessor for the a contact associated to a sensor contained in the map
  /// @details Version for a contact that is associated to a force sensor but to no surface
  /// @param name The name of the contact to access
  /// @return ContactWithSensorT&
  inline ContactWithSensorT & contact(const std::string & name)
  {
    BOOST_ASSERT(checkAlreadyExists(name) && "The requested sensor doesn't exist");
    return listContacts_.at(name);
  }
  /// @brief Accessor for the a contact associated to a sensor contained in the map
  ///
  /// @param num_ The index of the contact to access
  /// @return ContactWithSensor&
  inline ContactWithSensorT & contact(const int & num)
  {
    BOOST_ASSERT((num >= 0 && num < num_) && "The requested sensor doesn't exist");
    BOOST_ASSERT(checkAlreadyExists(getNameFromNum(num)) && "The requested sensor doesn't exist");
    return listContacts_.at(getNameFromNum(num));
  }

  /// @brief Get the map of all the contacts
  ///
  /// @return std::unordered_map<std::string, contactsWithSensorT>&
  inline std::unordered_map<std::string, ContactWithSensorT> & contacts() { return listContacts_; }

  /// @brief Get the list of all the contacts (with and without sensors)
  ///
  /// @return const std::vector<std::string> &
  inline const std::vector<std::string> & getList() { return insertOrder_; }

  /// @brief Get the name of a contact given its index
  ///
  /// @param num_ The index of the contact
  /// @return const std::string &
  inline const std::string & getNameFromNum(const int & num) { return insertOrder_.at(num); }

  /// @brief Get the index of a contact given its name
  /// @param name The name of the contact
  /// @return const int &
  inline const int & getNumFromName(const std::string & name) { return listContacts_.at(name).getID(); }

  /// @brief Checks if the given contact exists
  ///
  /// @param element The name of the contact
  /// @return bool
  inline bool hasElement(const std::string & element) { return listContacts_.find(element) != listContacts_.end(); }

  /// @brief Check that a contact still does not exist, if so, insert a contact to the map of contacts.
  /// @details Version for contacts that are detected by a thresholding on a force sensor measurement.
  /// @param element The name of the contact which corresponds to the name of the sensor.
  inline void insertContact(const std::string & forceSensorName)
  {
    if(checkAlreadyExists(forceSensorName)) return;
    insertElement(forceSensorName);

    num_++;
  }

  /// @brief Check that a contact still does not exist, if so, insert a contact to the map of contacts.
  /// @details Version for contacts that are associated to both a force sensor and a contact surface. The contact will
  /// be named with the name of the surface.
  /// @param forceSensorName The name of the force sensor.
  /// @param surface The name of the surface that will be used also to name the contact.
  inline void insertContact(const std::string & forceSensorName, const std::string surface)
  {
    if(checkAlreadyExists(forceSensorName, surface)) return;
    insertElement(forceSensorName, surface);

    num_++;
  }

private:
  /// @brief Insert a contact to the map of contacts.
  /// @details Version for contacts that are associated to both a force sensor and a contact surface. The contact will
  /// be named with the name of the surface.
  /// @param forceSensorName The name of the force sensor.
  /// @param surface The name of the surface that will be used also to name the contact.
  inline void insertElement(const std::string & forceSensorName, const std::string surface)
  {
    listContacts_.insert(std::make_pair(forceSensorName, ContactWithSensorT(num_, forceSensorName, surface)));
    insertOrder_.push_back(forceSensorName);
  }
  /// @brief Insert a contact to the map of contacts.
  /// @details Version for contacts that are associated to a force sensor but to no surface.
  /// @param name The name of the contact (= name of the sensor)
  inline void insertElement(const std::string & forceSensorName)
  {
    listContacts_.insert(std::make_pair(forceSensorName, ContactWithSensorT(num_, forceSensorName)));
    insertOrder_.push_back(forceSensorName);
  }

  /// @brief Check if a contact already exists in the list. If it already exists, checks that the contact remained
  /// unchanged.
  /// @details Version for contacts associated to a force sensor and to no surface
  /// @param name The name of the contact (=name of the force sensor)
  /// @return bool
  inline bool checkAlreadyExists(const std::string & forceSensorName)
  {
    if(listContacts_.find(forceSensorName) != listContacts_.end()) // the contact already exists
    {
      return true;
    }
    else { return false; }
  }

  /// @brief Check if a contact already exists in the list. If it already exists, checks that the contact remained
  /// unchanged.
  ///
  /// @param name The name of the contact
  /// @return bool
  inline bool checkAlreadyExists(const std::string & forceSensorName, const std::string & surface)
  {
    if(listContacts_.find(surface) != listContacts_.end()) // the contact already exists
    {
      if(contact(surface).forceSensorName() == forceSensorName) { return true; }
    }
    return false;
  }

private:
  // unordered map containing all the contacts
  std::unordered_map<std::string, ContactWithSensorT> listContacts_;
  // List of the contacts to access their indexes
  std::vector<std::string> insertOrder_;
  // Index generator, incremented everytime a new contact is created
  int num_ = 0;
};

/// @brief Structure that implements all the necessary functions to manage the map of contacts. Handles their detection
/// and updates the list of the detected contacts, newly removed contacts, etc., to apply the appropriate functions on
/// them.
/// @details The template allows to define other kinds of contacts and thus add custom parameters to them. Warning! This
/// class has been tested only on contacts with sensors
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
  inline ContactWithSensorT & contact(const std::string & name) { return mapContacts_.contact(name); }

  /// @brief Accessor for the a contact associated to a sensor contained in the map
  ///
  /// @param num The index of the contact to access
  /// @return ContactWithSensor&
  inline ContactWithSensorT & contact(const int & num) { return mapContacts_.contact(num); }

  /// @brief Get the map of all the contacts
  ///
  /// @return std::unordered_map<std::string, contactsWithSensorT>&
  inline std::unordered_map<std::string, ContactWithSensorT> & contacts() { return mapContacts_.contacts(); }

  /// @brief Get the list of all the contacts.
  /// @return const std::vector<std::string> &
  inline const std::vector<std::string> & getList() { return mapContacts_.getList(); }

  /// @brief Get the list of the currently set contacts.
  /// @return const std::vector<std::string> &
  inline const ContactsSet & contactsFound() { return contactsFound_; }

  inline const ContactsSet & removedContacts() { return removedContacts_; }

  inline const ContactsDetection & getContactsDetection() { return contactsDetectionMethod_; }

public:
  // map of contacts used by the manager.
  MapContacts<ContactWithSensorT> mapContacts_;

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

// allowed odometry types
enum class OdometryType
{
  Odometry6d,
  Flat,
  None
};

} // namespace mc_state_observation::measurements

#include <mc_state_observation/measurements/measurementsTools.hpp>
