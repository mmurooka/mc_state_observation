#pragma once
#include <mc_control/MCController.h>
#include <mc_state_observation/measurements/ContactWithSensor.h>

#include <mc_state_observation/measurements/ContactsManagerConfiguration.h>

#include <set>

namespace mc_state_observation::measurements
{
/// @brief Structure that implements all the necessary functions to manage the map of contacts. Handles their detection
/// and updates the list of the detected contacts, newly removed contacts, etc., to apply the appropriate functions on
/// them.
/// @details The template allows to define other kinds of contacts and thus add custom parameters to them.
/// @tparam ContactT Contact, associated to a sensor.
template<typename ContactT>
struct ContactsManager
{
public:
  // allowed contact detection methods
  enum ContactsDetection
  {
    Solver,
    Surfaces,
    Sensors,
    Undefined
  };
  typedef std::set<int> ContactsSet;

  using Configuration = std::variant<ContactsManagerSolverConfiguration,
                                     ContactsManagerSurfacesConfiguration,
                                     ContactsManagerSensorsConfiguration>;

  static_assert(std::is_base_of_v<ContactWithSensor, ContactT>,
                "The template class for the contacts with sensors must inherit from the ContactWithSensor class");

private:
  // map allowing to get the ContactsDetection value associated to the given string
  inline static const std::unordered_map<std::string, ContactsDetection> strToContactsDetection =
      {{"Solver", Solver}, {"Surfaces", Surfaces}, {"Sensors", Sensors}, {"Undefined", Undefined}};

protected:
  /// @brief Inserts a contact to the map of contacts.
  /// @details Version for contacts that are associated to both a force sensor and a contact surface. The contact will
  /// be named with the name of the force sensor.
  /// @param forceSensorName The name of the force sensor.
  /// @param surface The name of the surface that will be used also to name the contact.
  /// @param onAddedContact function to call when a contact is added to the manager
  /// @return ContactT &
  template<typename OnAddedContact = std::nullptr_t>
  inline ContactT & addContactToManager(const std::string & forceSensorName,
                                        const std::string & surface,
                                        OnAddedContact onAddedContact = nullptr);
  /// @brief Insert a contact to the map of contacts.
  /// @details Version for contacts that are associated to a force sensor but to no surface.
  /// @param name The name of the contact (= name of the sensor)
  /// @param onAddedContact function to call when a contact is added to the manager
  /// @return ContactT &
  template<typename OnAddedContact = std::nullptr_t>
  inline ContactT & addContactToManager(const std::string & forceSensorName, OnAddedContact onAddedContact = nullptr)
  {
    return addContactToManager(forceSensorName, "", onAddedContact);
  }

  /// @brief Updates the list @contactsFound_ of currently set contacts from the surfaces given by the user.
  /// @details Called by \ref updateContacts(const mc_control::MCController &, const std::string &, OnNewContact,
  /// OnMaintainedContact, OnRemovedContact, OnAddedContact) if @contactsDetection_ is equal to
  /// "Surfaces". The contacts are obtained by thresholded based the force measured by the associated force sensor).
  void findContactsFromSurfaces(const mc_control::MCController & ctl, const std::string & robotName);
  /// @brief Updates the list @contactsFound_ of currently set contacts from a thresholding of the measured forces.
  /// @details Called by \ref updateContacts(const mc_control::MCController &, const std::string &, OnNewContact,
  /// OnMaintainedContact, OnRemovedContact, OnAddedContact) if @contactsDetection_ is equal to
  /// "Sensors". The contacts are not required to be given by the controller (the detection is based on a
  /// thresholding of the measured force).
  void findContactsFromSensors(const mc_control::MCController & ctl, const std::string & robotName);

  /// @brief Updates the list @contactsFound_ of currently set contacts directly from the controller.
  /// @details Called by \ref updateContacts(const mc_control::MCController &, const std::string &, OnNewContact,
  /// OnMaintainedContact, OnRemovedContact, OnAddedContact) if @contactsDetection_ is equal to "Solver". The
  /// contacts are given by the controller directly (then thresholded based on the measured force).
  /// @param onAddedContact function to call when a contact is added to the manager
  template<typename OnAddedContact = std::nullptr_t>
  void findContactsFromSolver(const mc_control::MCController & ctl,
                              const std::string & robotName,
                              OnAddedContact onAddedContact = nullptr);

  /// @brief Returns the desired list of contacts as a string object
  std::string set_to_string(const ContactsSet & contactSet);

public:
  // initialization of the contacts manager
  /// @param ctl Controller
  /// @param robotName Name of the robot
  /// @param conf Configutation of the manager
  /// @param onAddedContact function to call when a contact is added to the manager
  template<typename OnAddedContact = std::nullptr_t>
  void init(const mc_control::MCController & ctl,
            const std::string & robotName,
            Configuration conf,
            OnAddedContact onAddedContact = nullptr);

  /// @brief Updates the list of contacts to inform whether they are newly set, removed, etc., and execute actions
  /// accordingly
  /// @param ctl Controller
  /// @param robotName Name of the robot
  /// @param onNewContact Function to call on a newly detected contact
  /// @param onNewContact Function to call on a newly detected contact
  /// @param onMaintainedContact Function to call on a contact maintainted since the last iteration
  /// @param onRemovedContact Function to call on a removed contact
  /// @param onAddedContact function to call when a contact is added to the manager
  /// @return void
  template<typename OnNewContact,
           typename OnMaintainedContact,
           typename OnRemovedContact,
           typename OnAddedContact = std::nullptr_t>
  void updateContacts(const mc_control::MCController & ctl,
                      const std::string & robotName,
                      OnNewContact onNewContact,
                      OnMaintainedContact onMaintainedContact,
                      OnRemovedContact onRemovedContact,
                      OnAddedContact onAddedContact = nullptr);

  /// @brief Accessor for the a contact associated to a sensor contained in the map
  ///
  /// @param name The name of the contact to access
  /// @return contactsWithSensorT&
  inline ContactT & contact(const std::string & name) { return listContacts_.at(name); }

  /// @brief Accessor for the a contact associated to a sensor contained in the map
  ///
  /// @param idx The index of the contact to access
  /// @return ContactWithSensor&
  inline ContactT & contact(const int idx) { return listContacts_.at(getNameFromIdx(idx)); }

  /// @brief Get the map of all the contacts
  ///
  /// @return std::unordered_map<std::string, contactsWithSensorT>&
  inline std::unordered_map<std::string, ContactT> & contacts() { return listContacts_; }

  /// @brief Get the list of all the contacts.
  /// @return const std::vector<std::string> &
  inline const std::vector<std::string> & getList() { return insertOrder_; }

  /// @brief Get the name of a contact given its index
  /// @param idx The index of the contact
  /// @return const std::string &
  inline const std::string & getNameFromIdx(const int idx) { return insertOrder_.at(idx); }

  /// @brief Get the list of the currently set contacts.
  /// @return const std::vector<std::string> &
  inline const ContactsSet & contactsFound() { return contactsFound_; }

  inline const ContactsDetection & getContactsDetection() { return contactsDetectionMethod_; }

  /// @brief Sets the contacts detection method used in the odometry.
  /// @details Allows to set the contacts detection method directly from a string, most likely obtained from a
  /// configuration file.
  /// @param str The string naming the method to be used.
  inline static ContactsDetection stringToContactsDetection(const std::string & str, const std::string & observerName)
  {
    auto it = strToContactsDetection.find(str);
    if(it != strToContactsDetection.end()) { return it->second; }
    mc_rtc::log::error_and_throw<std::runtime_error>("[{}]: No known ContactsDetection value for {}", observerName,
                                                     str);
  }

private:
  /// @brief Initializer for a contacts detection based on contact surfaces
  /// @param ctl The controller
  /// @param robotName Name of the robot
  /// @param conf Configuration of the contacts manager
  /// @param onAddedContact Function to call when a contact is added to the manager
  template<typename OnAddedContact = std::nullptr_t>
  inline void init_manager(const mc_control::MCController & ctl,
                           const std::string & robotName,
                           const ContactsManagerSurfacesConfiguration & conf,
                           OnAddedContact onAddedContact = nullptr);
  /// @brief Initializer for a contacts detection based on force sensors
  /// @param ctl The controller
  /// @param robotName Name of the robot
  /// @param conf Configuration of the contacts manager
  /// @param onAddedContact Function to call when a contact is added to the manager
  template<typename OnAddedContact = std::nullptr_t>
  inline void init_manager(const mc_control::MCController & ctl,
                           const std::string & robotName,
                           const ContactsManagerSensorsConfiguration & conf,
                           OnAddedContact onAddedContact = nullptr);
  /// @brief Initializer for a contacts detection based on the solver's contacts
  /// @param ctl The controller
  /// @param robotName Name of the robot
  /// @param conf Configuration of the contacts manager
  /// @param onAddedContact Function to call when a contact is added to the manager. Unused here as contacts are added
  /// to the manager during the run with this detection method, but added anyway to match the syntax of the other
  /// init_manager variants.
  template<typename OnAddedContact = std::nullptr_t>
  inline void init_manager(const mc_control::MCController & ctl,
                           const std::string & robotName,
                           const ContactsManagerSolverConfiguration & conf,
                           OnAddedContact onAddedContact = nullptr);

protected:
  // map of contacts used by the manager.
  // unordered map containing all the contacts
  std::unordered_map<std::string, ContactT> listContacts_;
  // List of the contacts used to access their indexes quickly
  std::vector<std::string> insertOrder_;
  // Index generator, incremented everytime a new contact is created
  int idx_ = 0;

  // method used to detect the contacts
  ContactsDetection contactsDetectionMethod_ = Undefined;
  // threshold for the contacts detection
  double contactDetectionThreshold_;

  // list of the current contacts
  ContactsSet contactsFound_;
  // list of contacts that were set on last iteration
  ContactsSet oldContacts_;
  // list of the contacts that just got removed
  ContactsSet removedContacts_;

  // list of surfaces used for contacts detection if @contactsDetection_ is set to "Surfaces"
  std::vector<std::string> surfacesForContactDetection_;

  // name of the observer using this contacts manager.
  std::string observerName_;

  bool verbose_ = true;
};
} // namespace mc_state_observation::measurements

#include <mc_state_observation/measurements/ContactsManager.hpp>
