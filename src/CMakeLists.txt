macro(add_so_observer observer_name)
  add_observer(${observer_name} "${observer_name}.cpp" "${CMAKE_SOURCE_DIR}/include/${PROJECT_NAME}/${observer_name}.h")
  target_link_libraries(${observer_name} PUBLIC state-observation::state-observation)
endmacro()

macro(add_simple_observer observer_name)
  add_observer(${observer_name} "${observer_name}.cpp" "${CMAKE_SOURCE_DIR}/include/${PROJECT_NAME}/${observer_name}.h")
endmacro()

macro(add_observer_with_filter observer_name)
  add_observer(${observer_name} "${observer_name}.cpp;filtering.cpp" "${CMAKE_SOURCE_DIR}/include/${PROJECT_NAME}/${observer_name}.h;${CMAKE_SOURCE_DIR}/include/${PROJECT_NAME}/filtering.h")
endmacro()

add_so_observer(AttitudeObserver)
add_so_observer(LegacyFlexibilityObserver)
add_so_observer(TiltObserver)

if(${ROSCPP_FOUND})
  add_simple_observer(MocapObserver)
  target_link_libraries(MocapObserver PUBLIC mc_rtc::mc_control) # XXX why needed?
  add_simple_observer(MocapObserverROS)
  target_link_libraries(MocapObserverROS PUBLIC MocapObserver)
  target_link_libraries(MocapObserverROS PUBLIC mc_state_observation::ROS mc_rtc::mc_rtc_ros)
  set_target_properties(MocapObserverROS PROPERTIES INSTALL_RPATH ${MC_OBSERVERS_RUNTIME_INSTALL_PREFIX})
endif()

if(${ROSCPP_FOUND})
  find_package(gram_savitzky_golay REQUIRED)
  add_observer_with_filter(SLAMObserver)
  target_link_libraries(SLAMObserver PUBLIC mc_rtc::mc_control mc_state_observation::ROS mc_rtc::mc_rtc_ros gram_savitzky_golay::gram_savitzky_golay)
  set_target_properties(SLAMObserver PROPERTIES INSTALL_RPATH ${MC_OBSERVERS_RUNTIME_INSTALL_PREFIX})
endif()

install(DIRECTORY ${CMAKE_SOURCE_DIR}/include/${PROJECT_NAME}
  DESTINATION include)
