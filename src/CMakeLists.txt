macro(add_observer observer_name)
  add_library(${observer_name} SHARED "${observer_name}.cpp" "${CMAKE_SOURCE_DIR}/include/${PROJECT_NAME}/${observer_name}.h")
  set_target_properties(${observer_name} PROPERTIES COMPILE_FLAGS "-DMC_OBSERVER_EXPORTS" PREFIX "")
  set_target_properties(${observer_name} PROPERTIES INSTALL_RPATH ${MC_OBSERVERS_INSTALL_PREFIX})
  target_link_libraries(${observer_name} PUBLIC mc_rtc::mc_observers)
  target_include_directories(${observer_name} PUBLIC $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
  install(TARGETS ${observer_name} DESTINATION "${MC_OBSERVERS_INSTALL_PREFIX}")
endmacro()

macro(add_so_observer observer_name)
  add_observer(${observer_name})
  target_link_libraries(${observer_name} PUBLIC state-observation::state-observation)
endmacro()

add_so_observer(AttitudeObserver)
add_so_observer(LegacyFlexibilityObserver)

install(DIRECTORY ${CMAKE_SOURCE_DIR}/include/${PROJECT_NAME}
  DESTINATION include)
