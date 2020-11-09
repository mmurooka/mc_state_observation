macro(add_observer observer_name observer_SRC observer_HDR)
  add_library(${observer_name} SHARED ${observer_SRC} ${observer_HDR})
  set_target_properties(${observer_name} PROPERTIES COMPILE_FLAGS "-DMC_OBSERVER_EXPORTS" PREFIX "")
  set_target_properties(${observer_name} PROPERTIES INSTALL_RPATH ${MC_OBSERVERS_INSTALL_PREFIX})
  target_link_libraries(${observer_name} PUBLIC mc_rtc::mc_observers)
  install(TARGETS ${observer_name} DESTINATION "${MC_OBSERVERS_INSTALL_PREFIX}")
endmacro()

add_observer(AttitudeObserver AttitudeObserver.cpp ../include/${PROJECT_NAME}/AttitudeObserver.h)
target_link_libraries(AttitudeObserver PUBLIC state-observation::state-observation)
target_include_directories(AttitudeObserver PUBLIC $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)

install(DIRECTORY ${CMAKE_SOURCE_DIR}/include/${PROJECT_NAME}
  DESTINATION include)
