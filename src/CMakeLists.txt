macro(add_so_observer observer_name)
  add_observer(${observer_name} "${observer_name}.cpp" "${CMAKE_SOURCE_DIR}/include/${PROJECT_NAME}/${observer_name}.h")
  target_link_libraries(${observer_name} PUBLIC state-observation::state-observation)
endmacro()

add_so_observer(AttitudeObserver)
add_so_observer(LegacyFlexibilityObserver)

install(DIRECTORY ${CMAKE_SOURCE_DIR}/include/${PROJECT_NAME}
  DESTINATION include)
