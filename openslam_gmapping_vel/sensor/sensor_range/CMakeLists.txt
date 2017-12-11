add_library(sensor_range_vel rangereading.cpp rangesensor.cpp)
target_link_libraries(sensor_range_vel sensor_base_vel)

install(TARGETS sensor_range_vel DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
