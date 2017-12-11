include_directories(./)
add_library(sensor_base_vel sensor.cpp sensorreading.cpp)
install(TARGETS sensor_base_vel DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
