include_directories(./)
add_library(utils_vel movement.cpp stat.cpp)
install(TARGETS utils_vel DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
