add_library(scanmatcher_vel eig3.cpp scanmatcher.cpp scanmatcherprocessor.cpp smmap.cpp)
target_link_libraries(scanmatcher_vel sensor_range_vel utils_vel)

install(TARGETS scanmatcher_vel DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
