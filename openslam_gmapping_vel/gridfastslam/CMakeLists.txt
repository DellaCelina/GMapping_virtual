add_library(gridfastslam_vel
    gfsreader.cpp
    gridslamprocessor.cpp
    gridslamprocessor_tree.cpp
    motionmodel.cpp
)
target_link_libraries(gridfastslam_vel scanmatcher_vel sensor_range_vel)

install(TARGETS gridfastslam_vel DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
