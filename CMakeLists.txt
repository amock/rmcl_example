cmake_minimum_required(VERSION 2.8.3)
project(rmcl_example)

find_package(catkin REQUIRED)
catkin_package()

install(DIRECTORY
    launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY
    maps/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/maps
)

install(DIRECTORY
    worlds/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/worlds
)

install(DIRECTORY
    urdf/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
)

install(DIRECTORY
    config/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)