cmake_minimum_required(VERSION 3.0.2)
project(gnss_odom)

if (NOT CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS
  )
endif()
if (CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS
    # catkin_virtualenv
  )
endif()

catkin_python_setup()
catkin_package()

install(PROGRAMS
   scripts/gnss_odom
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
   FILES_MATCHING PATTERN "*.launch"
)