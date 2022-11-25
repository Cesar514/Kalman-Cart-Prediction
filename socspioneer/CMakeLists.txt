# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(socspioneer)

find_package(catkin REQUIRED COMPONENTS)
catkin_package()

install(DIRECTORY 
   launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
 )
 
 catkin_install_python(PROGRAMS script/solver.py script/twist.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
