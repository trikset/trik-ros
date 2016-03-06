cmake_minimum_required(VERSION 2.8.3)
project(trik_ros)

find_package(catkin REQUIRED COMPONENTS nodelet roscpp)

catkin_package(
  LIBRARIES trik
  CATKIN_DEPENDS nodelet roscpp
)


include_directories(
  ${catkin_INCLUDE_DIRS}
#  ${OE_QMAKE_INCDIR_QT}
  ${QT}
)

add_library(trik src/trik.cpp)
target_link_libraries(trik ${catkin_LIBRARIES})
  

install(TARGETS trik
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES nodelet_trik.xml launch/trik.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
