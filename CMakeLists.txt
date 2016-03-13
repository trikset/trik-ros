cmake_minimum_required(VERSION 2.8.3)
project(trik_ros)

set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED COMPONENTS nodelet roscpp)

find_package(Qt4)
INCLUDE(${QT_USE_FILE})
ADD_DEFINITIONS(${QT_DEFINITIONS})

catkin_package(
  LIBRARIES trik
  CATKIN_DEPENDS nodelet roscpp
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${QT}
)

add_library(trik src/trik.cpp)
target_link_libraries(trik ${catkin_LIBRARIES})

add_executable(standalone_node src/standalone_node.cpp)
target_link_libraries(standalone_node ${catkin_LIBRARIES} trikControl QtCoreE QtGuiE)

add_executable(led_distance src/led_distance.cpp)
target_link_libraries(led_distance ${catkin_LIBRARIES})

install(TARGETS trik standalone_node led_distance
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES nodelet_trik.xml setup.sh DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
