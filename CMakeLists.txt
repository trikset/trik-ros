cmake_minimum_required(VERSION 2.8.3)
project(trik_ros)

set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED COMPONENTS nodelet roscpp)

catkin_package(
        CATKIN_DEPENDS nodelet roscpp
)

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${QT}
)

add_executable(trikNode src/trikNode.cpp)
target_link_libraries(trikNode ${catkin_LIBRARIES} trikControl QtCoreE QtGuiE)

add_executable(ledDistance src/ledDistance.cpp)
target_link_libraries(ledDistance ${catkin_LIBRARIES})

add_library(trikNodelet src/trikNodelet.cpp)
target_link_libraries(trikNodelet ${catkin_LIBRARIES} trikControl QtCoreE QtGuiE)

add_library(ledDistanceNodelet src/ledDistanceNodelet.cpp)
target_link_libraries(ledDistanceNodelet ${catkin_LIBRARIES})

install(TARGETS trikNodelet trikNode ledDistance ledDistanceNodelet
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(FILES trik_nodelet.xml led_distance_nodelet.xml setup.sh DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
