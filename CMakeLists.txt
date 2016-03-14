cmake_minimum_required(VERSION 2.8.3)
project(trik_ros)

set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED COMPONENTS nodelet roscpp)
# find_package(Boost) # todo: check run with/without on a real board

find_package(Qt4)
INCLUDE(${QT_USE_FILE})
ADD_DEFINITIONS(${QT_DEFINITIONS})

catkin_package(
        CATKIN_DEPENDS nodelet roscpp
)

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${QT}
        # ${Boost_INCLUDE_DIRS} # todo: check run with/without on a real board
)

add_executable(trikNode src/trikNode.cpp)
target_link_libraries(trikNode ${catkin_LIBRARIES} trikControl QtCoreE QtGuiE)

add_executable(ledDistance src/ledDistance.cpp)
target_link_libraries(ledDistance ${catkin_LIBRARIES})

add_library(trikNodelet src/trikNodelet.cpp)
target_link_libraries(trikNodelet
        ${catkin_LIBRARIES}
        trikControl
        QtCoreE
        QtGuiE
        # ${Boost_LIBRARIES} # todo: check run with/without on a real board
        )

add_library(ledDistanceNodelet src/ledDistanceNodelet.cpp)
target_link_libraries(ledDistanceNodelet
        ${catkin_LIBRARIES}
        # ${Boost_LIBRARIES} # todo: check run with/without on a real board
        )

install(TARGETS trikNodelet trikNode ledDistance ledDistanceNodelet
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(FILES trik_nodelet.xml led_distance_nodelet.xml setup.sh DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
