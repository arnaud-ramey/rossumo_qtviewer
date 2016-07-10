cmake_minimum_required(VERSION 2.8.3)
project(roswifibot_qtviewer)
# Debug, Release, RelWithDebInfo and MinSizeRe
set(CMAKE_BUILD_TYPE RelWithDebInfo)
# These find_package calls setup catkin_INCLUDE_DIRS and ROS_LIBRARIES and
# all the component variables too (e.g. roscpp_INCLUDE_DIR?)
SET(ROSDEPS cv_bridge image_transport roscpp qt_build std_msgs usb_cam)
find_package(catkin REQUIRED COMPONENTS ${ROSDEPS})
# https://github.com/stonier/qt_ros/blob/indigo/qt_tutorials/CMakeLists.txt
# this comes from qt_build's qt-ros.cmake which is automatically
# included via the dependency call in manifest.xml
#rosbuild_prepare_qt4(QtCore QtGui) # Add the appropriate components to the component list here
find_package(Qt4 COMPONENTS QtCore QtGui)
catkin_package(CATKIN_DEPENDS ${ROSDEPS})
include_directories(${catkin_INCLUDE_DIRS} ${QT_INCLUDES} ${PROJECT_BINARY_DIR})
include_directories("/home/arnaud/catkin_ws/build/roswifibot_qtviewer")
message("QT_INCLUDES:${QT_INCLUDES}")
message("PROJECT_BINARY_DIR:${PROJECT_BINARY_DIR}")

# UI
set(QT_FORMS ${CMAKE_CURRENT_SOURCE_DIR}/src/mainwindow.ui)
QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
# MOC
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS *.h)
QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})
# CPP
file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS *.cpp)
# Binaries
add_executable(mainwindow ${QT_SOURCES} ${QT_MOC_HPP} ${QT_FORMS_HPP})
#target_link_libraries(mainwindow ${QT_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(mainwindow ${catkin_LIBRARIES} Qt4::QtGui)
