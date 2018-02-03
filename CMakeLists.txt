cmake_minimum_required(VERSION 2.8.11)
project(horus_patrol)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOUIC ON)

find_package(catkin REQUIRED COMPONENTS qt_build roscpp image_transport cv_bridge rviz std_msgs)
find_package(Qt5Widgets)
find_package(Qt5Sql)
find_package(Qt5Multimedia)

include_directories(${catkin_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${Qt5Widgets_INCLUDE_DIRS})

catkin_package()

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/horus_patrol/*.hpp)
file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp)

qt5_add_resources(QT_RESOURCES_CPP ${QT_RESOURCES})
qt5_wrap_ui(QT_FORMS_HPP ${QT_FORMS})
qt5_wrap_cpp(QT_MOC_HPP ${QT_MOC})

add_executable(horus_patrol ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
target_link_libraries(horus_patrol ${catkin_LIBRARIES} Qt5::Widgets Qt5::Sql Qt5::Multimedia)
install(TARGETS horus_patrol RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
