cmake_minimum_required(VERSION 3.14)

project(horus_patrol LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

IF (NOT DEFINED BUILD_VERSION)
  SET(BUILD_VERSION "not set")
ENDIF()
ADD_DEFINITIONS(-DBUILD_VERSION="${BUILD_VERSION}")

if(NOT WIN32)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wextra -fPIC")
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(QT NAMES Qt6 Qt5 COMPONENTS Core Quick Sql REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Core Quick Sql REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(robot_msgs REQUIRED)

include_directories(
  /usr/include/OGRE
  )

set(HEADERS
  include/horus_patrol/robot.hpp
  include/horus_patrol/ros_node.hpp
  include/horus_patrol/user_queries.hpp
  )
foreach(header "${HEADERS}")
  qt5_wrap_cpp(HEADERS_FILES "${header}")
endforeach()

set(SRC_FILES
  src/main.cpp
  src/robot.cpp
  src/ros_node.cpp
  src/user_queries.cpp
  )

set(RES_FILES
  res/res.qrc
  )

if(${QT_VERSION_MAJOR} GREATER_EQUAL 6)
  qt_add_executable(horus_patrol
    ${HEADERS_FILES}
    ${SRC_FILES}
    ${RES_FILES}
    )
else()
  add_executable(horus_patrol
    ${HEADERS_FILES}
    ${SRC_FILES}
    ${RES_FILES}
    )
endif()

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  )

ament_target_dependencies(${PROJECT_NAME}
  "rclcpp"
  "std_msgs"
  "robot_msgs"
  )

#target_compile_definitions(horus_patrol
#  PRIVATE $<$<OR:$<CONFIG:Debug>,$<CONFIG:RelWithDebInfo>>:QT_QML_DEBUG>)
target_link_libraries(${PROJECT_NAME}
  Qt${QT_VERSION_MAJOR}::Core
  Qt${QT_VERSION_MAJOR}::Quick
  Qt${QT_VERSION_MAJOR}::Sql
  ${rclcpp_LIBRARIES}
  )

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
  )

ament_package()
