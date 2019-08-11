cmake_minimum_required(VERSION 2.8.3)
project(mars_controller)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  qt_build
  roscpp
  std_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})
# Use this to define what the package will export (e.g. libs, headers).
# Since the default here is to produce only a binary, we don't worry about
# exporting anything. 
catkin_package()

##############################################################################
# Qt Environment
##############################################################################

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

find_package(Qt5 COMPONENTS Core Gui WebSockets Widgets)

##############################################################################
# Sections
##############################################################################

#file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
#file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)


#QT5_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
#QT5_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})

include_directories(
    ${Qt5Core_INCLUDE_DIRS}
    ${Qt5Gui_INCLUDE_DIRS}
    ${Qt5WebSockets_INCLUDE_DIRS}
    ${Qt5Widgets_INCLUDE_DIRS}
    )

add_definitions( -std=c++11 -fPIC)
##############################################################################
# Sources
##############################################################################

set( QT_SOURCES
    src/main.cpp
    src/modelworker.cpp
    src/modelconfig.cpp
    src/controlsysservice.cpp
    src/mainlogic.cpp

    include/mars_controller/modelworker.h
    include/mars_controller/modelconfig.h
    include/mars_controller/controlsysservice.h
    include/mars_controller/mainlogic.h
    )

##############################################################################
# Binaries
##############################################################################

#add_executable(mars_controller ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
add_executable(mars_controller ${QT_SOURCES} ${QT_MOC_HPP})
target_link_libraries(mars_controller
    ${catkin_LIBRARIES}
    Qt5::Core
    Qt5::Widgets
    Qt5::WebSockets
)
install(TARGETS mars_controller RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})