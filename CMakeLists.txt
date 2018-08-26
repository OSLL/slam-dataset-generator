cmake_minimum_required(VERSION 3.1.0)

project(dataset_generator)

if(NOT STAGE_INSTALL)
    set(STAGE_INSTALL "${PROJECT_SOURCE_DIR}/3rdparty/install")
endif()

if(NOT catkin_LIBRARIES)
    set(catkin_LIBRARIES rosconsole console_bridge roscpp_serialization rostime rosbag_storage boost_system)
endif()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ggdb3 -Wall")
set(CMAKE_PREFIX_PATH "${CMAKE_PREFIX_PATH};/opt/ros/$ENV{ROS_DISTRO};${STAGE_INSTALL}")

find_package(Qt5Widgets REQUIRED)
find_package(stage 4.3 REQUIRED CONFIG)
find_package(ompl REQUIRED CONFIG)

if (Qt5Widgets_FOUND)
    if (Qt5Widgets_VERSION VERSION_LESS 5.7.0)
        message(FATAL_ERROR "Minimum supported Qt5 version is 5.7.0. You may need to specify qmake in PATH")
    endif()
endif()

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

#automoc works only on cpp files
#qt5_wrap_cpp(WORLD_OBJECT_MOC world_object.h)

qt5_wrap_ui(UI_HEADERS ui/simulator_config.ui ui/object_config.ui)
qt5_add_resources(UI_RESOURCES resources.qrc)

#force Qt Creator to show all project files
#file(GLOB_RECURSE Headers world_object.h)
#add_custom_target(headers SOURCES ${Headers})

link_directories(${STAGE_INSTALL}/lib)
link_directories(/opt/ros/$ENV{ROS_DISTRO}/lib)
include_directories(/opt/ros/$ENV{ROS_DISTRO}/include ${STAGE_INCLUDE_DIRS} ${OMPL_INCLUDE_DIRS})
include_directories(world)
add_executable(dataset_generator
    main.cpp mainwindow.cpp widgets.cpp simulator.cpp stage_wrapper.cpp project_file.cpp
    world/world_model.cpp world/world_items.cpp world/world_view.cpp
    ${UI_HEADERS} ${UI_RESOURCES}
)
target_link_libraries(dataset_generator ${catkin_LIBRARIES} ${STAGE_LIBRARIES} ${OMPL_LIBRARIES})

qt5_use_modules(dataset_generator Core Widgets)
