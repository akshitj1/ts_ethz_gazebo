cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
cmake_policy(SET CMP0042 NEW)
cmake_policy(SET CMP0048 NEW)
cmake_policy(SET CMP0054 NEW)

project(ts_ethz_gazebo)


## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

#######################
## Find Dependencies ##
#######################

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")


find_package(catkin REQUIRED COMPONENTS
        gazebo_ros
        roscpp
        std_msgs
        )

find_package(gazebo REQUIRED)

find_package(Eigen3 QUIET)
if (NOT EIGEN3_FOUND)
    # Fallback to cmake_modules
    find_package(Eigen QUIET)
    if (NOT EIGEN_FOUND)
        pkg_check_modules(EIGEN3 REQUIRED eigen3)
    else ()
        set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
        set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})
    endif ()
else ()
    set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif ()

find_package(TinyXML REQUIRED)

# for ROS subscribers and publishers
if (BUILD_ROS_INTERFACE)
    find_package(roscpp REQUIRED)
endif ()

link_directories(${GAZEBO_LIBRARY_DIRS})


catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES ts_ethz_gazebo
        #  CATKIN_DEPENDS gazebo_ros ros_cpp
        DEPENDS
        roscpp
        gazebo_ros
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIR}
        ${GAZEBO_INCLUDE_DIRS}
        ${TinyXML_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}/eigen3 # Workaround for Eigen3
        ${CMAKE_CURRENT_BINARY_DIR}/msgs
        ${std_msgs_INCLUDE_DIRS}
)
link_directories(${GAZEBO_LIBRARY_DIRS} ${CMAKE_CURRENT_BINARY_DIR}/msgs)
add_subdirectory(msgs)


#---------#
# Plugins #
#---------#

set(plugins
        model_push
        liftdrag_plugin
        gazebo_motor_model
        gazebo_imu_plugin
        )


## Declare a C++ library
#add_library(${PROJECT_NAME} src/hello_world_plugin.cpp)
#add_library(model_push SHARED src/model_push.cpp)
foreach (plugin ${plugins})
    add_library(${plugin} SHARED src/${plugin}.cpp)
endforeach ()


## Specify libraries to link a library or executable target against
#target_link_libraries(${PROJECT_NAME}
#  ${catkin_LIBRARIES}
#  ${GAZEBO_LIBRARIES}
#)
# TODO: check how we can inherit this list from msgs/Cmak...
foreach (plugin ${plugins})
    target_link_libraries(${plugin} ts_ethz_gazebo_msgs ${roscpp_LIBRARIES} ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${TinyXML_LIBRARIES})
    add_dependencies(${plugin} ts_ethz_gazebo_msgs)
endforeach ()


# Linux is not consistent with plugin availability, even on Gazebo 7
#if("${GAZEBO_VERSION}" VERSION_LESS "7.0")
add_library(LiftDragPlugin SHARED src/liftdrag_plugin.cpp)
list(APPEND plugins LiftDragPlugin)
#endif()