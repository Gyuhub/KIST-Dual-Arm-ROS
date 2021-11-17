cmake_minimum_required(VERSION 3.0.2)

project(dual_arm)

add_compile_options(-std=c++11 -O)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    tf
)

find_package(Eigen3 3.3 REQUIRED)

find_package(OpenGL REQUIRED)

find_package(GLEW REQUIRED)

if(NOT OMPL_INCLUDE_DIRS)
	find_package(ompl REQUIRED)
endif()

set(SRC_FILES
    src/controller.cpp
    src/quadraticprogram.cpp
    src/robotmodel.cpp
    src/trajectory.cpp
    src/motionplan.cpp
)

set(MUJOCO_PATH
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../mujoco200_linux
    #~/mujoco200_linux
)

 ################# For cmake ###########################
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin )
#set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin/lib )
#set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin/lib )

message("Mujoco Path : " ${MUJOCO_PATH})

catkin_package(
  INCLUDE_DIRS include
#   LIBRARIES dual_arm
  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${GLEW_INCLUDE_DIRS}
 #   ${CMAKE_CURRENT_SOURCE_DIR}/qpOASES-3.2/include
 #   ${CMAKE_CURRENT_SOURCE_DIR}/qpOASES-3.2/include/qpOASES
 #   ${CMAKE_CURRENT_SOURCE_DIR}/rbdl/include
    ${MUJOCO_PATH}/include
    /usr/local/include/qpOASES
    ${OMPL_INCLUDE_DIRS}
#    /usr/local/include/eigen3
)

link_directories(
    #/usr/local/lib
    #/usr/lib
    #/usr/bin
    ${MUJOCO_PATH}/bin
    #/usr/include/eigen3/Eigen
)

file(GLOB LIB_MUJOCO ${MUJOCO_PATH}/bin/libmujoco200.so)

file(GLOB UI_SRC ${MUJOCO_PATH}/include/uitools.c)

add_library(dualarm_lib
    ${UI_SRC}
    ${SRC_FILES}
    ${catkin_LIBRARIES}
)

add_executable(dualarm_exe
    src/simulate.cpp
    ${MUJOCO_PATH}/include/mjxmacro.h
    ${MUJOCO_PATH}/include/uitools.h
#    ${SRC_FILES}
)

target_link_libraries(dualarm_exe
    #${CMAKE_CURRENT_SOURCE_DIR}/qpOASES-3.2/include
    #${CMAKE_CURRENT_SOURCE_DIR}/qpOASES-3.2/include/qpOASES
    #${CMAKE_CURRENT_SOURCE_DIR}/rbdl/include
PUBLIC
    ${catkin_LIBRARIES}
    Eigen3::Eigen
    dualarm_lib
    -lrbdl
    -lrbdl_urdfreader
    -lpthread
    -lqpOASES
    ${OMPL_LIBRARIES}
    ${MUJOCO_PATH}/bin/libglew.so
    ${MUJOCO_PATH}/bin/libglfw.so.3
    #GL
    ${LIB_MUJOCO}
    OpenGL::OpenGL
    OpenGL::EGL
    GLEW::GLEW
    ${GLEW_LIBRARIES}
    ${OPENGL_LIBRARIES}
)

################# For ROS ###########################
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )

 ################# For cmake ###########################
#install(TARGETS dualarm_exe DESTINATION bin)

#target_include_directories(dualarm_lib
#PUBLIC
#    ${CMAKE_SOURCE_DIR}/include
#)