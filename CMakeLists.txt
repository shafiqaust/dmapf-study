cmake_minimum_required(VERSION 2.8.3)
project(dmapf)

## Compile as C++11, supported in ROS Melodic
add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  roslib
  std_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Aim.msg
  Area.msg
  Areas.msg
  Coor.msg
  Cross.msg
  Dist.msg
  Move.msg
  Next.msg
  Plan.msg
  Position.msg
  Step.msg
  Tier.msg
  Target.msg
  Track.msg
  Transit.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  Abstract.srv
  Aggregate.srv
  Migrate.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES dmapf
  CATKIN_DEPENDS message_runtime roscpp roslib std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  $ENV{CONDA_PREFIX}/include
  ${catkin_INCLUDE_DIRS}
  include
  solvers
)

link_directories(
  $ENV{CONDA_PREFIX}/lib
)

## Declare a C++ library
add_library(hungarian src/hungarian.cpp)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/dmapf_node.cpp)
add_executable(gendiv src/gendiv.cpp)
add_executable(solver src/solver.cpp)
add_executable(lp2map src/lp2map.cpp)
add_executable(lp2yaml src/lp2yaml.cpp)
add_executable(map2lp src/map2lp.cpp)
add_executable(d2od src/d2od.cpp)
add_executable(probmod src/probmod.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#add_dependencies(solver ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(gendiv ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(solver ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(lp2map ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(lp2yaml ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(map2lp ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(d2od ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(probmod ${${PROJECT_NAME}_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )
target_link_libraries(gendiv ${catkin_LIBRARIES} clingo hungarian)
target_link_libraries(lp2map ${catkin_LIBRARIES} clingo)
target_link_libraries(lp2yaml ${catkin_LIBRARIES} clingo)
target_link_libraries(map2lp ${catkin_LIBRARIES})
target_link_libraries(d2od ${catkin_LIBRARIES})
target_link_libraries(probmod ${catkin_LIBRARIES})

if (DEFINED SOLVER)
  string(TOUPPER ${SOLVER} solver)
  if (solver STREQUAL "CBSH2_RTC")
    add_definitions(-DSOLVER_CBSH2_RTC)
    set(CBSH2_RTC_SOURCES
      solvers/CBSH2-RTC/src/CBS.cpp
      solvers/CBSH2-RTC/src/CBSHeuristic.cpp
      solvers/CBSH2-RTC/src/CBSNode.cpp
      solvers/CBSH2-RTC/src/common.cpp
      solvers/CBSH2-RTC/src/Conflict.cpp
      solvers/CBSH2-RTC/src/ConstraintPropagation.cpp
      solvers/CBSH2-RTC/src/ConstraintTable.cpp
      solvers/CBSH2-RTC/src/CorridorReasoning.cpp
      solvers/CBSH2-RTC/src/IncrementalPairwiseMutexPropagation.cpp
      solvers/CBSH2-RTC/src/Instance.cpp
      solvers/CBSH2-RTC/src/MDD.cpp
      solvers/CBSH2-RTC/src/MutexReasoning.cpp
      solvers/CBSH2-RTC/src/RectangleReasoning.cpp
      solvers/CBSH2-RTC/src/ReservationTable.cpp
      solvers/CBSH2-RTC/src/SingleAgentSolver.cpp
      solvers/CBSH2-RTC/src/SIPP.cpp
      solvers/CBSH2-RTC/src/SpaceTimeAStar.cpp
    )
    add_library(cbsh2_rtc ${CBSH2_RTC_SOURCES})
    target_include_directories(cbsh2_rtc PRIVATE solvers/CBSH2-RTC/inc)
    target_link_libraries(solver ${catkin_LIBRARIES} clingo cbsh2_rtc)
  elseif (solver STREQUAL "EECBS")
    add_definitions(-DSOLVER_EECBS)
    set(EECBS_SOURCES
      solvers/EECBS/src/CBS.cpp
      solvers/EECBS/src/CBSHeuristic.cpp
      solvers/EECBS/src/CBSNode.cpp
      solvers/EECBS/src/common.cpp
      solvers/EECBS/src/Conflict.cpp
      solvers/EECBS/src/ConstraintPropagation.cpp
      solvers/EECBS/src/ConstraintTable.cpp
      solvers/EECBS/src/CorridorReasoning.cpp
      solvers/EECBS/src/ECBS.cpp
      solvers/EECBS/src/IncrementalPairwiseMutexPropagation.cpp
      solvers/EECBS/src/Instance.cpp
      solvers/EECBS/src/MDD.cpp
      solvers/EECBS/src/MutexReasoning.cpp
      solvers/EECBS/src/RectangleReasoning.cpp
      solvers/EECBS/src/ReservationTable.cpp
      solvers/EECBS/src/SingleAgentSolver.cpp
      solvers/EECBS/src/SIPP.cpp
      solvers/EECBS/src/SpaceTimeAStar.cpp
    )
    add_library(eecbs ${EECBS_SOURCES})
    target_include_directories(eecbs PRIVATE solvers/EECBS/inc)
    target_link_libraries(solver ${catkin_LIBRARIES} clingo eecbs)
  elseif (solver STREQUAL "PBS")
    add_definitions(-DSOLVER_PBS)
    set(PBS_SOURCES
      solvers/PBS/src/common.cpp
      solvers/PBS/src/Conflict.cpp
      solvers/PBS/src/ConstraintTable.cpp
      solvers/PBS/src/Instance.cpp
      solvers/PBS/src/PBS.cpp
      solvers/PBS/src/PBSNode.cpp
      solvers/PBS/src/ReservationTable.cpp
      solvers/PBS/src/SingleAgentSolver.cpp
      solvers/PBS/src/SIPP.cpp
      solvers/PBS/src/SpaceTimeAStar.cpp
    )
    add_library(pbs ${PBS_SOURCES})
    target_include_directories(pbs PRIVATE solvers/PBS/inc)
    target_link_libraries(solver ${catkin_LIBRARIES} clingo pbs)
  else()
    set(solver "ASP")
    add_definitions(-DSOLVER_ASP)
    target_link_libraries(solver ${catkin_LIBRARIES} clingo)
  endif()
else()
  set(solver "ASP")
  add_definitions(-DSOLVER_ASP)
  target_link_libraries(solver ${catkin_LIBRARIES} clingo)
endif()

message(STATUS "Using CMAKE_BUILD_TYPE: " ${CMAKE_BUILD_TYPE})
message(STATUS "Using SOLVER: " ${solver})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_dmapf.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
