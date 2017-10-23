cmake_minimum_required(VERSION 2.8.3)
project(opw_kinematics)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED)

find_package(Eigen3 REQUIRED)
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

catkin_package(
  INCLUDE_DIRS include
  DEPENDS EIGEN3
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/test.cpp)

set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
 ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
catkin_add_gtest(${PROJECT_NAME}-test
  test/abb2400_tests.cpp
  test/abb_irb2400_manipulator_ikfast_solver.cpp
)

if(TARGET ${PROJECT_NAME}-test)
  target_compile_definitions(${PROJECT_NAME}-test PUBLIC -DIKFAST_NO_MAIN -DIKFAST_CLIBRARY -DIKFAST_HAS_LIBRARY)
endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
