cmake_minimum_required(VERSION 2.8.3)
project(opw_kinematics)

add_compile_options(-std=c++11 -Wall -Wextra)

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

if(CATKIN_ENABLE_TESTING)

  # Compares the opw solutions with an ikfast generated for the same robot model
  catkin_add_gtest(${PROJECT_NAME}-test-2400-ikfast
    test/abb2400_ikfast_tests.cpp
    test/abb_irb2400_manipulator_ikfast_solver.cpp
  )
  target_compile_definitions(${PROJECT_NAME}-test-2400-ikfast PUBLIC -DIKFAST_NO_MAIN
    -DIKFAST_CLIBRARY -DIKFAST_HAS_LIBRARY)

  # Compares a known solution for a robot with varying joint "signs"
  catkin_add_gtest(${PROJECT_NAME}-test-sign-corrections test/sign_corrections_tests.cpp)

  # Runs tests that iteratively solve FK then IK then confirm the new FK matches
  # Also contains some throughput tests
  catkin_add_gtest(${PROJECT_NAME}-fk-ik test/fk_ik_tests.cpp)

endif() # end catkin_enable_testing
