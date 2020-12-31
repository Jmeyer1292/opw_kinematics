cmake_minimum_required(VERSION 3.5.0)

# Extract package name and version
find_package(cmake_common_scripts REQUIRED)
extract_package_metadata(pkg)
project(${pkg_extracted_name} VERSION ${pkg_extracted_version} LANGUAGES CXX)

find_package(Eigen3 REQUIRED)
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

# If testing is enabled the clang tidy will errors instead of warning. The -line-filter option is added so the two files
# ikfast.h and abb_irb2400_manipulator_ikfast_solver.cpp are not processed by clang tidy since these are auto generated
# file from ikfast.
if(OPW_ENABLE_TESTING)
  set(CLANG_ARGS "-line-filter=[{'name':'ikfast.h','lines':[[9999999,9999999]]}, {'name':'abb_irb2400_manipulator_ikfast_solver.cpp','lines':[[9999999,9999999]]}, {'name':'.h'}, {'name':'.hpp'}, {'name':'.cpp'}]" ${DEFAULT_CLANG_TIDY_ERROR_ARGS})
else()
  set(OPW_ENABLE_TESTING OFF) # Since this is being passed to the macros it must have a value otherwise it causes issues
  set(CLANG_ARGS "-line-filter=[{'name':'ikfast.h','lines':[[9999999,9999999]]}, {'name':'abb_irb2400_manipulator_ikfast_solver.cpp','lines':[[9999999,9999999]]}, {'name':'.h'}, {'name':'.hpp'}, {'name':'.cpp'}]" ${DEFAULT_CLANG_TIDY_WARNING_ARGS})
endif()

if (NOT DEFINED OPW_ENABLE_RUN_TESTING)
  set(OPW_ENABLE_RUN_TESTING OFF)
endif()

add_code_coverage_all_targets(ENABLE ${OPW_ENABLE_TESTING})

# Create interface target
add_library(${PROJECT_NAME} INTERFACE)
target_compile_definitions(${PROJECT_NAME} INTERFACE _USE_MATH_DEFINES=ON)
target_link_libraries(${PROJECT_NAME} INTERFACE)
target_cxx_version(${PROJECT_NAME} INTERFACE VERSION 11)

if(NOT MSVC)
  target_compile_options(${PROJECT_NAME} INTERFACE -Wall -Wextra)
  target_clang_tidy(${PROJECT_NAME} ENABLE ${OPW_ENABLE_TESTING} ARGUMENTS ${CLANG_ARGS})
  target_code_coverage(${PROJECT_NAME} INTERFACE ALL ENABLE ${OPW_ENABLE_TESTING})
endif()

target_include_directories(${PROJECT_NAME} INTERFACE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME} SYSTEM INTERFACE
    ${EIGEN3_INCLUDE_DIRS})

# Create test executable
add_executable(${PROJECT_NAME}_node src/test.cpp)
target_link_libraries(${PROJECT_NAME}_node PRIVATE ${PROJECT_NAME})
set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")
target_cxx_version(${PROJECT_NAME}_node PRIVATE VERSION 11)

if(NOT MSVC)
  target_compile_options(${PROJECT_NAME}_node PRIVATE -Wall -Wextra)
  target_clang_tidy(${PROJECT_NAME}_node ENABLE ${OPW_ENABLE_TESTING} ARGUMENTS ${CLANG_ARGS})
  target_include_what_you_use(${PROJECT_NAME}_node ENABLE ${OPW_ENABLE_TESTING} ARGUMENTS ${DEFAULT_IWYU_ARGS})
  target_cppcheck(${PROJECT_NAME}_node ENABLE ${OPW_ENABLE_TESTING} ARGUMENTS ${DEFAULT_CPPCHECK_ARGS})
  target_code_coverage(${PROJECT_NAME}_node ALL ENABLE ${OPW_ENABLE_TESTING})
endif()

#############
## Install ##
#############
configure_package(NAMESPACE opw_kinematics TARGETS ${PROJECT_NAME})

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp" PATTERN "*.hxx"
  PATTERN ".svn" EXCLUDE
 )

#############
## Testing ##
#############
if (OPW_ENABLE_TESTING)
  enable_testing()
  add_run_tests_target(ENABLE OPW_ENABLE_RUN_TESTING)
  add_subdirectory(test)
endif()
