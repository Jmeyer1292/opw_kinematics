cmake_minimum_required(VERSION 3.5.0)

# Extract package name and version
find_package(ros_industrial_cmake_boilerplate REQUIRED)
extract_package_metadata(pkg)
project(${pkg_extracted_name} VERSION ${pkg_extracted_version} LANGUAGES CXX)

include(cmake/opw_kinematics_macros.cmake)
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/")

find_package(Eigen3 REQUIRED)
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

# Load variable for clang tidy args, compiler options and cxx version
opw_variables()

if(NOT MSVC)
  initialize_code_coverage()
  add_code_coverage_all_targets(ENABLE ${OPW_ENABLE_CODE_COVERAGE})
endif()

# Create interface target
add_library(${PROJECT_NAME} INTERFACE)
target_compile_definitions(${PROJECT_NAME} INTERFACE ${OPW_COMPILE_DEFINITIONS})
target_link_libraries(${PROJECT_NAME} INTERFACE)
target_cxx_version(${PROJECT_NAME} INTERFACE VERSION ${OPW_CXX_VERSION})
target_compile_options(${PROJECT_NAME} INTERFACE ${OPW_COMPILE_OPTIONS_PUBLIC})
target_clang_tidy(${PROJECT_NAME} ENABLE ${OPW_ENABLE_CLANG_TIDY} ARGUMENTS ${OPW_CLANG_TIDY_ARGS})
target_code_coverage(${PROJECT_NAME} INTERFACE ALL ENABLE ${OPW_ENABLE_CODE_COVERAGE})
target_include_directories(${PROJECT_NAME} INTERFACE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME} SYSTEM INTERFACE
    ${EIGEN3_INCLUDE_DIRS})

# Create test executable
add_executable(${PROJECT_NAME}_node src/test.cpp)
target_link_libraries(${PROJECT_NAME}_node PRIVATE ${PROJECT_NAME})
set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")
target_cxx_version(${PROJECT_NAME}_node PRIVATE VERSION ${OPW_CXX_VERSION})
target_compile_options(${PROJECT_NAME}_node PRIVATE ${OPW_COMPILE_OPTIONS_PRIVATE})
target_clang_tidy(${PROJECT_NAME}_node ENABLE ${OPW_ENABLE_CLANG_TIDY} ARGUMENTS ${OPW_CLANG_TIDY_ARGS})
target_include_what_you_use(${PROJECT_NAME}_node ENABLE ${OPW_ENABLE_TESTING} ARGUMENTS ${DEFAULT_IWYU_ARGS})
target_cppcheck(${PROJECT_NAME}_node ENABLE ${OPW_ENABLE_TESTING} ARGUMENTS ${DEFAULT_CPPCHECK_ARGS})
target_code_coverage(${PROJECT_NAME}_node ALL ENABLE ${OPW_ENABLE_CODE_COVERAGE})

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

install(FILES
  "${CMAKE_CURRENT_LIST_DIR}/cmake/opw_kinematics_macros.cmake"
  DESTINATION lib/cmake/${PROJECT_NAME})

#############
## Testing ##
#############
if (OPW_ENABLE_TESTING)
  enable_testing()
  add_run_tests_target(ENABLE ${OPW_ENABLE_RUN_TESTING})
  add_subdirectory(test)
endif()
