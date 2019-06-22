find_package(GTest REQUIRED)
find_package(opw_kinematics REQUIRED)

macro(opw_add_test name)
    if(${CMAKE_VERSION} VERSION_LESS "3.10.0")
        gtest_add_tests(${name})
    else()
        gtest_discover_tests(${name})
    endif()
endmacro()

# Compares the opw solutions with an ikfast generated for the same robot model
add_executable(${PROJECT_NAME}-test-2400-ikfast abb2400_ikfast_tests.cpp abb_irb2400_manipulator_ikfast_solver.cpp)
target_link_libraries(${PROJECT_NAME}-test-2400-ikfast ${GTEST_BOTH_LIBRARIES} opw_kinematics::${PROJECT_NAME})
target_compile_definitions(${PROJECT_NAME}-test-2400-ikfast PUBLIC -DIKFAST_NO_MAIN -DIKFAST_CLIBRARY -DIKFAST_HAS_LIBRARY)
target_compile_features(${PROJECT_NAME}-test-2400-ikfast PRIVATE cxx_std_11)
target_include_directories(${PROJECT_NAME}-test-2400-ikfast PRIVATE ${GTEST_INCLUDE_DIRS})
opw_add_test(${PROJECT_NAME}-test-2400-ikfast)
add_dependencies(run_tests ${PROJECT_NAME}-test-2400-ikfast)

# Compares a known solution for a robot with varying joint "signs"
add_executable(${PROJECT_NAME}-test-sign-corrections sign_corrections_tests.cpp)
target_link_libraries(${PROJECT_NAME}-test-sign-corrections ${GTEST_BOTH_LIBRARIES} opw_kinematics::${PROJECT_NAME})
target_compile_features(${PROJECT_NAME}-test-sign-corrections PRIVATE cxx_std_11)
target_include_directories(${PROJECT_NAME}-test-sign-corrections PRIVATE ${GTEST_INCLUDE_DIRS})
opw_add_test(${PROJECT_NAME}-test-sign-corrections)
add_dependencies(run_tests ${PROJECT_NAME}-test-sign-corrections)

# Runs tests that iteratively solve FK then IK then confirm the new FK matches
# Also contains some throughput tests
add_executable(${PROJECT_NAME}-fk-ik fk_ik_tests.cpp)
target_link_libraries(${PROJECT_NAME}-fk-ik ${GTEST_BOTH_LIBRARIES} opw_kinematics::${PROJECT_NAME})
target_compile_features(${PROJECT_NAME}-fk-ik PRIVATE cxx_std_11)
target_include_directories(${PROJECT_NAME}-fk-ik PRIVATE ${GTEST_INCLUDE_DIRS})
opw_add_test(${PROJECT_NAME}-fk-ik)
add_dependencies(run_tests ${PROJECT_NAME}-fk-ik)
