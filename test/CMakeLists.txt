find_gtest()

macro(opw_add_test name)
  target_cxx_version(${name} PUBLIC VERSION ${OPW_CXX_VERSION})
  target_clang_tidy(${name} ENABLE ${OPW_ENABLE_CLANG_TIDY} ARGUMENTS ${OPW_CLANG_TIDY_ARGS})
  target_include_what_you_use(${name} ENABLE ${OPW_ENABLE_TESTING} ARGUMENTS ${DEFAULT_IWYU_ARGS})
  target_cppcheck(${name} ENABLE ${OPW_ENABLE_TESTING} ARGUMENTS ${DEFAULT_CPPCHECK_ARGS})
  add_gtest_discover_tests(${name})
  add_dependencies(${name} ${PROJECT_NAME})
  add_dependencies(run_tests ${name})

  if(NOT ${GTest_FOUND})
    add_dependencies(${name} GTest ${PROJECT_NAME})
  endif()
endmacro()

# Compares the opw solutions with an ikfast generated for the same robot model
add_executable(${PROJECT_NAME}-test-2400-ikfast abb2400_ikfast_tests.cpp abb_irb2400_manipulator_ikfast_solver.cpp)
target_link_libraries(${PROJECT_NAME}-test-2400-ikfast PRIVATE GTest::GTest GTest::Main ${PROJECT_NAME})
target_compile_definitions(${PROJECT_NAME}-test-2400-ikfast PUBLIC -DIKFAST_NO_MAIN -DIKFAST_CLIBRARY
                                                                   -DIKFAST_HAS_LIBRARY)
opw_add_test(${PROJECT_NAME}-test-2400-ikfast)

# Compares a known solution for a robot with varying joint "signs"
add_executable(${PROJECT_NAME}-test-sign-corrections sign_corrections_tests.cpp)
target_link_libraries(${PROJECT_NAME}-test-sign-corrections PRIVATE GTest::GTest GTest::Main ${PROJECT_NAME})
target_compile_options(${PROJECT_NAME}-test-sign-corrections PRIVATE ${OPW_COMPILE_OPTIONS_PRIVATE})
opw_add_test(${PROJECT_NAME}-test-sign-corrections)

# Runs tests that iteratively solve FK then IK then confirm the new FK matches Also contains some throughput tests
add_executable(${PROJECT_NAME}-fk-ik fk_ik_tests.cpp)
target_link_libraries(${PROJECT_NAME}-fk-ik PRIVATE GTest::GTest GTest::Main ${PROJECT_NAME})
target_compile_options(${PROJECT_NAME}-fk-ik PRIVATE ${OPW_COMPILE_OPTIONS_PRIVATE})
opw_add_test(${PROJECT_NAME}-fk-ik)
