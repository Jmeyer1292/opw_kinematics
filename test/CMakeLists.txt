find_package(GTest QUIET)
if ( NOT ${GTest_FOUND} )
    include(ExternalProject)

    ExternalProject_Add(GTest
      GIT_REPOSITORY    https://github.com/google/googletest.git
      GIT_TAG           release-1.8.1
      SOURCE_DIR        ${CMAKE_BINARY_DIR}/../googletest-src
      BINARY_DIR        ${CMAKE_BINARY_DIR}/../googletest-build
      CMAKE_CACHE_ARGS
              -DCMAKE_INSTALL_PREFIX:STRING=${CMAKE_INSTALL_PREFIX}
              -DCMAKE_BUILD_TYPE:STRING=Release
              -DBUILD_GMOCK:BOOL=OFF
              -DBUILD_GTEST:BOOL=ON
              -DBUILD_SHARED_LIBS:BOOL=ON
    )

    set(GTEST_INCLUDE_DIRS ${CMAKE_INSTALL_PREFIX}/include)
    set(GTEST_BOTH_LIBRARIES ${CMAKE_INSTALL_PREFIX}/lib/libgtest.so)
endif()

macro(opw_add_test name)
    if(${CMAKE_VERSION} VERSION_LESS "3.10.0")
        gtest_add_tests(${name} "" AUTO)
    else()
        gtest_discover_tests(${name})
    endif()
endmacro()

# Compares the opw solutions with an ikfast generated for the same robot model
add_executable(${PROJECT_NAME}-test-2400-ikfast abb2400_ikfast_tests.cpp abb_irb2400_manipulator_ikfast_solver.cpp)
target_link_libraries(${PROJECT_NAME}-test-2400-ikfast ${GTEST_BOTH_LIBRARIES} ${PROJECT_NAME})
target_compile_definitions(${PROJECT_NAME}-test-2400-ikfast PUBLIC -DIKFAST_NO_MAIN -DIKFAST_CLIBRARY -DIKFAST_HAS_LIBRARY)
if(CXX_FEATURE_FOUND EQUAL "-1")
    target_compile_options(${PROJECT_NAME}-test-2400-ikfast PRIVATE -std=c++11)
else()
    target_compile_features(${PROJECT_NAME}-test-2400-ikfast PRIVATE cxx_std_11)
endif()
target_include_directories(${PROJECT_NAME}-test-2400-ikfast PRIVATE ${GTEST_INCLUDE_DIRS})
opw_add_test(${PROJECT_NAME}-test-2400-ikfast)
add_dependencies(${PROJECT_NAME}-test-2400-ikfast GTest ${PROJECT_NAME})
add_dependencies(run_tests ${PROJECT_NAME}-test-2400-ikfast)

# Compares a known solution for a robot with varying joint "signs"
add_executable(${PROJECT_NAME}-test-sign-corrections sign_corrections_tests.cpp)
target_link_libraries(${PROJECT_NAME}-test-sign-corrections ${GTEST_BOTH_LIBRARIES} ${PROJECT_NAME})
if(CXX_FEATURE_FOUND EQUAL "-1")
    target_compile_options(${PROJECT_NAME}-test-sign-corrections PRIVATE -std=c++11)
else()
    target_compile_features(${PROJECT_NAME}-test-sign-corrections PRIVATE cxx_std_11)
endif()
target_include_directories(${PROJECT_NAME}-test-sign-corrections PRIVATE ${GTEST_INCLUDE_DIRS})
opw_add_test(${PROJECT_NAME}-test-sign-corrections)
add_dependencies(${PROJECT_NAME}-test-sign-corrections GTest ${PROJECT_NAME})
add_dependencies(run_tests ${PROJECT_NAME}-test-sign-corrections)

# Runs tests that iteratively solve FK then IK then confirm the new FK matches
# Also contains some throughput tests
add_executable(${PROJECT_NAME}-fk-ik fk_ik_tests.cpp)
target_link_libraries(${PROJECT_NAME}-fk-ik ${GTEST_BOTH_LIBRARIES} ${PROJECT_NAME})
if(CXX_FEATURE_FOUND EQUAL "-1")
    target_compile_options(${PROJECT_NAME}-fk-ik PRIVATE -std=c++11)
else()
    target_compile_features(${PROJECT_NAME}-fk-ik PRIVATE cxx_std_11)
endif()
target_include_directories(${PROJECT_NAME}-fk-ik PRIVATE ${GTEST_INCLUDE_DIRS})
opw_add_test(${PROJECT_NAME}-fk-ik)
add_dependencies(${PROJECT_NAME}-fk-ik GTest ${PROJECT_NAME})
add_dependencies(run_tests ${PROJECT_NAME}-fk-ik)
