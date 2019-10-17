find_package(GTest QUIET)
if ( NOT ${GTest_FOUND} )
  include(ExternalProject)

  ExternalProject_Add(GTest
    GIT_REPOSITORY    https://github.com/google/googletest.git
    GIT_TAG           release-1.8.1
    SOURCE_DIR        ${CMAKE_BINARY_DIR}/../${PROJECT_NAME}-googletest-src
    BINARY_DIR        ${CMAKE_BINARY_DIR}/../${PROJECT_NAME}-googletest-build
    CMAKE_CACHE_ARGS
            -DCMAKE_INSTALL_PREFIX:STRING=${CMAKE_INSTALL_PREFIX}/${PROJECT_NAME}
            -DCMAKE_BUILD_TYPE:STRING=Release
            -DBUILD_GMOCK:BOOL=OFF
            -DBUILD_GTEST:BOOL=ON
            -DBUILD_SHARED_LIBS:BOOL=ON
  )

  file(MAKE_DIRECTORY ${CMAKE_INSTALL_PREFIX}/${PROJECT_NAME}/include)
  set(GTEST_INCLUDE_DIRS ${CMAKE_INSTALL_PREFIX}/${PROJECT_NAME}/include)
  set(GTEST_LIBRARIES ${CMAKE_INSTALL_PREFIX}/${PROJECT_NAME}/lib/libgtest.so)
  set(GTEST_MAIN_LIBRARIES ${CMAKE_INSTALL_PREFIX}/${PROJECT_NAME}/lib/libgtest_main.so)
endif()

if(NOT TARGET GTest::GTest)
  find_package(Threads QUIET)

  add_library(GTest::GTest INTERFACE IMPORTED)
  set_target_properties(GTest::GTest PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${GTEST_INCLUDE_DIRS}")
  if(TARGET Threads::Threads)
      set_target_properties(GTest::GTest PROPERTIES INTERFACE_LINK_LIBRARIES "${GTEST_LIBRARIES};Threads::Threads")
  else()
    set_target_properties(GTest::GTest PROPERTIES INTERFACE_LINK_LIBRARIES "${GTEST_LIBRARIES}")
  endif()
endif()

if(NOT TARGET GTest::Main)
  add_library(GTest::Main INTERFACE IMPORTED)
  set_target_properties(GTest::Main PROPERTIES INTERFACE_LINK_LIBRARIES "${GTEST_MAIN_LIBRARIES};GTest::GTest")
endif()

macro(opw_add_test name)
  if(CXX_FEATURE_FOUND EQUAL "-1")
      target_compile_options(${name} PRIVATE -std=c++11)
  else()
      target_compile_features(${name} PRIVATE cxx_std_11)
  endif()
  if(${CMAKE_VERSION} VERSION_LESS "3.10.0")
      gtest_add_tests(${name} "" AUTO)
  else()
      gtest_discover_tests(${name})
  endif()
  if ( NOT ${GTest_FOUND} )
    add_dependencies(${name} GTest ${PROJECT_NAME})
  endif()
  add_dependencies(run_tests ${name})
endmacro()

# Compares the opw solutions with an ikfast generated for the same robot model
add_executable(${PROJECT_NAME}-test-2400-ikfast abb2400_ikfast_tests.cpp abb_irb2400_manipulator_ikfast_solver.cpp)
target_link_libraries(${PROJECT_NAME}-test-2400-ikfast GTest::GTest GTest::Main ${PROJECT_NAME})
target_compile_definitions(${PROJECT_NAME}-test-2400-ikfast PUBLIC -DIKFAST_NO_MAIN -DIKFAST_CLIBRARY -DIKFAST_HAS_LIBRARY)
opw_add_test(${PROJECT_NAME}-test-2400-ikfast)

# Compares a known solution for a robot with varying joint "signs"
add_executable(${PROJECT_NAME}-test-sign-corrections sign_corrections_tests.cpp)
target_link_libraries(${PROJECT_NAME}-test-sign-corrections GTest::GTest GTest::Main ${PROJECT_NAME})
opw_add_test(${PROJECT_NAME}-test-sign-corrections)

# Runs tests that iteratively solve FK then IK then confirm the new FK matches
# Also contains some throughput tests
add_executable(${PROJECT_NAME}-fk-ik fk_ik_tests.cpp)
target_link_libraries(${PROJECT_NAME}-fk-ik GTest::GTest GTest::Main ${PROJECT_NAME})
opw_add_test(${PROJECT_NAME}-fk-ik)
