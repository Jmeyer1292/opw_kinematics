# ~~~
# @file opw_kinematics_macros.cmake
# @brief Common OPW Kinematics CMake Macros
#
# @author Levi Armstrong
# @date February 17, 2021
# @version TODO @bug No known bugs
#
# @copyright Copyright (c) 2021, Southwest Research Institute
#
# @par License Software License Agreement (Apache License) @par Licensed under the Apache License, Version 2.0 (the
# "License"); you may not use this file except in compliance with the License. You may obtain a copy of the License at
# http://www.apache.org/licenses/LICENSE-2.0 @par Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either
# express or implied. See the License for the specific language governing permissions and limitations under the License.
# ~~~
macro(opw_variables)
  if(NOT DEFINED BUILD_SHARED_LIBS)
    set(BUILD_SHARED_LIBS ON)
  endif()

  if(NOT DEFINED OPW_ENABLE_CLANG_TIDY)
    set(OPW_ENABLE_CLANG_TIDY OFF)
  endif()

  if(NOT DEFINED OPW_ENABLE_CODE_COVERAGE)
    set(OPW_ENABLE_CODE_COVERAGE OFF)
  elseif(OPW_ENABLE_CODE_COVERAGE)
    set(OPW_ENABLE_TESTING ON)
  endif()

  if(NOT DEFINED OPW_ENABLE_TESTING)
    set(OPW_ENABLE_TESTING OFF)
  endif()

  if(NOT DEFINED OPW_ENABLE_RUN_TESTING)
    set(OPW_ENABLE_RUN_TESTING OFF)
  endif()

  if(OPW_ENABLE_TESTING_ALL)
    set(OPW_ENABLE_TESTING ON)
    set(OPW_ENABLE_CLANG_TIDY ON)
    set(OPW_ENABLE_CODE_COVERAGE ON)
  endif()

  set(OPW_COMPILE_DEFINITIONS "")
  set(OPW_COMPILE_OPTIONS_PUBLIC "")
  set(OPW_COMPILE_OPTIONS_PRIVATE "")
  if(NOT OPW_ENABLE_TESTING AND NOT OPW_ENABLE_TESTING_ALL)
    if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
      set(OPW_COMPILE_OPTIONS_PRIVATE
          -Wall
          -Wextra
          -Wconversion
          -Wsign-conversion
          -Wno-sign-compare
          -Wnon-virtual-dtor)
      exec_program(uname ARGS -p OUTPUT_VARIABLE CMAKE_SYSTEM_NAME2)
      if(NOT
         CMAKE_SYSTEM_NAME2
         MATCHES
         "aarch64"
         AND NOT
             CMAKE_SYSTEM_NAME2
             MATCHES
             "armv7l"
         AND NOT
             CMAKE_SYSTEM_NAME2
             MATCHES
             "unknown")
        set(OPW_COMPILE_OPTIONS_PUBLIC -mno-avx)
      endif()
    elseif(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
      set(OPW_COMPILE_OPTIONS_PRIVATE
          -Wall
          -Wextra
          -Wconversion
          -Wsign-conversion)
      message(WARNING "Non-GNU compiler detected. If using AVX instructions, Eigen alignment issues may result.")
    elseif(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
      set(OPW_COMPILE_DEFINITIONS "_USE_MATH_DEFINES=ON")
      message(WARNING "Non-GNU compiler detected. If using AVX instructions, Eigen alignment issues may result.")
    else()
      message(WARNING "${CMAKE_CXX_COMPILER_ID} Unsupported compiler detected.")
    endif()
  else()
    if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
      set(OPW_COMPILE_OPTIONS_PRIVATE
          -Werror=all
          -Werror=extra
          -Werror=conversion
          -Werror=sign-conversion
          -Wno-sign-compare
          -Werror=non-virtual-dtor)
      exec_program(uname ARGS -p OUTPUT_VARIABLE CMAKE_SYSTEM_NAME2)
      if(NOT
         CMAKE_SYSTEM_NAME2
         MATCHES
         "aarch64"
         AND NOT
             CMAKE_SYSTEM_NAME2
             MATCHES
             "armv7l"
         AND NOT
             CMAKE_SYSTEM_NAME2
             MATCHES
             "unknown")
        set(OPW_COMPILE_OPTIONS_PUBLIC -mno-avx)
      endif()
    elseif(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
      set(OPW_COMPILE_OPTIONS_PRIVATE
          -Werror=all
          -Werror=extra
          -Werror=conversion
          -Werror=sign-conversion)
      message(WARNING "Non-GNU compiler detected. If using AVX instructions, Eigen alignment issues may result.")
    elseif(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
      set(OPW_COMPILE_DEFINITIONS "_USE_MATH_DEFINES=ON")
      message(WARNING "Non-GNU compiler detected. If using AVX instructions, Eigen alignment issues may result.")
    else()
      message(WARNING "${CMAKE_CXX_COMPILER_ID} Unsupported compiler detected.")
    endif()
  endif()

  set(OPW_CXX_VERSION 11)
endmacro()
