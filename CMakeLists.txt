#
# Copyright © 2019 Paweł Dziepak
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.12)

project(sopt
  LANGUAGES CXX
)

include(CTest)

option(SOPT_SANITIZERS "Build with AddressSanitizer and UndefinedBehaviorSanitizer." ON)

list(APPEND SOPT_CXX_FLAGS -Wall -Wextra -Werror -g)
if (SOPT_SANITIZERS)
  list(APPEND SOPT_CXX_DEBUG_FLAGS -fsanitize=address,undefined)
endif()

if(CMAKE_BUILD_TYPE STREQUAL Debug)
  list(APPEND SOPT_CXX_FLAGS ${SOPT_CXX_DEBUG_FLAGS})
else()
  list(APPEND SOPT_CXX_FLAGS ${SOPT_CXX_RELEASE_FLAGS})
endif()

find_package(Z3 REQUIRED)
find_package(fmt REQUIRED)
find_package(spdlog REQUIRED)

add_library(sopt-lib
  src/optimizer/path.cpp
  src/optimizer/target.cpp

  src/uarch/example.cpp

  src/evaluation.cpp
  src/mutate.cpp
  src/optimizer.cpp
  src/perf.cpp
  src/smt.cpp
)
target_include_directories(sopt-lib PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)
target_compile_features(sopt-lib PUBLIC cxx_std_17)
target_link_options(sopt-lib PUBLIC ${SOPT_CXX_FLAGS})
target_compile_options(sopt-lib PUBLIC ${SOPT_CXX_FLAGS})
target_link_libraries(sopt-lib PUBLIC
  spdlog::spdlog
  fmt::fmt
  z3::libz3
)

if(BUILD_TESTING)
  add_subdirectory(tests)
endif()
