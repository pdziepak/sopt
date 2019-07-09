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

add_executable(test-runner
  test-runner.cpp
)
target_link_libraries(test-runner PRIVATE sopt-lib)

function(sopt_add_func_test SUITE CASE)
  add_test(NAME ${SUITE}-${CASE} COMMAND test-runner ${CMAKE_CURRENT_SOURCE_DIR}/${CASE}.txt)
endfunction(sopt_add_func_test)

add_subdirectory(example)
