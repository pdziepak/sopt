/*
 * Copyright © 2019 Paweł Dziepak
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include "instruction.hpp"
#include "uarch/uarch.hpp"

struct interface {
  std::vector<unsigned> input_registers;
  std::vector<unsigned> output_registers;
};

basic_block optimize(uarch::uarch const& ua, interface const&, basic_block, double score);

double score_performance(basic_block const& bb);

bool equivalent(interface const&, basic_block const& a, basic_block const& b);
