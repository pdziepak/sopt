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

#include <random>

#include "../stats.hpp"

#include "target.hpp"

namespace optimizer {

class path {
  std::default_random_engine* prng_;
  z3::solver* z3slv_;

  target* target_;

  basic_block current_;
  double current_score_;

private:
  bool check_tests(basic_block& candidate);
  void synthesize_immediates(basic_block& candidate, stats& st);

public:
  path(std::default_random_engine& prng, z3::solver& z3slv, target& trgt);

  void next_step(stats& st);

  basic_block const& current() const { return current_; }
  double current_score() const { return current_score_; }
};

} // namespace optimizer
