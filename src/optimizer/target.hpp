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

#include "../instruction.hpp"
#include "../optimizer.hpp"
#include "../uarch/uarch.hpp"
#include "detail.hpp"

namespace optimizer {

class target {
  uarch::uarch const& ua_;
  interface ifce_;
  std::vector<test> tests_;

  basic_block target_;

  z3::context z3ctx_;

  std::vector<std::pair<unsigned, std::vector<z3::expr>>> in_exprs_;
  std::map<uint64_t, z3::expr> param_exprs_;

  std::vector<z3::expr> target_smt_;
  z3::expr target_extra_smt_;

  basic_block best_;
  double best_score_;

  // FIXME: proper encapsulation
  friend class path;

public:
  target(uarch::uarch const& ua, interface const& ifce, basic_block trgt);

  z3::context& z3_context() { return z3ctx_; }

  double best_score() const { return best_score_; }
  basic_block const& best() const { return best_; }
};

} // namespace optimizer
