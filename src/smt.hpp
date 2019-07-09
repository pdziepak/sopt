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

#include <functional>
#include <vector>

#include <fmt/format.h>
#include <fmt/ostream.h>

#include <z3++.h>

class smt_context {
  z3::context& z3_;
  std::vector<std::tuple<z3::expr, std::function<void(uint64_t)>>> unknown_immediates_;
  std::vector<z3::expr> registers_;

public:
  explicit smt_context(z3::context& z3ctx) : z3_(z3ctx), registers_(9, z3_.int_val(0)) {}
  z3::expr get_register(unsigned r) const {
    assert(r < registers_.size());
    return registers_[r];
  }
  void set_register(unsigned r, z3::expr v) {
    assert(r < registers_.size());
    registers_[r] = v;
  }
  z3::expr get_constant(uint64_t v) const { return z3_.bv_val(v, 32); }

  z3::expr add_unknown_immediate(std::function<void(uint64_t)> fn) {
    return std::get<0>(unknown_immediates_.emplace_back(
        z3_.bv_const(fmt::format("imm{}", unknown_immediates_.size()).c_str(), 32), fn));
  }
  void resolve_unknown_immediates(z3::model const& mdl) {
    for (auto& [expr, fn] : unknown_immediates_) {
      auto e = mdl.eval(expr);
      uint64_t v;
      if (e.is_numeral_u64(v)) { fn(e.get_numeral_uint64()); }
    }
  }
};
