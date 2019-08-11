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
#include <unordered_map>
#include <map>
#include <vector>

#include <z3++.h>

namespace uarch {
class uarch;
}
class basic_block;

template<typename Range> z3::expr_vector make_vector(z3::context& ctx, Range&& r) {
  auto expr = z3::expr_vector(ctx);
  for (auto&& e : r) { expr.push_back(e); }
  return expr;
}

template<typename Range> z3::expr make_and(z3::context& ctx, Range&& r) {
  return z3::mk_and(make_vector(ctx, std::forward<Range>(r)));
}

template<typename Range> z3::expr make_or(z3::context& ctx, Range&& r) {
  return z3::mk_or(make_vector(ctx, std::forward<Range>(r)));
}

class smt_context {
  uarch::uarch const& ua_;
  z3::context& z3_;

  std::map<uint64_t, z3::expr> parameters_;

  std::vector<std::tuple<z3::expr, std::function<void(uint64_t)>>> unknown_immediates_;
  std::vector<z3::expr> registers_;

  z3::expr extra_restrictions_;

public:
  using value_type = z3::expr;

  explicit smt_context(uarch::uarch const& ua, z3::context& z3ctx,
                       std::map<uint64_t, z3::expr> const& params = {});

  z3::expr get_register(unsigned r) const;
  void set_register(unsigned r, z3::expr v);

  z3::expr get_parameter(uint64_t p) const;
  z3::expr get_parameter(z3::expr p);

  z3::expr get_constant(uint64_t v) const;

  z3::expr add_unknown_immediate(std::function<void(uint64_t)> fn);
  void resolve_unknown_immediates(z3::model const& mdl);
  z3::expr extra_restrictions() const;

  static z3::expr make_u64(z3::expr const& lo, z3::expr const& hi);
  static std::tuple<z3::expr, z3::expr> split_u64(z3::expr v);
};

std::tuple<std::vector<z3::expr>, z3::expr> emit_smt(uarch::uarch const& ua, z3::context& z3ctx, basic_block& bb,
                                                     std::map<uint64_t, z3::expr> const& params,
                                                     std::vector<std::pair<unsigned, z3::expr>> const& in,
                                                     std::vector<unsigned> const& out);
