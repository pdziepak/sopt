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

#include "smt.hpp"

#include <range/v3/view.hpp>

#include "instruction.hpp"
#include "uarch/uarch.hpp"

smt_context::smt_context(uarch::uarch const& ua, z3::context& z3ctx,
                         std::unordered_map<uint64_t, z3::expr> const& params)
    : ua_(ua), z3_(z3ctx), parameters_(params), registers_(ua.gp_registers(), z3_.int_val(0)),
      extra_restrictions_(z3_.bool_val(true)) {
}

z3::expr smt_context::get_register(unsigned r) const {
  if (ua_.zero_gp_register() == r) { return get_constant(0); }
  assert(r < registers_.size());
  return registers_[r];
}
void smt_context::set_register(unsigned r, z3::expr v) {
  if (ua_.zero_gp_register() == r) { return; }
  assert(r < registers_.size());
  registers_[r] = v;
}

z3::expr smt_context::get_parameter(uint64_t p) const {
  return parameters_.at(p);
}
z3::expr smt_context::get_parameter(z3::expr p) {
  auto current = get_constant(0); // TODO: poison?
  for (auto [k, v] : parameters_) { current = z3::ite(p == get_constant(k), v, std::move(current)); }
  extra_restrictions_ = std::move(extra_restrictions_) &&
                        make_or(z3_, parameters_ | ranges::view::keys |
                                         ranges::view::transform([&](uint64_t k) { return p == get_constant(k); }));
  return current;
}

z3::expr smt_context::get_constant(uint64_t v) const {
  return z3_.bv_val(v, 32);
}

z3::expr smt_context::add_unknown_immediate(std::function<void(uint64_t)> fn) {
  return std::get<0>(
      unknown_immediates_.emplace_back(z3_.bv_const(fmt::format("imm{}", unknown_immediates_.size()).c_str(), 32), fn));
}
void smt_context::resolve_unknown_immediates(z3::model const& mdl) {
  for (auto& [expr, fn] : unknown_immediates_) {
    auto e = mdl.eval(expr);
    uint64_t v;
    if (e.is_numeral_u64(v)) { fn(e.get_numeral_uint64()); }
  }
}

z3::expr smt_context::extra_restrictions() const {
  return extra_restrictions_;
}

z3::expr smt_context::make_u64(z3::expr const& lo, z3::expr const& hi) {
  return z3::shl(z3::zext(hi, 32), 32) | z3::zext(lo, 32);
}
std::tuple<z3::expr, z3::expr> smt_context::split_u64(z3::expr v) {
  return {v.extract(31, 0), v.extract(63, 32)};
}

std::tuple<std::vector<z3::expr>, z3::expr> emit_smt(uarch::uarch const& ua, z3::context& z3ctx, basic_block& bb,
                                                     std::unordered_map<uint64_t, z3::expr> const& params,
                                                     std::vector<std::pair<unsigned, z3::expr>> const& in,
                                                     std::vector<unsigned> const& out) {
  // FIXME: const correctness

  auto ctx = smt_context(ua, z3ctx, params);
  for (auto [reg, expr] : in) { ctx.set_register(reg, expr); }
  for (auto& inst : bb.instructions_) { inst.opcode_->emit_smt(ctx, inst.operands_); }
  return {out | ranges::view::transform([&](unsigned reg) { return ctx.get_register(reg).simplify(); }) |
              ranges::to<std::vector>(),
          ctx.extra_restrictions()};
}
