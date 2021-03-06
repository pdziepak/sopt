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

smt_context::smt_context(uarch::uarch const& ua, z3_context& z3ctx, std::map<uint64_t, z3::expr> const& params)
    : ua_(ua), z3_(z3ctx), parameters_(params), registers_(ua.lanes()), extra_restrictions_(z3_.ctx_.bool_val(true)) {
  for (auto& rs : registers_) { rs.resize(ua.gp_registers(), get_constant(0)); }
}

z3::expr smt_context::get_register(unsigned r, unsigned lane) const {
  if (ua_.zero_gp_register() == r) { return get_constant(0); }
  assert(r < ua_.gp_registers());
  return registers_[lane % ua_.lanes()][r];
}
z3::expr smt_context::get_register(unsigned r, z3::expr lane) {
  if (ua_.zero_gp_register() == r) { return get_constant(0); }
  uint64_t l;
  lane = z3::urem(lane, ua_.lanes()).simplify();
  if (lane.is_numeral_u64(l)) {
    assert(l < ua_.lanes());
    return registers_[l][r];
  }
  auto current = registers_[0][r];
  for (auto l = 1u; l < ua_.lanes(); ++l) {
    current = z3::ite(lane == get_constant(l), registers_[l][r], std::move(current));
  }
  return current;
}
void smt_context::set_register(unsigned r, z3::expr v, unsigned lane) {
  if (ua_.zero_gp_register() == r) { return; }
  assert(r < ua_.gp_registers());
  pending_register_writes_.emplace_back(lane, r, std::move(v));
}

void smt_context::commit_pending_operations() {
  for (auto& [lane, reg, expr] : pending_register_writes_) {
    assert(lane < ua_.lanes());
    assert(reg < ua_.gp_registers());
    registers_[lane][reg] = std::move(expr);
  }
  pending_register_writes_.clear();
}

z3::expr smt_context::get_parameter(uint64_t p) const {
  return parameters_.at(p);
}
z3::expr smt_context::get_parameter(z3::expr p) {
  auto current = get_constant(0); // TODO: poison?
  for (auto [k, v] : parameters_) { current = z3::ite(p == get_constant(k), v, std::move(current)); }
  extra_restrictions_ = std::move(extra_restrictions_) &&
                        make_or(z3_.ctx_, parameters_ | ranges::view::keys | ranges::view::transform([&](uint64_t k) {
                                            return p == get_constant(k);
                                          }));
  return current;
}

static thread_local std::unordered_map<uint64_t, z3::expr> cached_constants;

z3::expr smt_context::get_constant(uint64_t v) const {
  auto it = z3_.cached_constants_.find(v);
  if (it == z3_.cached_constants_.end()) {
    auto ex = z3_.ctx_.bv_val(v, 32);
    z3_.cached_constants_.emplace(v, ex);
    return ex;
  }
  return it->second;
}

z3::expr smt_context::add_unknown_immediate(void* id, std::function<void(uint64_t)> fn) {
  auto it = unique_unknown_immediates_.find(id);
  if (it != unique_unknown_immediates_.end()) { return it->second; }
  auto expr = std::get<0>(unknown_immediates_.emplace_back(
      z3_.ctx_.bv_const(fmt::format("imm{}", unknown_immediates_.size()).c_str(), 32), fn));
  unique_unknown_immediates_.insert(std::pair(id, expr));
  return expr;
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

std::tuple<std::vector<z3::expr>, z3::expr> emit_smt(uarch::uarch const& ua, z3_context& z3ctx, basic_block& bb,
                                                     std::map<uint64_t, z3::expr> const& params,
                                                     std::vector<std::pair<unsigned, std::vector<z3::expr>>> const& in,
                                                     std::vector<unsigned> const& out) {
  // FIXME: const correctness

  auto ctx = smt_context(ua, z3ctx, params);
  for (auto [reg, expr] : in) {
    for (auto lane = 0u; lane < ua.lanes(); ++lane) { ctx.set_register(reg, expr[lane], lane); }
  }
  ctx.commit_pending_operations();
  for (auto& inst : bb.instructions_) {
    for (auto lane = 0u; lane < ua.lanes(); ++lane) { inst.opcode_->emit_smt(ctx, lane, inst.operands_); }
    ctx.commit_pending_operations();
  }
  return {out | ranges::view::transform([&](unsigned reg) { return ctx.get_register(reg, 0).simplify(); }) |
              ranges::to<std::vector>(),
          ctx.extra_restrictions()};
}
