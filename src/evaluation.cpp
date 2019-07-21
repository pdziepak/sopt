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

#include "evaluation.hpp"

#include <range/v3/algorithm/any_of.hpp>
#include <range/v3/view.hpp>

#include "instruction.hpp"
#include "uarch/uarch.hpp"

std::ostream& operator<<(std::ostream& os, value const& v) {
  if (v.defined_) {
    fmt::print(os, "{}:i32", v.integer_);
  } else {
    fmt::print(os, "undef");
  }
  return os;
}

evaluation_context::evaluation_context(uarch::uarch const& ua, std::unordered_map<uint64_t, value> const& params)
    : ua_(ua), parameters_(params), registers_(ua.gp_registers()), defined_registers_(ua.gp_registers()) {
}

value evaluation_context::get_register(unsigned r) const {
  if (ua_.zero_gp_register() == r) { return value(0); }
  assert(r < registers_.size());
  assert(defined_registers_[r]);
  return registers_[r];
}
void evaluation_context::set_register(unsigned r, value v) {
  if (ua_.zero_gp_register() == r) { return; }
  assert(r < registers_.size());
  v.trim();
  registers_[r] = v;
  defined_registers_[r] = true;
}
bool evaluation_context::is_register_defined(unsigned r) const {
  if (ua_.zero_gp_register() == r) { return true; }
  return defined_registers_[r];
}

value evaluation_context::get_parameter(value p) const {
  if (p.is_undefined()) { return {}; }
  return parameters_.at(p.as_i32());
}
bool evaluation_context::is_valid_parameter(value p) const {
  if (p.is_undefined()) { return true; }
  return parameters_.count(p.as_i32());
}
bool evaluation_context::is_valid_parameter64(value p) const {
  return is_valid_parameter(p) && is_valid_parameter(p + 4);
}

value evaluation_context::make_u64(value lo, value hi) {
  if (lo.is_undefined() || hi.is_undefined()) { return {}; }
  return value((uint64_t(hi.as_i32()) << 32) | lo.as_i32());
}
std::tuple<value, value> evaluation_context::split_u64(value v) {
  if (v.is_undefined()) { return {{}, {}}; }
  return {value(uint32_t(v.as_i64())), value(uint32_t(v.as_i64() >> 32))};
}

std::optional<std::vector<value>> evaluate(uarch::uarch const& ua, basic_block& bb,
                                           std::unordered_map<uint64_t, value> const& params,
                                           std::vector<std::pair<unsigned, value>> const& in,
                                           std::vector<unsigned> const& out) {
  auto ctx = evaluation_context(ua, params);
  for (auto [reg, val] : in) { ctx.set_register(reg, val); }
  for (auto& inst : bb.instructions_) {
    if (!inst.opcode_->evaluate(ctx, inst.operands_)) { return {}; }
  }
  if (ranges::any_of(out, [&](unsigned reg) { return !ctx.is_register_defined(reg); })) { return {}; }
  return out | ranges::view::transform([&](unsigned reg) { return ctx.get_register(reg); }) | ranges::to<std::vector>();
}
