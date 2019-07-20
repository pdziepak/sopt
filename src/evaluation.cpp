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

evaluation_context::evaluation_context(std::unordered_map<uint64_t, uint64_t> const& params)
    : parameters_(params), registers_(9), defined_registers_(9) {
}

uint64_t evaluation_context::get_register(unsigned r) const {
  assert(r < registers_.size());
  assert(defined_registers_[r]);
  return registers_[r];
}
void evaluation_context::set_register(unsigned r, uint64_t v) {
  assert(r < registers_.size());
  registers_[r] = v;
  defined_registers_[r] = true;
}
bool evaluation_context::is_register_defined(unsigned r) const {
  return defined_registers_[r];
}

uint64_t evaluation_context::get_parameter(uint64_t p) const {
  return parameters_.at(p);
}
bool evaluation_context::is_valid_parameter(uint64_t p) const {
  return parameters_.count(p);
}
bool evaluation_context::is_valid_parameter64(uint64_t p) const {
  return is_valid_parameter(p) && is_valid_parameter(p + 4);
}

uint64_t evaluation_context::make_u64(uint32_t lo, uint32_t hi) {
  return (uint64_t(hi) << 32) | lo;
}
std::tuple<uint32_t, uint32_t> evaluation_context::split_u64(uint64_t v) {
  return {uint32_t(v), uint32_t(v >> 32)};
}

std::optional<std::vector<uint64_t>> evaluate(basic_block& bb, std::unordered_map<uint64_t, uint64_t> const& params,
                                              std::vector<std::pair<unsigned, uint64_t>> in,
                                              std::vector<unsigned> const& out) {
  auto ctx = evaluation_context(params);
  for (auto [reg, val] : in) { ctx.set_register(reg, val); }
  for (auto& inst : bb.instructions_) {
    if (!inst.opcode_->evaluate(ctx, inst.operands_)) { return {}; }
  }
  if (ranges::any_of(out, [&](unsigned reg) { return !ctx.is_register_defined(reg); })) { return {}; }
  return out | ranges::view::transform([&](unsigned reg) { return ctx.get_register(reg); }) | ranges::to<std::vector>();
}
