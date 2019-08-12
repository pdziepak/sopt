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

#include "perf.hpp"

#include <range/v3/algorithm/max.hpp>
#include <range/v3/view/transform.hpp>

// FIXME: assuming in-order, single-issue uarch
// FIXME: assuming all instructions have fixed latency, independent from data
// FIXME: ignoring throughput

double cost_performance(uarch::uarch const& ua, interface const& ifce, basic_block const& bb) {
  // FIXME: assuming that between issuing an instruction and its retiring the destination is in undefined state, in
  // practice it may be perfectly fine to read the destination register which would still contain the previous value
  auto current_cycle = 0u;
  auto registers_ready_at = std::vector<unsigned>(ua.gp_registers());
  for (auto& inst : bb.instructions_) {
    // FIXME: assuming first operand is destination
    for (auto op_idx = 1u; op_idx < inst.operands_.size(); ++op_idx) {
      auto& op = inst.operands_[op_idx];
      if (!op.is_register()) { continue; }
      current_cycle = std::max(current_cycle, registers_ready_at[op.get_register_id()]);
      if (inst.opcode_->is_wide_operand(op_idx)) {
        current_cycle = std::max(current_cycle, registers_ready_at[op.get_register_id() + 1]);
      }
    }
    registers_ready_at[inst.operands_.front().get_register_id()] = current_cycle + inst.opcode_->latency();
    if (inst.opcode_->is_wide_operand(0)) {
      registers_ready_at[inst.operands_.front().get_register_id() + 1] = current_cycle + inst.opcode_->latency();
    }
    ++current_cycle;
  }
  auto outs_ready =
      ranges::max(ifce.output_registers | ranges::view::transform([&](unsigned r) { return registers_ready_at[r]; }));
  return std::max(current_cycle, outs_ready) + bb.instructions_.size() * 0.1;
}
