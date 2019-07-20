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

#include "mutate.hpp"
#include "optimizer.hpp"

#include <range/v3/algorithm/count.hpp>
#include <range/v3/algorithm/find.hpp>

class mutator {
  std::default_random_engine& prng_;
  uarch::uarch const& ua_;
  interface const& ifce_;
  basic_block& bb_;

private:
  std::vector<bool> get_defined_registers_at(instruction const& inst) {
    assert(&inst >= bb_.instructions_.data());
    assert(&inst < bb_.instructions_.data() + bb_.instructions_.size());
    auto defined_registers = std::vector<bool>(ua_.gp_registers());
    for (auto idx : ifce_.input_registers) { defined_registers[idx] = true; }
    if (ua_.zero_gp_register()) { defined_registers[*ua_.zero_gp_register()] = true; }
    auto current = bb_.instructions_.begin();
    while (&*current != &inst) {
      current->update_defined_registers(defined_registers);
      ++current;
    }

    return defined_registers;
  }

  operand random_source_register(std::vector<bool> const& defined_registers) {
    auto defined_count = ranges::count(defined_registers, true);
    assert(defined_count);
    auto idx = std::uniform_int_distribution<ptrdiff_t>{0, defined_count - 1}(prng_);
    for (auto i = 0u; i < defined_registers.size(); ++i) {
      if (!defined_registers[i]) { continue; }
      if (!idx) { return operand::make_register(i); }
      --idx;
    }
    std::abort();
  }

  operand random_destination_register(std::vector<bool> const& defined_registers) {
    auto defined_count = ranges::count(defined_registers, true);
    auto can_use_new_register = defined_count < ua_.gp_registers();
    defined_count -= bool(ua_.zero_gp_register());

    assert(defined_count || can_use_new_register);

    // FIXME: What would be the right balance between choosing a new register or an old one?
    if (!defined_count || (can_use_new_register && std::uniform_int_distribution<unsigned>{0, 1}(prng_))) {
      auto idx = std::distance(defined_registers.begin(), ranges::find(defined_registers, false));
      return operand::make_register(idx);
    }

    auto idx = std::uniform_int_distribution<ptrdiff_t>{0, defined_count - 1}(prng_);
    for (auto i = 0u; i < defined_registers.size(); ++i) {
      if (!defined_registers[i]) { continue; }
      if (ua_.zero_gp_register() && i == *ua_.zero_gp_register()) { continue; }
      if (!idx) { return operand::make_register(i); }
      --idx;
    }
    std::abort();
  }

  operand generate_operand(instruction const& inst, unsigned oper_idx) {
    auto defined_register = get_defined_registers_at(inst);
    if (oper_idx == 0) { // FIXME: don't assume first operand is the register destination
      for (auto i : ifce_.output_registers) {
        defined_register[i] = true;
      }
      return random_destination_register(defined_register);
    }
    bool allow_immediate = inst.opcode_->can_be_immediate(oper_idx);
    bool allow_parameter = inst.opcode_->can_be_parameter(oper_idx) && ifce_.parameters.size();
    if (!allow_immediate || std::uniform_int_distribution<unsigned>{0, 100}(prng_)) {
      if (!allow_parameter || std::uniform_int_distribution<unsigned>{0, 1}(prng_)) {
        return random_source_register(defined_register);
      } else {
        return operand::make_parameter(
            ifce_.parameters[std::uniform_int_distribution<size_t>{0, ifce_.parameters.size() - 1}(prng_)]);
      }
    } else {
      return operand::make_unknown_immediate();
    }
  }

  opcode* random_opcode() {
    auto& opcodes = ua_.all_opcodes();
    assert(!opcodes.empty());
    return opcodes[std::uniform_int_distribution<size_t>{0, opcodes.size() - 1}(prng_)].get();
  }

public:
  mutator(std::default_random_engine& prng, uarch::uarch const& ua, interface const& ifce, basic_block& bb)
      : prng_(prng), ua_(ua), ifce_(ifce), bb_(bb) {}

  void operator()() {
    do {
      switch (std::uniform_int_distribution<unsigned>{0, 4}(prng_)) {
      case 0: {
        if (bb_.instructions_.empty()) { continue; }
        auto& inst = bb_.instructions_[std::uniform_int_distribution<size_t>{0, bb_.instructions_.size() - 1}(prng_)];
        inst.opcode_ = random_opcode();
        inst.operands_.resize(inst.opcode_->operand_count());
        break;
      }
      case 1: {
        if (bb_.instructions_.empty()) { continue; }
        auto& inst = bb_.instructions_[std::uniform_int_distribution<size_t>{0, bb_.instructions_.size() - 1}(prng_)];
        if (inst.operands_.empty()) { continue; }
        auto oper_idx = std::uniform_int_distribution<size_t>{0, inst.operands_.size() - 1}(prng_);
        inst.operands_[oper_idx] = generate_operand(inst, oper_idx);
        break;
      }
      case 2: {
        auto idx = std::uniform_int_distribution<size_t>{0, bb_.instructions_.size()}(prng_);
        auto it = bb_.instructions_.insert(bb_.instructions_.begin() + idx, instruction{});
        auto& inst = *it;
        inst.opcode_ = random_opcode();
        for (auto i = 0u; i < inst.opcode_->operand_count(); ++i) {
          inst.operands_.emplace_back(generate_operand(inst, i));
        }
        break;
      }
      case 3: {
        if (bb_.instructions_.size() < 2) { continue; }
        auto dist = std::uniform_int_distribution<size_t>{0, bb_.instructions_.size() - 1};
        std::swap(bb_.instructions_[dist(prng_)], bb_.instructions_[dist(prng_)]);
        break;
      }
      case 4: {
        if (bb_.instructions_.empty()) { continue; }
        auto idx = std::uniform_int_distribution<size_t>{0, bb_.instructions_.size() - 1}(prng_);
        bb_.instructions_.erase(bb_.instructions_.begin() + idx);
        break;
      }
      }
      break;
    } while (true);
  }
};

void mutate(uarch::uarch const& ua, std::default_random_engine& prng, interface const& ifce, basic_block& bb) {
  mutator{prng, ua, ifce, bb}();
}
