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

#include "evaluation.hpp"
#include "smt.hpp"

class operand {
  uint64_t value_;
  bool constant_;
  bool known_ = true;

public:
  uint64_t get(evaluation_context const& ctx) const { return constant_ ? value_ : ctx.get_register(value_); }
  void set(evaluation_context& ctx, uint64_t v) const {
    assert(!constant_);
    ctx.set_register(value_, v);
  }

  z3::expr get(smt_context& ctx) {
    return constant_ ? (known_ ? ctx.get_constant(value_) : ctx.add_unknown_immediate([this](uint64_t v) {
      value_ = v;
      known_ = true;
    }))
                     : ctx.get_register(value_);
  }
  void set(smt_context& ctx, z3::expr v) const {
    assert(!constant_);
    ctx.set_register(value_, v);
  }

  bool is_register() const { return !constant_; }
  bool is_immediate() const { return constant_; }
  bool is_known() const { return is_register() || known_; }
  bool is_valid(evaluation_context const& ctx) const { return constant_ ? true : ctx.is_register_defined(value_); }

  uint64_t get_register_id() const {
    assert(is_register());
    return value_;
  }

  void set_immediate_value(uint64_t v) {
    assert(is_immediate());
    value_ = v;
  }

  static operand make_register(unsigned r) {
    auto op = operand{};
    op.value_ = r;
    op.constant_ = false;
    return op;
  }

  static operand make_unknown_immediate() {
    auto op = operand{};
    op.constant_ = true;
    op.known_ = false;
    return op;
  }

  static operand make_immediate(uint64_t v) {
    auto op = operand{};
    op.value_ = v;
    op.constant_ = true;
    op.known_ = true;
    return op;
  }

  friend std::ostream& operator<<(std::ostream& os, operand const& op) {
    if (op.constant_) {
      if (op.known_) {
        return os << op.value_;
      } else {
        return os << "<imm>";
      }
    } else {
      return os << 'r' << op.value_;
    }
  }
};

class opcode {
  std::string name_;
  unsigned operand_count_ = 0;
  std::vector<bool> immediates_;

public:
  explicit opcode(std::string_view name, unsigned opcount, std::vector<bool> immediates)
      : name_(name), operand_count_(opcount), immediates_(std::move(immediates)) {}
  virtual ~opcode() = default;

  std::string_view name() const { return name_; }
  unsigned operand_count() const { return operand_count_; }

  bool can_be_immediate(unsigned op) const {
    assert(op < operand_count());
    return immediates_[op];
  }

  virtual bool evaluate(evaluation_context& ctx, std::vector<operand>& operands) = 0;
  virtual void emit_smt(smt_context& ctx, std::vector<operand>& operands) = 0;

  friend std::ostream& operator<<(std::ostream& os, opcode const& op) { return os << op.name_; }
};

struct instruction {
  opcode* opcode_;
  std::vector<operand> operands_;

  friend std::ostream& operator<<(std::ostream& os, instruction const& inst) {
    fmt::print(os, "{} {}", *inst.opcode_, fmt::join(inst.operands_, ", "));
    return os;
  }
};

struct basic_block {
  std::vector<instruction> instructions_;

  friend std::ostream& operator<<(std::ostream& os, basic_block const& bb) {
    fmt::print(os, "  {}", fmt::join(bb.instructions_, "\n  "));
    return os;
  }
};
