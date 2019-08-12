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

#include <fmt/format.h>
#include <fmt/ostream.h>

#include "evaluation.hpp"
#include "smt.hpp"

class operand {
  enum class type {
    reg,
    imm,
    param,
  };

private:
  uint64_t value_ = 0;
  type type_ = type::reg;
  bool known_ = true;

public:
  template<typename LaneType> value get(evaluation_context const& ctx, LaneType lane) const {
    assert(is_valid(ctx));
    switch (type_) {
    case type::reg: return ctx.get_register(value_, lane);
    case type::imm: return known_ ? value(value_) : value();
    case type::param: return ctx.get_parameter(value(value_));
    }
    abort();
  }
  value get_hi(evaluation_context const& ctx, unsigned lane) const {
    assert(is_valid(ctx));
    switch (type_) {
    case type::reg: return ctx.get_register(value_ + 1, lane);
    case type::imm: abort();
    case type::param: return ctx.get_parameter(value(value_ + 4));
    }
    abort();
  }
  void set(evaluation_context& ctx, value v, unsigned lane) const {
    assert(type_ == type::reg);
    ctx.set_register(value_, v, lane);
  }
  void set_hi(evaluation_context& ctx, value v, unsigned lane) const {
    assert(type_ == type::reg);
    ctx.set_register(value_ + 1, v, lane);
  }

  template<typename LaneType> z3::expr get(smt_context& ctx, LaneType lane) {
    switch (type_) {
    case type::reg: return ctx.get_register(value_, lane);
    case type::imm:
      if (known_) {
        return ctx.get_constant(value_);
      } else {
        return ctx.add_unknown_immediate([this](uint64_t v) {
          value_ = v;
          known_ = true;
        });
      }
    case type::param: return ctx.get_parameter(value_);
    }
    abort();
  }
  z3::expr get_hi(smt_context& ctx, unsigned lane) {
    switch (type_) {
    case type::reg: return ctx.get_register(value_ + 1, lane);
    case type::imm: abort();
    case type::param: return ctx.get_parameter(value_ + 4);
    }
    abort();
  }
  void set(smt_context& ctx, z3::expr v, unsigned lane) const {
    assert(type_ == type::reg);
    ctx.set_register(value_, v, lane);
  }
  void set_hi(smt_context& ctx, z3::expr v, unsigned lane) const {
    assert(type_ == type::reg);
    ctx.set_register(value_ + 1, v, lane);
  }

  bool is_register() const { return type_ == type::reg; }
  bool is_immediate() const { return type_ == type::imm; }
  bool is_parameter() const { return type_ == type::param; }
  bool is_known() const { return !is_immediate() || known_; }
  bool is_valid(evaluation_context const& ctx) const { return !is_register() ? true : ctx.is_register_defined(value_); }
  bool is_valid64(evaluation_context const& ctx) const {
    assert(is_register());
    return ctx.is_register_defined(value_) && ctx.is_register_defined(value_ + 1);
  }

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
    op.type_ = type::reg;
    return op;
  }

  static operand make_unknown_immediate() {
    auto op = operand{};
    op.type_ = type::imm;
    op.known_ = false;
    return op;
  }

  static operand make_immediate(uint64_t v) {
    auto op = operand{};
    op.value_ = v;
    op.type_ = type::imm;
    op.known_ = true;
    return op;
  }

  static operand make_parameter(uint64_t v) {
    auto op = operand{};
    op.value_ = v;
    op.type_ = type::param;
    return op;
  }

  friend std::ostream& operator<<(std::ostream& os, operand const& op) {
    switch (op.type_) {
    case type::reg: fmt::print(os, "r{}", op.value_); break;
    case type::imm:
      if (op.known_) {
        fmt::print(os, "{:#x}", op.value_);
      } else {
        fmt::print(os, "<imm>");
      }
      break;
    case type::param: fmt::print(os, "p[{:#x}]", op.value_); break;
    }
    return os;
  }
};

class opcode {
  std::string name_;
  unsigned operand_count_ = 0;
  std::vector<bool> registers_;
  std::vector<bool> immediates_;
  std::vector<bool> parameters_;
  std::vector<bool> wide_;

  unsigned latency_;

public:
  explicit opcode(std::string_view name, unsigned opcount, std::vector<bool> registers, std::vector<bool> immediates,
                  std::vector<bool> parameters, std::vector<bool> wide, unsigned lat)
      : name_(name), operand_count_(opcount), registers_(std::move(registers)), immediates_(std::move(immediates)),
        parameters_(std::move(parameters)), wide_(std::move(wide)), latency_(lat) {}
  virtual ~opcode() = default;

  std::string_view name() const { return name_; }
  unsigned operand_count() const { return operand_count_; }

  bool can_be_register(unsigned op) const {
    assert(op < operand_count());
    return registers_[op];
  }
  bool can_be_immediate(unsigned op) const {
    assert(op < operand_count());
    return immediates_[op];
  }
  bool can_be_parameter(unsigned op) const {
    assert(op < operand_count());
    return parameters_[op];
  }

  bool is_wide_operand(unsigned op) const {
    assert(op < operand_count());
    return wide_[op];
  }

  unsigned latency() const { return latency_; }

  virtual bool evaluate(evaluation_context& ctx, unsigned lane, std::vector<operand>& operands) = 0;
  virtual void emit_smt(smt_context& ctx, unsigned lane, std::vector<operand>& operands) = 0;

  virtual void update_defined_registers(std::vector<operand> const& operands,
                                        std::vector<bool>& defined_registers) const = 0;

  friend std::ostream& operator<<(std::ostream& os, opcode const& op) { return os << op.name_; }
};

struct instruction {
  opcode* opcode_;
  std::vector<operand> operands_;

  void update_defined_registers(std::vector<bool>& registers) const {
    opcode_->update_defined_registers(operands_, registers);
  }

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
