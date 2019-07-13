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

#include <list>
#include <memory>
#include <unordered_map>

#include "instruction.hpp"

namespace uarch {

class uarch {
  std::vector<std::unique_ptr<opcode>> all_opcodes_;
  std::unordered_map<std::string_view, opcode*> opcode_by_name_;

  friend class uarch_builder;

public:
  std::vector<std::unique_ptr<opcode>> const& all_opcodes() const { return all_opcodes_; }

  opcode* get_opcode(std::string_view op) const { return opcode_by_name_.at(op); }
};

namespace operand_descriptors {

namespace detail {
template<typename A, typename B> struct or_t : A, B {};
struct param_address_t {};
} // namespace detail

inline struct imm_t {
} imm;
inline struct reg_t {
} reg;
inline struct param_t {
} param;
inline struct dst_reg_t : reg_t {
} dst_reg;

template<typename T> auto param_addr(T) {
  return detail::or_t<T, detail::param_address_t>{};
}

template<typename A, typename B> auto operator|(A, B) {
  return detail::or_t<A, B>{};
}

} // namespace operand_descriptors

namespace detail {

template<typename Operand> constexpr bool allows_immediates = std::is_base_of_v<operand_descriptors::imm_t, Operand>;
template<typename Operand> constexpr bool allows_parameters = std::is_base_of_v<operand_descriptors::param_t, Operand>;
template<typename Operand>
constexpr bool is_parameter_address = std::is_base_of_v<operand_descriptors::detail::param_address_t, Operand>;

template<typename Context> class dst_wrapper {
  Context* ctx_;
  operand* op_;

public:
  dst_wrapper(Context& ctx, operand& op) : ctx_(&ctx), op_(&op) {}
  template<typename Value> void operator=(Value value) {
    op_->set(*ctx_, static_cast<typename Context::value_type>(value));
  }
};

template<typename Context> class src_wrapper {
  Context* ctx_;
  operand* op_;

public:
  src_wrapper(Context& ctx, operand& op) : ctx_(&ctx), op_(&op) {}
  typename Context::value_type value() const { return op_->get(*ctx_); }
  operator typename Context::value_type() const { return value(); }

  friend auto operator+(src_wrapper const& a, src_wrapper const& b) { return a.value() + b.value(); }
  friend auto operator*(src_wrapper const& a, src_wrapper const& b) { return a.value() * b.value(); }

  friend auto params(src_wrapper const& addr) { return addr.ctx_->get_parameter(addr.value()); }
};

template<typename... Operands, size_t... Idx, typename Context>
bool do_verify(std::index_sequence<Idx...>, Context& ctx, std::vector<operand> const& ops) {
  auto verify_operand = [&ctx](auto desc_ptr, auto& op) {
    using namespace operand_descriptors;
    using desc = std::remove_pointer_t<decltype(desc_ptr)>;
    if constexpr (!std::is_same_v<desc, dst_reg_t>) {
      if (!op.is_valid(ctx)) { return false; }
    }
    if constexpr (!allows_immediates<desc>) {
      if (op.is_immediate()) { return false; }
    }
    if constexpr (!allows_parameters<desc>) {
      if (op.is_parameter()) { return false; }
    }
    if constexpr (is_parameter_address<desc>) {
      if (!ctx.is_valid_parameter(op.get(ctx))) { return false; }
    }
    return true;
  };
  return (verify_operand(static_cast<Operands*>(nullptr), ops[Idx]) && ...);
}

template<size_t... Idx, typename Context, typename Function>
void do_apply(std::index_sequence<Idx...>, Function&& fn, Context& ctx, std::vector<operand>& ops) {
  fn(dst_wrapper<std::decay_t<Context>>(ctx, ops[0]), src_wrapper<std::decay_t<Context>>(ctx, ops[Idx + 1])...);
};

} // namespace detail

template<typename... Operands> struct operands {
  static constexpr size_t count = sizeof...(Operands);

  // FIXME: assuming the first operand is the only destination

  operands(Operands...) {}

  template<typename Context> static bool verify(Context const& ctx, std::vector<operand> const& ops) {
    using namespace detail;
    if (ops.size() != count || !ops[0].is_register()) { return false; }
    return do_verify<Operands...>(std::make_index_sequence<count>(), ctx, ops);
  }

  template<typename Function, typename Context>
  static void apply(Function&& fn, Context&& ctx, std::vector<operand>& ops) {
    using namespace detail;
    do_apply(std::make_index_sequence<count - 1>(), fn, ctx, ops);
  }

  static std::vector<bool> make_immediate_mask() { return std::vector<bool>{detail::allows_immediates<Operands>...}; }
  static std::vector<bool> make_parameter_mask() { return std::vector<bool>{detail::allows_parameters<Operands>...}; }
};

class uarch_builder {
  std::unique_ptr<uarch> uarch_;

public:
  void add_opcode(std::unique_ptr<opcode> op_ptr) {
    auto& op = *op_ptr;
    uarch_->all_opcodes_.emplace_back(std::move(op_ptr));
    auto [it, inserted] = uarch_->opcode_by_name_.emplace(op.name(), &op);
    if (!inserted) { throw std::runtime_error(fmt::format("opcode \"{}\" already exists", op.name())); }
  }

public:
  uarch_builder() : uarch_(std::make_unique<uarch>()) {}

  template<typename... Operands, typename Function>
  uarch_builder&& operator()(std::string_view name, operands<Operands...>, Function&& fn) && {
    using ops = operands<Operands...>;

    class concrete_opcode final : public opcode, Function {
    public:
      explicit concrete_opcode(std::string_view name, Function fn)
          : opcode(name, ops::count, ops::make_immediate_mask(), ops::make_parameter_mask()), Function(std::move(fn)) {}

      virtual bool evaluate(evaluation_context& ctx, std::vector<operand>& operands) override {
        if (!ops::verify(ctx, operands)) { return false; }
        ops::apply(*static_cast<Function*>(this), ctx, operands);
        return true;
      }

      virtual void emit_smt(smt_context& ctx, std::vector<operand>& operands) override {
        ops::apply(*static_cast<Function*>(this), ctx, operands);
      }
    };

    add_opcode(std::make_unique<concrete_opcode>(name, std::forward<Function>(fn)));
    return std::move(*this);
  }

  std::unique_ptr<uarch> build() && { return std::move(uarch_); }
};

} // namespace uarch
