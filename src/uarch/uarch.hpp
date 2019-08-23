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
#include <optional>
#include <unordered_map>

#include "instruction.hpp"

namespace uarch {

class uarch {
  unsigned gp_registers_ = 0;
  std::optional<unsigned> zero_gp_register_;

  unsigned lanes_ = 1;

  std::vector<std::unique_ptr<opcode>> all_opcodes_;
  std::unordered_map<std::string_view, opcode*> opcode_by_name_;

  friend class uarch_builder;

public:
  unsigned gp_registers() const { return gp_registers_; }
  std::optional<unsigned> zero_gp_register() const { return zero_gp_register_; }

  unsigned lanes() const { return lanes_; }

  std::vector<std::unique_ptr<opcode>> const& all_opcodes() const { return all_opcodes_; }

  opcode* get_opcode(std::string_view op) const { return opcode_by_name_.at(op); }
};

class latency {
  unsigned latency_;

public:
  explicit latency(unsigned l) : latency_(l) {}

  explicit operator unsigned() const { return latency_; }
};

namespace operand_descriptors {

namespace detail {
template<typename A, typename B> struct or_t : A, B {};
struct param_address_t {};
struct param2_address_t : param_address_t {};
} // namespace detail

inline struct imm_t {
} imm;
inline struct reg_t {
} reg;
inline struct reg2_t : reg_t {
} reg2;
inline struct param_t {
} param;
inline struct dst_reg_t : reg_t {
} dst_reg;
inline struct dst_reg2_t : reg2_t {
} dst_reg2;

template<typename T> auto param_addr(T) {
  return detail::or_t<T, detail::param_address_t>{};
}
template<typename T> auto param2_addr(T) {
  return detail::or_t<T, detail::param2_address_t>{};
}

template<typename A, typename B> auto operator|(A, B) {
  return detail::or_t<A, B>{};
}

} // namespace operand_descriptors

namespace detail {

template<size_t N, typename... Operands> struct get_operand;
template<size_t N, typename Operand, typename... Operands>
struct get_operand<N, Operand, Operands...> : get_operand<N - 1, Operands...> {};
template<typename Operand, typename... Operands> struct get_operand<0, Operand, Operands...> { using type = Operand; };

template<size_t N, typename... Operands> using get_operand_t = typename get_operand<N, Operands...>::type;

template<typename Operand> constexpr bool allows_registers = std::is_base_of_v<operand_descriptors::reg_t, Operand>;
template<typename Operand> constexpr bool allows_immediates = std::is_base_of_v<operand_descriptors::imm_t, Operand>;
template<typename Operand> constexpr bool allows_parameters = std::is_base_of_v<operand_descriptors::param_t, Operand>;
template<typename Operand> constexpr bool is_wide = std::is_base_of_v<operand_descriptors::reg2_t, Operand>;
template<typename Operand>
constexpr bool is_parameter_address = std::is_base_of_v<operand_descriptors::detail::param_address_t, Operand>;
template<typename Operand>
constexpr bool is_double_parameter_address = std::is_base_of_v<operand_descriptors::detail::param2_address_t, Operand>;

template<typename Context> class dst_wrapper {
  Context* ctx_;
  unsigned lane_;
  operand* op_;

public:
  dst_wrapper(Context& ctx, unsigned lane, operand& op) : ctx_(&ctx), lane_(lane), op_(&op) {}
  template<typename Value> void operator=(Value value) {
    op_->set(*ctx_, static_cast<typename Context::value_type>(value), lane_);
  }
  template<typename Value> void set64(Value value) {
    auto [lo, hi] = ctx_->split_u64(static_cast<typename Context::value_type>(value));
    op_->set(*ctx_, lo, lane_);
    op_->set_hi(*ctx_, hi, lane_);
  }
};

template<typename Context> class src_wrapper {
  Context* ctx_;
  unsigned lane_;
  operand* op_;

public:
  src_wrapper(Context& ctx, unsigned lane, operand& op) : ctx_(&ctx), lane_(lane), op_(&op) {}
  typename Context::value_type value() const { return op_->get(*ctx_, lane_); }
  operator typename Context::value_type() const { return value(); }

  auto lane(src_wrapper const& lane) const { return op_->get(*ctx_, lane.value() + int(lane_)); }

  auto value64() { return ctx_->make_u64(op_->get(*ctx_, lane_), op_->get_hi(*ctx_, lane_)); }

  friend auto operator+(src_wrapper const& a, src_wrapper const& b) { return a.value() + b.value(); }
  friend auto operator*(src_wrapper const& a, src_wrapper const& b) { return a.value() * b.value(); }

  friend auto params(src_wrapper const& addr) { return addr.ctx_->get_parameter(addr.value()); }
  friend auto params64(src_wrapper const& addr) {
    return addr.ctx_->make_u64(addr.ctx_->get_parameter(addr.value()), addr.ctx_->get_parameter(addr.value() + 4));
  }
};

template<typename... Operands, size_t... Idx, typename Context>
bool do_verify(std::index_sequence<Idx...>, Context& ctx, unsigned lane, std::vector<operand> const& ops) {
  auto verify_operand = [&ctx, lane](auto desc_ptr, auto& op) {
    using namespace operand_descriptors;
    using desc = std::remove_pointer_t<decltype(desc_ptr)>;
    if constexpr (!std::is_same_v<desc, dst_reg_t> && !std::is_same_v<desc, dst_reg2_t>) {
      if (!op.is_valid(ctx)) { return false; }
    }
    if constexpr (std::is_same_v<desc, reg2_t>) {
      if (!op.is_register() || !op.is_valid64(ctx)) { return false; }
    }
    if constexpr (!allows_immediates<desc>) {
      if (op.is_immediate()) { return false; }
    }
    if constexpr (!allows_parameters<desc>) {
      if (op.is_parameter()) { return false; }
    }
    if constexpr (!allows_registers<desc>) {
      if (op.is_register()) { return false; }
    }
    if constexpr (is_double_parameter_address<desc>) {
      if (!ctx.is_valid_parameter64(op.get(ctx, lane))) { return false; }
    } else if constexpr (is_parameter_address<desc>) {
      if (!ctx.is_valid_parameter(op.get(ctx, lane))) { return false; }
    }
    return true;
  };
  return (verify_operand(static_cast<Operands*>(nullptr), ops[Idx]) && ...);
}

template<size_t... Idx, typename Context, typename Function>
void do_apply(std::index_sequence<Idx...>, Function&& fn, Context& ctx, unsigned lane, std::vector<operand>& ops) {
  fn(dst_wrapper<std::decay_t<Context>>(ctx, lane, ops[0]),
     src_wrapper<std::decay_t<Context>>(ctx, lane, ops[Idx + 1])...);
};

} // namespace detail

template<typename... Operands> struct operands {
  static constexpr size_t count = sizeof...(Operands);

  // FIXME: assuming the first operand is the only destination

  operands(Operands...) {}

  template<typename Context> static bool verify(Context const& ctx, unsigned lane, std::vector<operand> const& ops) {
    using namespace detail;
    if (ops.size() != count || !ops[0].is_register()) { return false; }
    return do_verify<Operands...>(std::make_index_sequence<count>(), ctx, lane, ops);
  }

  template<typename Function, typename Context>
  static void apply(Function&& fn, Context&& ctx, unsigned lane, std::vector<operand>& ops) {
    using namespace detail;
    do_apply(std::make_index_sequence<count - 1>(), fn, ctx, lane, ops);
  }

  static std::vector<bool> make_register_mask() { return std::vector<bool>{detail::allows_registers<Operands>...}; }
  static std::vector<bool> make_immediate_mask() { return std::vector<bool>{detail::allows_immediates<Operands>...}; }
  static std::vector<bool> make_parameter_mask() { return std::vector<bool>{detail::allows_parameters<Operands>...}; }

  static std::vector<bool> make_wide_mask() { return std::vector<bool>{detail::is_wide<Operands>...}; }
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

  uarch_builder&& gp_registers(unsigned n) && {
    uarch_->gp_registers_ = n;
    return std::move(*this);
  }
  uarch_builder&& zero_gp_register(unsigned idx) && {
    uarch_->zero_gp_register_ = idx;
    return std::move(*this);
  }

  uarch_builder&& lanes(unsigned l) && {
    uarch_->lanes_ = l;
    return std::move(*this);
  }

  template<typename... Operands, typename Function>
  uarch_builder&& operator()(std::string_view name, operands<Operands...>, latency lat, Function&& fn) && {
    using ops = operands<Operands...>;

    class concrete_opcode final : public opcode, Function {
    public:
      explicit concrete_opcode(uarch& ua, std::string_view name, ::uarch::latency lat, Function fn)
          : opcode(ua, name, ops::count, ops::make_register_mask(), ops::make_immediate_mask(),
                   ops::make_parameter_mask(), ops::make_wide_mask(), unsigned(lat)),
            Function(std::move(fn)) {}

      virtual bool evaluate(evaluation_context& ctx, unsigned lane, std::vector<operand>& operands) override {
        if (!ops::verify(ctx, lane, operands)) { return false; }
        ops::apply(*static_cast<Function*>(this), ctx, lane, operands);
        return true;
      }

      virtual bool evaluate_all_lanes(evaluation_context& ctx, std::vector<operand>& operands) override {
        if (!ops::verify(ctx, 0, operands)) { return false; }
        for (auto l = 0u; l < 32; ++l) { ops::apply(*static_cast<Function*>(this), ctx, l, operands); }
        ctx.commit_pending_operations();
        return true;
      }

      virtual void emit_smt(smt_context& ctx, unsigned lane, std::vector<operand>& operands) override {
        ops::apply(*static_cast<Function*>(this), ctx, lane, operands);
      }

      virtual void update_defined_registers(std::vector<operand> const& operands,
                                            std::vector<bool>& defined_registers) const override {
        assert(!operands.empty());
        assert(operands[0].is_register());
        assert(operands[0].get_register_id() < defined_registers.size());
        defined_registers[operands[0].get_register_id()] = true;

        // FIXME: use a trait for this
        if constexpr (std::is_same_v<detail::get_operand_t<0, std::decay_t<Operands>...>,
                                     operand_descriptors::dst_reg2_t>) {
          assert(operands[0].get_register_id() + 1 < defined_registers.size());

          defined_registers[operands[0].get_register_id() + 1] = true;
        }
      }
    };

    add_opcode(std::make_unique<concrete_opcode>(*uarch_, name, lat, std::forward<Function>(fn)));
    return std::move(*this);
  }

  std::unique_ptr<uarch> build() && { return std::move(uarch_); }
};

} // namespace uarch
