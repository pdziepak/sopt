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

#include <range/v3/to_container.hpp>
#include <range/v3/view/iota.hpp>
#include <range/v3/view/transform.hpp>
#include <range/v3/view/zip.hpp>

#include <fmt/format.h>
#include <fmt/ostream.h>
#include <fmt/ranges.h>

#include <spdlog/spdlog.h>

#include "target.hpp"

#include "../perf.hpp"

namespace optimizer {

static constexpr std::array<value, 5> initial_tests = {value(uint32_t(-2)), value(uint32_t(-1)), value(0), value(1),
                                                       value(2)};

target::target(uarch::uarch const& ua, interface const& ifce, basic_block trgt)
    : ua_(ua), ifce_(ifce),
      tests_(initial_tests | ranges::view::transform([&](value v) {
               auto params = ifce.parameters | ranges::view::transform([&](uint64_t p) { return std::pair(p, v); }) |
                             ranges::to<std::map>();
               auto in = ifce.input_registers | ranges::view::transform([&](unsigned r) {
                           return std::pair(r, std::vector<value>(ua.lanes(), v));
                         }) |
                         ranges::to<std::vector>();
               auto out = evaluate(ua, trgt, params, in, ifce.output_registers).value();
               return std::tuple(params, in, out);
             }) |
             ranges::to<std::vector>()),
      target_(trgt),
      in_exprs_(ifce.input_registers | ranges::view::transform([&](unsigned reg) {
                  return std::pair(reg,
                                   ranges::view::iota(0u, ua.lanes()) | ranges::view::transform([&](unsigned lane) {
                                     return z3ctx_.bv_const(fmt::format("r{}l{}", reg, lane).c_str(), 32);
                                   }) | ranges::to<std::vector>());
                }) |
                ranges::to<std::vector>()),
      param_exprs_(ifce.parameters | ranges::view::transform([&](uint64_t p) {
                     return std::pair(p, z3ctx_.bv_const(fmt::format("p{}", p).c_str(), 32));
                   }) |
                   ranges::to<std::map>()),
      target_extra_smt_(z3ctx_), best_(trgt), best_score_(score(ua, ifce, tests_, target_)) {
  spdlog::trace("prepared tests: {}", fmt::join(tests_, ", "));
  std::tie(target_smt_, target_extra_smt_) =
      emit_smt(ua, z3ctx_, target_, param_exprs_, in_exprs_, ifce.output_registers);
  for (auto& tsmt : target_smt_) { tsmt = tsmt.simplify(); }
  target_extra_smt_ = target_extra_smt_.simplify();
  spdlog::debug("target smt formulae: {} and {}",
                fmt::join(ranges::view::zip(ifce.output_registers, target_smt_), ", "), target_extra_smt_);

  spdlog::debug("initial score: {} (perf cost: {})", best_score_, cost_performance(ua, ifce, target_));
}

} // namespace optimizer
