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

#include <random>
#include <unordered_set>

#include <fmt/format.h>
#include <fmt/ostream.h>
#include <fmt/ranges.h>

#include <spdlog/spdlog.h>

#include <range/v3/algorithm.hpp>
#include <range/v3/core.hpp>
#include <range/v3/view.hpp>

#include "evaluation.hpp"
#include "mutate.hpp"
#include "optimizer.hpp"
#include "smt.hpp"
#include "stats.hpp"

using test = std::tuple<std::map<uint64_t, value>, std::vector<std::pair<unsigned, value>>, std::vector<value>>;

bool has_unknown_immediates(basic_block const& bb) {
  for (auto& inst : bb.instructions_) {
    for (auto& op : inst.operands_) {
      if (!op.is_known()) { return true; }
    }
  }
  return false;
}

void synthesize_immediates(uarch::uarch const& ua, z3::context& z3ctx, z3::solver& z3slv, interface const& ifce,
                           std::vector<test>& tests, basic_block& candidate, std::map<uint64_t, z3::expr> const& params,
                           std::vector<std::pair<unsigned, z3::expr>> const& in, basic_block& target,
                           std::vector<z3::expr> const& target_smt, stats& st) {
  if (!has_unknown_immediates(candidate)) { return; }

  // FIXME: this is is_valid() actually
  auto ret = evaluate(ua, candidate, std::get<0>(tests.front()), std::get<1>(tests.front()), ifce.output_registers);
  if (!ret) { return; }
  if (ranges::none_of(*ret, std::mem_fn(&value::is_undefined))) { return; }

  // FIXME: emit_smt
  auto ctx = smt_context(ua, z3ctx, params);
  for (auto [reg, expr] : in) { ctx.set_register(reg, expr); }
  for (auto& inst : candidate.instructions_) { inst.opcode_->emit_smt(ctx, inst.operands_); }
  auto original_candidate_smt =
      ifce.output_registers | ranges::view::transform([&](unsigned reg) { return ctx.get_register(reg).simplify(); }) |
      ranges::to<std::vector>();
  auto original_candidate_extra_smt = ctx.extra_restrictions();

  auto original_candidate = candidate;

  auto vars = make_vector(z3ctx, ranges::view::concat(params | ranges::view::values, in | ranges::view::values));

  do {
    candidate = original_candidate;
    auto candidate_smt = original_candidate_smt;
    auto candidate_extra_smt = original_candidate_extra_smt;

    z3slv.reset();

    for (auto& [paramv, inv, outv] : tests) {
      auto vals =
          make_vector(z3ctx, ranges::view::concat(paramv | ranges::view::values, inv | ranges::view::values) |
                                 ranges::view::transform([&](value const& v) { return z3ctx.bv_val(v.as_i32(), 32); }));
      auto cnd = (make_and(z3ctx, ranges::view::zip(candidate_smt, outv) |
                                      ranges::view::transform([&](std::tuple<z3::expr, value> v) {
                                        return (std::get<0>(v) == z3ctx.bv_val(std::get<1>(v).as_i32(), 32));
                                      })) &&
                  candidate_extra_smt);
      z3slv.add(cnd.substitute(vars, vals));
    }

    st.on_smt_query();
    if (z3slv.check() != z3::check_result::sat) { return; }

    assert(candidate_smt.size() == target_smt.size());
    ctx.resolve_unknown_immediates(z3slv.get_model());

    auto ctx = smt_context(ua, z3ctx, params);
    for (auto [reg, expr] : in) { ctx.set_register(reg, expr); }
    for (auto& inst : candidate.instructions_) { inst.opcode_->emit_smt(ctx, inst.operands_); }
    candidate_smt = ifce.output_registers |
                    ranges::view::transform([&](unsigned reg) { return ctx.get_register(reg).simplify(); }) |
                    ranges::to<std::vector>();
    candidate_extra_smt = ctx.extra_restrictions();

    z3slv.reset();
    z3slv.add(make_or(z3ctx, ranges::view::zip(candidate_smt, target_smt) | ranges::view::transform([&](auto&& a_b) {
                               return std::get<0>(a_b) != std::get<1>(a_b);
                             })));
    st.on_smt_query();
    auto result = z3slv.check();
    if (result == z3::check_result::unsat) {
      return;
    } else {
      auto paramv = params | ranges::view::transform([&](std::pair<uint64_t, z3::expr> in) {
                      auto e = z3slv.get_model().eval(std::get<1>(in));
                      uint64_t v;
                      if (!e.is_numeral_u64(v)) { v = 0; }
                      return std::pair(std::get<0>(in), value(v));
                    }) |
                    ranges::to<std::map>();
      auto inv = in | ranges::view::transform([&](std::pair<unsigned, z3::expr> in) {
                   auto e = z3slv.get_model().eval(std::get<1>(in));
                   uint64_t v;
                   if (!e.is_numeral_u64(v)) { v = 0; }
                   return std::pair(std::get<0>(in), value(v));
                 }) |
                 ranges::to<std::vector>();
      auto outv = evaluate(ua, target, paramv, inv, ifce.output_registers).value();
      spdlog::trace("adding test case (constant synthesis): ({}, {}, {})", paramv, inv, outv);
      tests.emplace_back(paramv, inv, outv);
    }
  } while (true);
}

bool check_tests(uarch::uarch const& ua, interface const& ifce, std::vector<test> const& tests, basic_block& bb) {
  for (auto [param, in, out] : tests) {
    auto actual_out = evaluate(ua, bb, param, in, ifce.output_registers);
    if (!actual_out || actual_out != out) { return false; }
  }
  return true;
}

double cost_to_score(double cost) {
  return std::exp(-0.5 * cost);
}

double cost_performance(basic_block const& bb) {
  return bb.instructions_.size();
}

double score_performance(basic_block const& bb) {
  return cost_to_score(cost_performance(bb));
}

double score(uarch::uarch const& ua, interface const& ifce, std::vector<test> const& tests, basic_block& bb) {
  double cost = 0;

  for (auto [param, in, out] : tests) {
    auto actual_out = evaluate(ua, bb, param, in, ifce.output_registers);
    for (auto idx = 0u; idx < out.size(); ++idx) {
      if (!actual_out || (*actual_out)[idx] != out[idx]) { cost += 1; }
    }
  }

  cost += cost_performance(bb);

  return cost_to_score(cost);
}

bool equivalent(uarch::uarch const& ua, interface const& ifce, basic_block const& a, basic_block const& b) {
  assert(!has_unknown_immediates(a));
  assert(!has_unknown_immediates(b));

  auto a1 = a;
  auto b1 = b;

  (void)ifce;

  z3::context ctx;
  auto param = ifce.parameters | ranges::view::transform([&](uint64_t p) {
                 return std::pair(p, ctx.bv_const(fmt::format("p{}", p).c_str(), 32));
               }) |
               ranges::to<std::map>();
  auto in = ifce.input_registers | ranges::view::transform([&](unsigned reg) {
              return std::pair(reg, ctx.bv_const(fmt::format("r{}", reg).c_str(), 32));
            }) |
            ranges::to<std::vector>();
  auto [expr_a, extra_a] = emit_smt(ua, ctx, a1, param, in, ifce.output_registers);
  auto [expr_b, extra_b] = emit_smt(ua, ctx, b1, param, in, ifce.output_registers);
  assert(expr_a.size() == expr_b.size());

  z3::solver slv(ctx);
  auto in_expr = z3::expr_vector(ctx);
  for (auto&& expr : ranges::view::concat(param | ranges::view::values, in | ranges::view::values)) {
    in_expr.push_back(expr);
  }
  for (auto&& [a, b] : ranges::view::zip(expr_a, expr_b)) { slv.add(z3::forall(in_expr, a == b)); }
  slv.add(z3::forall(in_expr, extra_a));
  slv.add(z3::forall(in_expr, extra_b));
  return slv.check() == z3::check_result::sat;
}

basic_block optimize(uarch::uarch const& ua, interface const& ifce, basic_block target, double min_score) {
  spdlog::info("optimizing basic block, inputs: {}, outputs: {}\n{}", ifce.input_registers, ifce.output_registers,
               target);

  static constexpr std::array<value, 5> initial_tests = {value(uint32_t(-2)), value(uint32_t(-1)), value(0), value(1),
                                                         value(2)};
  spdlog::debug("preparing {} tests...", initial_tests.size());
  auto tests = initial_tests | ranges::view::transform([&](value v) {
                 auto params = ifce.parameters | ranges::view::transform([&](uint64_t p) { return std::pair(p, v); }) |
                               ranges::to<std::map>();
                 auto in = ifce.input_registers | ranges::view::transform([&](unsigned r) { return std::pair(r, v); }) |
                           ranges::to<std::vector>();
                 auto out = evaluate(ua, target, params, in, ifce.output_registers).value();
                 return std::tuple(params, in, out);
               }) |
               ranges::to<std::vector>();
  spdlog::trace("prepared tests: {}", fmt::join(tests, ", "));

  z3::context z3ctx;
  auto in = ifce.input_registers | ranges::view::transform([&](unsigned reg) {
              return std::pair(reg, z3ctx.bv_const(fmt::format("r{}", reg).c_str(), 32));
            }) |
            ranges::to<std::vector>();
  auto param = ifce.parameters | ranges::view::transform([&](uint64_t p) {
                 return std::pair(p, z3ctx.bv_const(fmt::format("p{}", p).c_str(), 32));
               }) |
               ranges::to<std::map>();
  auto [target_smt, target_extra_smt] = emit_smt(ua, z3ctx, target, param, in, ifce.output_registers);
  for (auto& tsmt : target_smt) { tsmt = tsmt.simplify(); }
  target_extra_smt = target_extra_smt.simplify();
  spdlog::debug("target smt formulae: {} and {}", fmt::join(ranges::view::zip(ifce.output_registers, target_smt), ", "),
                target_extra_smt);

  auto target_score = score(ua, ifce, tests, target);

  auto best_score = target_score;
  auto best = target;
  spdlog::debug("initial score: {}", target_score);

  auto prng = std::default_random_engine{std::random_device{}()};

  auto start = std::chrono::steady_clock::now();
  auto total_stats = stats{};
  auto tick_stats = stats{};

  auto trace_tick = start + std::chrono::seconds(1);

  z3::solver z3slv(z3ctx);

  while (best_score < min_score) {
    auto now = std::chrono::steady_clock::now();
    if (now > trace_tick) {
      spdlog::trace("tick, best score: {}, {}", best_score, tick_stats);
      total_stats += tick_stats;
      tick_stats = {};
      trace_tick = now + std::chrono::seconds(1);
    }

    tick_stats.on_new_candidate();

    auto candidate = target;
    mutate(ua, prng, ifce, candidate);

    synthesize_immediates(ua, z3ctx, z3slv, ifce, tests, candidate, param, in, best, target_smt, tick_stats);

    auto candidate_score = score(ua, ifce, tests, candidate);
    auto alpha = std::min(1., candidate_score / target_score);
    if (std::uniform_real_distribution<double>{}(prng) > alpha) { continue; }

    bool is_best_score = candidate_score > best_score;

    target = candidate;
    target_score = candidate_score;

    if (!is_best_score) { continue; }

    if (check_tests(ua, ifce, tests, target)) {
      auto [candidate_smt, candidate_extra_smt] = emit_smt(ua, z3ctx, candidate, param, in, ifce.output_registers);

      tick_stats.on_smt_query();

      z3slv.reset();
      z3slv.add(make_or(z3ctx, ranges::view::zip(candidate_smt, target_smt) | ranges::view::transform([](auto c_t) {
                                 return std::get<0>(c_t) != std::get<1>(c_t);
                               })) ||
                !candidate_extra_smt);
      auto result = z3slv.check();
      if (result == z3::check_result::unsat) {
        spdlog::debug("found better basic block with score {}:\n{}", candidate_score, candidate);
        best_score = candidate_score;
        best = candidate;
      } else if (result == z3::check_result::sat) {
        auto paramv = param | ranges::view::transform([&](std::pair<uint64_t, z3::expr> in) {
                        auto e = z3slv.get_model().eval(std::get<1>(in));
                        uint64_t v;
                        if (!e.is_numeral_u64(v)) { v = 0; }
                        return std::pair(std::get<0>(in), value(v));
                      }) |
                      ranges::to<std::map>();
        auto inv = in | ranges::view::transform([&](std::pair<unsigned, z3::expr> in) {
                     auto e = z3slv.get_model().eval(std::get<1>(in));
                     uint64_t v;
                     if (!e.is_numeral_u64(v)) { v = 0; }
                     return std::pair(std::get<0>(in), value(v));
                   }) |
                   ranges::to<std::vector>();
        auto outv = evaluate(ua, best, paramv, inv, ifce.output_registers).value();
        spdlog::trace("adding test case: ({}, {}, {})", paramv, inv, outv);
        tests.emplace_back(paramv, inv, outv);

        target_score = score(ua, ifce, tests, target);
      }
    }
  }

  total_stats += tick_stats;

  auto elapsed_time =
      std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count() / 1'000.;
  spdlog::info("finished with score {} after {} seconds: {}\n{}", best_score, elapsed_time, total_stats, best);

  return best;
}
