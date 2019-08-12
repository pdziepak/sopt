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
#include "perf.hpp"
#include "smt.hpp"
#include "stats.hpp"

#include "optimizer/detail.hpp"
#include "optimizer/path.hpp"
#include "optimizer/target.hpp"

double cost_to_score(double cost) {
  return std::exp(-0.5 * cost);
}

double score_performance(uarch::uarch const& ua, interface const& ifce, basic_block const& bb) {
  return cost_to_score(cost_performance(ua, ifce, bb));
}

double score(uarch::uarch const& ua, interface const& ifce, std::vector<test> const& tests, basic_block& bb) {
  double cost = 0;

  for (auto [param, in, out] : tests) {
    auto actual_out = evaluate(ua, bb, param, in, ifce.output_registers);
    for (auto idx = 0u; idx < out.size(); ++idx) {
      if (!actual_out || (*actual_out)[idx] != out[idx]) { cost += 1; }
    }
  }

  cost += cost_performance(ua, ifce, bb);

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
              return std::pair(reg, ranges::view::iota(0u, ua.lanes()) | ranges::view::transform([&](unsigned lane) {
                                      return ctx.bv_const(fmt::format("r{}l{}", reg, lane).c_str(), 32);
                                    }) | ranges::to<std::vector>());
            }) |
            ranges::to<std::vector>();
  auto [expr_a, extra_a] = emit_smt(ua, ctx, a1, param, in, ifce.output_registers);
  auto [expr_b, extra_b] = emit_smt(ua, ctx, b1, param, in, ifce.output_registers);
  assert(expr_a.size() == expr_b.size());

  z3::solver slv(ctx);
  auto in_expr = z3::expr_vector(ctx);
  for (auto&& expr :
       ranges::view::concat(param | ranges::view::values, in | ranges::view::values | ranges::view::join)) {
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

  auto trgt = optimizer::target(ua, ifce, target);

  auto prng = std::default_random_engine{std::random_device{}()};

  auto start = std::chrono::steady_clock::now();
  auto total_stats = stats{};
  auto tick_stats = stats{};

  auto trace_tick = start + std::chrono::seconds(1);

  z3::solver z3slv(trgt.z3_context());

  auto pths = std::vector<optimizer::path>(16, optimizer::path(prng, z3slv, trgt));

  while (trgt.best_score() < min_score) {
    auto now = std::chrono::steady_clock::now();
    if (now > trace_tick) {
      spdlog::trace("tick, best score: {}, {}", trgt.best_score(), tick_stats);
      total_stats += tick_stats;
      tick_stats = {};
      trace_tick = now + std::chrono::seconds(1);

      ranges::sort(pths, [](optimizer::path const& a, optimizer::path const& b) {
        return a.current_score() > b.current_score();
      });
      pths.erase(pths.begin() + 12, pths.end());
      pths.resize(16, optimizer::path(prng, z3slv, trgt));
    }

    tick_stats.on_new_candidate();

    for (auto& pth : pths) { pth.next_step(tick_stats); }
  }

  total_stats += tick_stats;

  auto elapsed_time =
      std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count() / 1'000.;
  spdlog::info("finished with score {} (perf cost: {}) after {} seconds: {}\n{}", trgt.best_score(),
               cost_performance(ua, ifce, trgt.best()), elapsed_time, total_stats, trgt.best());

  return trgt.best();
}
