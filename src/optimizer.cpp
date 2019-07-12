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

#include "optimizer.hpp"

std::optional<std::vector<uint64_t>> evaluate(basic_block& bb, std::vector<std::pair<unsigned, uint64_t>> in,
                                              std::vector<unsigned> const& out) {
  auto ctx = evaluation_context();
  for (auto [reg, val] : in) { ctx.set_register(reg, val); }
  for (auto& inst : bb.instructions_) {
    if (!inst.opcode_->evaluate(ctx, inst.operands_)) { return {}; }
  }
  if (ranges::any_of(out, [&](unsigned reg) { return !ctx.is_register_defined(reg); })) { return {}; }
  return out | ranges::view::transform([&](unsigned reg) { return ctx.get_register(reg); }) | ranges::to<std::vector>();
}

std::vector<z3::expr> emit_smt(z3::context& z3ctx, basic_block& bb,
                               std::vector<std::pair<unsigned, z3::expr>> const& in, std::vector<unsigned> const& out) {
  // FIXME: const correctness

  auto ctx = smt_context(z3ctx);
  for (auto [reg, expr] : in) { ctx.set_register(reg, expr); }
  for (auto& inst : bb.instructions_) { inst.opcode_->emit_smt(ctx, inst.operands_); }
  return out | ranges::view::transform([&](unsigned reg) { return ctx.get_register(reg).simplify(); }) |
         ranges::to<std::vector>();
}

opcode* random_opcode(uarch::uarch const& ua, std::default_random_engine& prng) {
  auto& opcodes = ua.all_opcodes();
  assert(!opcodes.empty());
  return opcodes[std::uniform_int_distribution<size_t>{0, opcodes.size() - 1}(prng)].get();
}

void mutate(uarch::uarch const& ua, std::default_random_engine& prng, basic_block& bb) {
  do {
    switch (std::uniform_int_distribution<unsigned>{0, 4}(prng)) {
    case 0: {
      if (bb.instructions_.empty()) { continue; }
      auto& inst = bb.instructions_[std::uniform_int_distribution<size_t>{0, bb.instructions_.size() - 1}(prng)];
      inst.opcode_ = random_opcode(ua, prng);
      inst.operands_.resize(inst.opcode_->operand_count());
      break;
    }
    case 1: {
      if (bb.instructions_.empty()) { continue; }
      auto& inst = bb.instructions_[std::uniform_int_distribution<size_t>{0, bb.instructions_.size() - 1}(prng)];
      if (inst.operands_.empty()) { continue; }
      auto oper_idx = std::uniform_int_distribution<size_t>{0, inst.operands_.size() - 1}(prng);
      auto& oper = inst.operands_[oper_idx];
      bool allow_immediate = inst.opcode_->can_be_immediate(oper_idx);
      if (!allow_immediate || std::uniform_int_distribution<unsigned>{0, 100}(prng)) {
        oper = operand::make_register(std::uniform_int_distribution<unsigned>{0, 3}(prng));
      } else {
        oper = operand::make_unknown_immediate();
      }
      break;
    }
    case 2: {
      auto idx = std::uniform_int_distribution<size_t>{0, bb.instructions_.size()}(prng);
      auto inst = instruction{};
      inst.opcode_ = random_opcode(ua, prng);
      for (auto i = 0u; i < inst.opcode_->operand_count(); ++i) {
        bool allow_immediate = inst.opcode_->can_be_immediate(i);
        if (!allow_immediate || std::uniform_int_distribution<unsigned>{0, 100}(prng)) {
          inst.operands_.emplace_back(operand::make_register(std::uniform_int_distribution<unsigned>{0, 3}(prng)));
        } else {
          inst.operands_.emplace_back(operand::make_unknown_immediate());
        }
      }
      bb.instructions_.insert(bb.instructions_.begin() + idx, inst);
      break;
    }
    case 3: {
      if (bb.instructions_.size() < 2) { continue; }
      auto dist = std::uniform_int_distribution<size_t>{0, bb.instructions_.size() - 1};
      std::swap(bb.instructions_[dist(prng)], bb.instructions_[dist(prng)]);
      break;
    }
    case 4: {
      if (bb.instructions_.empty()) { continue; }
      auto idx = std::uniform_int_distribution<size_t>{0, bb.instructions_.size() - 1}(prng);
      bb.instructions_.erase(bb.instructions_.begin() + idx);
      break;
    }
    }
    break;
  } while (true);
}

bool has_unknown_immediates(basic_block const& bb) {
  for (auto& inst : bb.instructions_) {
    for (auto& op : inst.operands_) {
      if (!op.is_known()) { return true; }
    }
  }
  return false;
}

void synthesize_immediates(
    z3::context& z3ctx, interface const& ifce,
    std::vector<std::tuple<std::vector<std::pair<unsigned, uint64_t>>, std::vector<uint64_t>>> const& tests,
    basic_block& candidate, std::vector<std::pair<unsigned, z3::expr>> const& in,
    std::vector<z3::expr> const& target_smt, uint64_t& queries) {
  if (!has_unknown_immediates(candidate)) { return; }

  // FIXME: this is is_valid() actually
  auto ret = evaluate(candidate, std::get<0>(tests.front()), ifce.output_registers);
  if (!ret) { return; }

  // FIXME: emit_smt
  auto ctx = smt_context(z3ctx);
  for (auto [reg, expr] : in) { ctx.set_register(reg, expr); }
  for (auto& inst : candidate.instructions_) { inst.opcode_->emit_smt(ctx, inst.operands_); }
  auto candidate_smt = ifce.output_registers |
                       ranges::view::transform([&](unsigned reg) { return ctx.get_register(reg).simplify(); }) |
                       ranges::to<std::vector>();

  z3::solver z3slv(z3ctx);
  for (auto [inv, outv] : tests) {
    z3slv.add(z3::implies(make_and(z3ctx, ranges::view::zip(in | ranges::view::values, inv | ranges::view::values) |
                                              ranges::view::transform([&](std::tuple<z3::expr, uint64_t> v) {
                                                return std::get<0>(v) == z3ctx.bv_val(std::get<1>(v), 32);
                                              })),
                          make_and(z3ctx, ranges::view::zip(candidate_smt, outv) |
                                              ranges::view::transform([&](std::tuple<z3::expr, uint64_t> v) {
                                                return std::get<0>(v) == z3ctx.bv_val(std::get<1>(v), 32);
                                              }))));
  }
  ++queries;
  if (z3slv.check() != z3::check_result::sat) { return; }

  assert(candidate_smt.size() == target_smt.size());

  auto in_expr = z3::expr_vector(z3ctx);
  for (auto&& expr : in | ranges::view::values) { in_expr.push_back(expr); }
  for (auto&& [a, b] : ranges::view::zip(candidate_smt, target_smt)) { z3slv.add(z3::forall(in_expr, a == b)); }
  ++queries;
  if (z3slv.check() == z3::check_result::sat) { ctx.resolve_unknown_immediates(z3slv.get_model()); }
}

bool check_tests(
    interface const& ifce,
    std::vector<std::tuple<std::vector<std::pair<unsigned, uint64_t>>, std::vector<uint64_t>>> const& tests,
    basic_block& bb) {
  for (auto [in, out] : tests) {
    auto actual_out = evaluate(bb, in, ifce.output_registers);
    if (!actual_out || actual_out != out) { return false; }
  }
  return true;
}

double cost_to_score(double cost) {
  return std::exp(-0.5 * cost);
}

double cost_performance(basic_block const& bb) {
  std::unordered_set<uint64_t> registers;
  for (auto& inst : bb.instructions_) {
    for (auto& op : inst.operands_) {
      if (op.is_register()) { registers.emplace(op.get_register_id()); }
    }
  }
  return bb.instructions_.size() + registers.size();
}

double score_performance(basic_block const& bb) {
  return cost_to_score(cost_performance(bb));
}

double score(interface const& ifce,
             std::vector<std::tuple<std::vector<std::pair<unsigned, uint64_t>>, std::vector<uint64_t>>> const& tests,
             basic_block& bb) {
  double cost = 0;

  for (auto [in, out] : tests) {
    auto actual_out = evaluate(bb, in, ifce.output_registers);
    if (!actual_out) {
      cost += 4;
    } else if (actual_out != out) {
      cost += 1;
    }
  }

  cost += cost_performance(bb);

  return cost_to_score(cost);
}

bool equivalent(interface const& ifce, basic_block const& a, basic_block const& b) {
  assert(!has_unknown_immediates(a));
  assert(!has_unknown_immediates(b));

  auto a1 = a;
  auto b1 = b;

  (void)ifce;

  z3::context ctx;
  auto in = ifce.input_registers | ranges::view::transform([&](unsigned reg) {
              return std::pair(reg, ctx.bv_const(fmt::format("r{}", reg).c_str(), 32));
            }) |
            ranges::to<std::vector>();
  auto expr_a = emit_smt(ctx, a1, in, ifce.output_registers);
  auto expr_b = emit_smt(ctx, b1, in, ifce.output_registers);
  assert(expr_a.size() == expr_b.size());

  z3::solver slv(ctx);
  auto in_expr = z3::expr_vector(ctx);
  for (auto&& expr : in | ranges::view::values) { in_expr.push_back(expr); }
  for (auto&& [a, b] : ranges::view::zip(expr_a, expr_b)) { slv.add(z3::forall(in_expr, a == b)); }
  return slv.check() == z3::check_result::sat;
}

basic_block optimize(uarch::uarch const& ua, interface const& ifce, basic_block target, double min_score) {
  spdlog::info("optimizing basic block, inputs: {}, outputs: {}\n{}", ifce.input_registers, ifce.output_registers,
               target);

  static constexpr std::array<uint64_t, 5> initial_tests = {uint64_t(-2), uint64_t(-1), 0, 1, 2};
  spdlog::debug("preparing {} tests...", initial_tests.size());
  auto tests = initial_tests | ranges::view::transform([&](uint64_t v) {
                 auto in = ifce.input_registers | ranges::view::transform([&](unsigned r) { return std::pair(r, v); }) |
                           ranges::to<std::vector>();
                 auto out = evaluate(target, in, ifce.output_registers).value();
                 return std::tuple(in, out);
               }) |
               ranges::to<std::vector>();
  spdlog::trace("prepared tests: {}", fmt::join(tests, ", "));

  z3::context z3ctx;
  auto in = ifce.input_registers | ranges::view::transform([&](unsigned reg) {
              return std::pair(reg, z3ctx.bv_const(fmt::format("r{}", reg).c_str(), 32));
            }) |
            ranges::to<std::vector>();
  auto target_smt = emit_smt(z3ctx, target, in, ifce.output_registers);
  spdlog::debug("target smt formulae: {}", fmt::join(ranges::view::zip(ifce.output_registers, target_smt), ", "));

  auto target_score = score(ifce, tests, target);

  auto best_score = target_score;
  auto best = target;
  spdlog::debug("initial score: {}", target_score);

  auto prng = std::default_random_engine{std::random_device{}()};

  auto start = std::chrono::steady_clock::now();
  uint64_t candidates_total = 0;
  uint64_t solver_queries = 0;

  auto trace_tick = start + std::chrono::seconds(1);
  size_t candidates_tick = 0;
  while (best_score < min_score) {
    auto now = std::chrono::steady_clock::now();
    if (now > trace_tick) {
      spdlog::trace("considered {} candidates, best score: {}", candidates_tick, best_score);
      trace_tick = now + std::chrono::seconds(1);
      candidates_tick = 0;
    }

    ++candidates_tick;
    ++candidates_total;

    auto candidate = target;
    mutate(ua, prng, candidate);

    synthesize_immediates(z3ctx, ifce, tests, candidate, in, target_smt, solver_queries);

    auto candidate_score = score(ifce, tests, candidate);
    auto alpha = std::min(1., candidate_score / target_score);
    if (std::uniform_real_distribution<double>{}(prng) > alpha) { continue; }

    bool is_best_score = candidate_score > best_score;

    target = candidate;
    target_score = candidate_score;

    if (!is_best_score) { continue; }

    if (check_tests(ifce, tests, target)) {
      auto candidate_smt = emit_smt(z3ctx, candidate, in, ifce.output_registers);

      ++solver_queries;

      z3::solver z3slv(z3ctx);
      z3slv.add(make_or(z3ctx, ranges::view::zip(candidate_smt, target_smt) | ranges::view::transform([](auto c_t) {
                                 return std::get<0>(c_t) != std::get<1>(c_t);
                               })));
      auto result = z3slv.check();
      if (result == z3::check_result::unsat) {
        spdlog::debug("found better basic block with score {}:\n{}", candidate_score, candidate);
        best_score = candidate_score;
        best = candidate;
      } else if (result == z3::check_result::sat) {
        auto inv = in | ranges::view::transform([&](std::pair<unsigned, z3::expr> in) {
                     return std::pair(std::get<0>(in), z3slv.get_model().eval(std::get<1>(in)).get_numeral_uint64());
                   }) |
                   ranges::to<std::vector>();
        auto outv = evaluate(best, inv, ifce.output_registers).value();
        spdlog::trace("adding test case: ({}, {})", inv, outv);
        tests.emplace_back(inv, outv);

        target_score = score(ifce, tests, target);
      }
    }
  }

  auto elapsed_time =
      std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count() / 1'000.;
  spdlog::info("finished with score {} after {} seconds, tried {} candidates, did {} solver queries\n{}", best_score,
               elapsed_time, candidates_total, solver_queries, best);

  return best;
}
