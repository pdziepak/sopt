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

#include "path.hpp"

#include <range/v3/algorithm/none_of.hpp>
#include <range/v3/to_container.hpp>
#include <range/v3/view/concat.hpp>
#include <range/v3/view/join.hpp>
#include <range/v3/view/map.hpp>
#include <range/v3/view/transform.hpp>
#include <range/v3/view/zip.hpp>

#include <fmt/format.h>
#include <fmt/ostream.h>
#include <fmt/ranges.h>

#include <spdlog/spdlog.h>

#include "../mutate.hpp"
#include "../perf.hpp"

namespace optimizer {

path::path(std::default_random_engine& prng, z3::solver& z3slv, target& trgt)
    : prng_(&prng), z3slv_(&z3slv), target_(&trgt), current_(target_->best_), current_score_(target_->best_score_) {
}

bool has_unknown_immediates(basic_block const& bb) {
  for (auto& inst : bb.instructions_) {
    for (auto& op : inst.operands_) {
      if (!op.is_known()) { return true; }
    }
  }
  return false;
}

void path::synthesize_immediates(basic_block& candidate, stats& st) {
  // FIXME: clean this up~
  auto& ua = target_->ua_;
  auto& ifce = target_->ifce_;
  auto& tests = target_->tests_;
  auto& z3ctx = target_->z3ctx_;
  auto& params = target_->param_exprs_;
  auto& in = target_->in_exprs_;
  auto& target_smt = target_->target_smt_;

  if (!has_unknown_immediates(candidate)) { return; }

  // FIXME: this is is_valid() actually
  auto ret = evaluate(ua, candidate, std::get<0>(tests.front()), std::get<1>(tests.front()), ifce.output_registers);
  if (!ret) { return; }
  if (ranges::none_of(*ret, std::mem_fn(&value::is_undefined))) { return; }

  // FIXME: emit_smt
  auto ctx = smt_context(ua, z3ctx, params);
  for (auto [reg, expr] : in) {
    for (auto lane = 0u; lane < ua.lanes(); ++lane) { ctx.set_register(reg, expr[lane], lane); }
  }
  ctx.commit_pending_operations();
  for (auto& inst : candidate.instructions_) {
    for (auto lane = 0u; lane < ua.lanes(); ++lane) { inst.opcode_->emit_smt(ctx, lane, inst.operands_); }
    ctx.commit_pending_operations();
  }
  auto original_candidate_smt =
      ifce.output_registers |
      ranges::view::transform([&](unsigned reg) { return ctx.get_register(reg, 0).simplify(); }) |
      ranges::to<std::vector>();
  auto original_candidate_extra_smt = ctx.extra_restrictions();

  auto original_candidate = candidate;

  auto vars = make_vector(
      z3ctx.ctx_, ranges::view::concat(params | ranges::view::values, in | ranges::view::values | ranges::view::join));

  do {
    candidate = original_candidate;
    auto candidate_smt = original_candidate_smt;
    auto candidate_extra_smt = original_candidate_extra_smt;

    z3slv_->reset();

    for (auto& [paramv, inv, outv] : tests) {
      auto vals = make_vector(
          z3ctx.ctx_,
          ranges::view::concat(paramv | ranges::view::values, inv | ranges::view::values | ranges::view::join) |
              ranges::view::transform([&](value const& v) { return z3ctx.ctx_.bv_val(v.as_i32(), 32); }));
      auto cnd = (make_and(z3ctx.ctx_, ranges::view::zip(candidate_smt, outv) |
                                      ranges::view::transform([&](std::tuple<z3::expr, value> v) {
                                        return (std::get<0>(v) == z3ctx.ctx_.bv_val(std::get<1>(v).as_i32(), 32));
                                      })) &&
                  candidate_extra_smt);
      z3slv_->add(cnd.substitute(vars, vals));
    }

    st.on_smt_query();
    if (z3slv_->check() != z3::check_result::sat) { return; }

    assert(candidate_smt.size() == target_smt.size());
    ctx.resolve_unknown_immediates(z3slv_->get_model());

    auto ctx = smt_context(ua, z3ctx, params);
    for (auto [reg, expr] : in) {
      for (auto lane = 0u; lane < ua.lanes(); ++lane) { ctx.set_register(reg, expr[lane], lane); }
    }
    ctx.commit_pending_operations();
    for (auto& inst : candidate.instructions_) {
      for (auto lane = 0u; lane < ua.lanes(); ++lane) { inst.opcode_->emit_smt(ctx, lane, inst.operands_); }
      ctx.commit_pending_operations();
    }
    candidate_smt = ifce.output_registers |
                    ranges::view::transform([&](unsigned reg) { return ctx.get_register(reg, 0).simplify(); }) |
                    ranges::to<std::vector>();
    candidate_extra_smt = ctx.extra_restrictions();

    z3slv_->reset();
    z3slv_->add(make_or(z3ctx.ctx_,
                        ranges::view::zip(candidate_smt, target_smt) |
                            ranges::view::transform([&](auto&& a_b) { return std::get<0>(a_b) != std::get<1>(a_b); })));
    st.on_smt_query();
    auto result = z3slv_->check();
    if (result == z3::check_result::unsat) {
      return;
    } else {
      auto paramv = params | ranges::view::transform([&](std::pair<uint64_t, z3::expr> in) {
                      auto e = z3slv_->get_model().eval(std::get<1>(in));
                      uint64_t v;
                      if (!e.is_numeral_u64(v)) { v = 0; }
                      return std::pair(std::get<0>(in), value(v));
                    }) |
                    ranges::to<std::map>();
      auto inv =
          in | ranges::view::transform([&](std::pair<unsigned, std::vector<z3::expr>> in) {
            return std::pair(std::get<0>(in), std::get<1>(in) | ranges::view::transform([&](z3::expr const& var) {
                                                auto e = z3slv_->get_model().eval(var);
                                                uint64_t v;
                                                if (!e.is_numeral_u64(v)) { v = 0; }
                                                return value(v);
                                              }) | ranges::to<std::vector>()

            );
          }) |
          ranges::to<std::vector>();
      auto outv = evaluate(ua, target_->target_, paramv, inv, ifce.output_registers).value();
      spdlog::trace("adding test case (constant synthesis): ({}, {}, {})", paramv, inv, outv);
      tests.emplace_back(paramv, inv, outv);
    }
  } while (true);
}

bool path::check_tests(basic_block& bb) {
  for (auto [param, in, out] : target_->tests_) {
    auto actual_out = evaluate(target_->ua_, bb, param, in, target_->ifce_.output_registers);
    if (!actual_out || actual_out != out) { return false; }
  }
  return true;
}

void path::next_step(stats& st) {
  st.on_new_candidate();

  auto candidate = current_;
  mutate(target_->ua_, *prng_, target_->ifce_, candidate);

  synthesize_immediates(candidate, st);

  auto candidate_score = score(target_->ua_, target_->ifce_, target_->tests_, candidate);
  auto alpha = std::min(1., candidate_score / current_score_);
  if (std::uniform_real_distribution<double>{}(*prng_) > alpha) { return; }

  bool is_best_score = candidate_score > target_->best_score_;

  current_ = candidate;
  current_score_ = candidate_score;

  if (!is_best_score || !check_tests(current_)) { return; }

  auto [candidate_smt, candidate_extra_smt] = emit_smt(target_->ua_, target_->z3ctx_, candidate, target_->param_exprs_,
                                                       target_->in_exprs_, target_->ifce_.output_registers);

  st.on_smt_query();

  z3slv_->reset();
  z3slv_->add(make_or(target_->z3ctx_.ctx_,
                      ranges::view::zip(candidate_smt, target_->target_smt_) |
                          ranges::view::transform([](auto c_t) { return std::get<0>(c_t) != std::get<1>(c_t); })) ||
              !candidate_extra_smt);
  auto result = z3slv_->check();
  if (result == z3::check_result::unsat) {
    spdlog::debug("found better basic block with score {}:\n{}", candidate_score, candidate);
    target_->best_score_ = candidate_score;
    target_->best_ = candidate;
  } else if (result == z3::check_result::sat) {
    auto paramv = target_->param_exprs_ | ranges::view::transform([&](std::pair<uint64_t, z3::expr> in) {
                    auto e = z3slv_->get_model().eval(std::get<1>(in));
                    uint64_t v;
                    if (!e.is_numeral_u64(v)) { v = 0; }
                    return std::pair(std::get<0>(in), value(v));
                  }) |
                  ranges::to<std::map>();
    auto inv = target_->in_exprs_ | ranges::view::transform([&](std::pair<unsigned, std::vector<z3::expr>> in) {
                 return std::pair(std::get<0>(in), std::get<1>(in) | ranges::view::transform([&](z3::expr const& ex) {
                                                     auto e = z3slv_->get_model().eval(ex);
                                                     uint64_t v;
                                                     if (!e.is_numeral_u64(v)) { v = 0; }
                                                     return value(v);
                                                   }) | ranges::to<std::vector>());
               }) |
               ranges::to<std::vector>();
    auto outv = evaluate(target_->ua_, target_->best_, paramv, inv, target_->ifce_.output_registers).value();
    spdlog::trace("adding test case: ({}, {}, {})", paramv, inv, outv);
    target_->tests_.emplace_back(paramv, inv, outv);

    current_score_ = score(target_->ua_, target_->ifce_, target_->tests_, current_);
  }
}

} // namespace optimizer
