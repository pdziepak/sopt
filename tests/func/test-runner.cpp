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

#include <charconv>
#include <fstream>

#include <fmt/ranges.h>

#include <spdlog/spdlog.h>

#include <range/v3/view.hpp>

#include "instruction.hpp"
#include "optimizer.hpp"
#include "uarch/example.hpp"

std::vector<std::string_view> split(std::string_view str, char c) {
  auto trim_and_add = [](std::vector<std::string_view>& strs, std::string_view str) {
    auto first = str.find_first_not_of(" \t");
    if (first == std::string_view::npos) { return; }

    auto last = str.find_last_not_of(" \t");
    strs.emplace_back(str.substr(first, last + 1));
  };

  auto strs = std::vector<std::string_view>{};
  while (!str.empty()) {
    auto pos = str.find(c);
    if (pos == std::string_view::npos) {
      trim_and_add(strs, str);
      str.remove_prefix(str.size());
    }
    trim_and_add(strs, str.substr(0, pos));
    str.remove_prefix(pos + 1);
  }
  return strs;
}

instruction parse_instruction(uarch::uarch const& ua, std::string_view line) {
  auto pos = line.find_first_of(" \t");
  auto opcode = ua.get_opcode(line.substr(0, pos));
  line.remove_prefix(pos);

  auto args = split(line, ',');
  auto operands = std::vector<operand>{};
  for (auto arg : args) {
    bool reg = false;
    bool param = false;
    if (arg.front() == 'r') {
      reg = true;
      arg.remove_prefix(1);
    } else if (arg.front() == 'p') {
      arg.remove_prefix(2);
      param = true;
    }
    uint64_t value = 0;
    auto [p, ec] = std::from_chars(arg.begin(), arg.end() - param, value, param ? 16 : 10);
    if (ec != std::errc{}) { throw ec; }
    if (reg) {
      operands.emplace_back(operand::make_register(value));
    } else if (param) {
      operands.emplace_back(operand::make_parameter(value));
    } else {
      operands.emplace_back(operand::make_immediate(value));
    }
  }
  return instruction{opcode, std::move(operands)};
}

std::tuple<std::vector<uint64_t>, std::vector<unsigned>> parse_input_interface(std::string_view str) {
  auto param = std::vector<uint64_t>{};
  auto regs = std::vector<unsigned>{};
  for (auto arg : split(str, ' ')) {
    if (arg.front() == 'r') {
      unsigned value = 0;
      auto [p, ec] = std::from_chars(arg.begin() + 1, arg.end(), value);
      if (ec != std::errc{}) { throw ec; }
      regs.emplace_back(value);
    } else if (arg.front() == 'p') {
      auto pos = arg.find(']');
      if (arg.data()[1] != '[' || pos == arg.npos) { throw std::runtime_error("parser error"); }
      uint64_t value = 0;
      auto [p, ec] = std::from_chars(arg.begin() + 2, arg.begin() + pos, value, 16);
      if (ec != std::errc{}) { throw ec; }
      param.emplace_back(value);
    }
  }
  return {std::move(param), std::move(regs)};
}

std::vector<unsigned> parse_output_interface(std::string_view str) {
  auto strs = split(str, ' ');
  return strs | ranges::view::transform([](std::string_view arg) {
           if (arg.front() != 'r') { throw std::runtime_error("expected a register"); }
           unsigned value = 0;
           auto [p, ec] = std::from_chars(arg.begin() + 1, arg.end(), value);
           if (ec != std::errc{}) { throw ec; }
           return value;
         }) |
         ranges::to<std::vector>();
}

int main(int argc, char** argv) {
  spdlog::set_level(spdlog::level::trace);

  if (argc != 2) {
    fmt::print("Usage:\n\t{} test-file\n", argv[0]);
    return 1;
  }

  auto uarch = uarch::make_example();

  auto test = [&] {
    try {
      auto file = std::ifstream{};
      file.exceptions(std::ifstream::failbit | std::ifstream::badbit);
      file.open(argv[1]);
      return std::string((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>{});
    } catch (std::exception const& ex) {
      spdlog::critical("failed to read test file \"{}\": {}", argv[1], ex.what());
      std::exit(1);
    }
  }();
  auto lines = split(test, '\n');

  uint64_t total = 0;
  uint64_t failed = 0;

  auto ifce = interface{};
  auto input = std::optional<basic_block>{};
  auto actual_output = basic_block{};
  auto expected_output = basic_block{};
  basic_block* target = {};

  auto run_previous = [&] {
    if (!input) { return; }

    ++total;
    spdlog::info("running test case #{}...", total);

    auto expected_score = score_performance(*uarch, ifce, expected_output);
    try {
      actual_output = optimize(*uarch, ifce, *input, expected_score);

      if (!equivalent(*uarch, ifce, actual_output, expected_output)) {
        spdlog::error("actual output not equivalent to expected output\nactual:\n{}\nexpected:\n{}", actual_output,
                      expected_output);
        ++failed;
        return;
      }
    } catch (z3::exception const& ex) {
      spdlog::error("z3 error: {}", ex);
      ++failed;
      return;
    }
    auto actual_score = score_performance(*uarch, ifce, actual_output);
    if (expected_score > actual_score) {
      spdlog::error("actual output scores worse ({}) than expected output ({})\nactual:\n{}\nexpected:\n{}",
                    actual_score, expected_score, actual_output, expected_output);
      ++failed;
    } else if (actual_score > expected_score) {
      spdlog::warn("actual output scores better ({}) than expected output ({})\nactual:\n{}\nexpected:\n{}",
                   actual_score, expected_score, actual_output, expected_output);
    }
  };

  for (auto line : lines) {
    if (line.front() == '#') {
      continue;
    } else if (line.front() == '<') {
      run_previous();
      input.emplace();
      target = &*input;
      std::tie(ifce.parameters, ifce.input_registers) = parse_input_interface(line.substr(1));
    } else if (line.front() == '>') {
      expected_output = {};
      target = &expected_output;
      ifce.output_registers = parse_output_interface(line.substr(1));
    } else {
      target->instructions_.emplace_back(parse_instruction(*uarch, line));
    }
  }

  run_previous();

  if (failed) {
    spdlog::error("ran {} test cases, {} failed", total, failed);
    return 1;
  } else {
    spdlog::info("OK, all {} test casses passed", total);
  }
}