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

#include <spdlog/spdlog.h>

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
    if (arg.front() == 'r') {
      reg = true;
      arg.remove_prefix(1);
    }
    uint64_t value = 0;
    auto [p, ec] = std::from_chars(arg.begin(), arg.end(), value);
    if (ec != std::errc{}) { throw ec; }
    operands.emplace_back(reg ? operand::make_register(value) : operand::make_immediate(value));
  }
  return instruction{opcode, std::move(operands)};
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

  auto input = std::optional<basic_block>{};
  auto actual_output = basic_block{};
  auto expected_output = basic_block{};
  basic_block* target = {};

  auto run_previous = [&] {
    if (!input) { return; }

    ++total;
    spdlog::info("running test case #{}...", total);

    auto expected_score = score_performance(expected_output);
    actual_output = optimize(*uarch, *input, expected_score);
    if (!equivalent(actual_output, expected_output)) {
      spdlog::error("actual output not equivalent to expected output\nactual:\n{}\nexpected:\n{}", actual_output,
                    expected_output);
      ++failed;
      return;
    }
    auto actual_score = score_performance(actual_output);
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
    } else if (line.front() == '>') {
      expected_output = {};
      target = &expected_output;
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