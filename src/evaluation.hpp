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

#include <cassert>
#include <cstdint>
#include <unordered_map>
#include <vector>

class evaluation_context {
  std::unordered_map<uint64_t, uint64_t> parameters_;

  std::vector<uint64_t> registers_;
  std::vector<bool> defined_registers_;

public:
  using value_type = uint64_t;

  explicit evaluation_context(std::unordered_map<uint64_t, uint64_t> const& params)
      : parameters_(params), registers_(9), defined_registers_(9) {}

  uint64_t get_register(unsigned r) const {
    assert(r < registers_.size());
    assert(defined_registers_[r]);
    return registers_[r];
  }
  void set_register(unsigned r, uint64_t v) {
    assert(r < registers_.size());
    registers_[r] = v;
    defined_registers_[r] = true;
  }
  bool is_register_defined(unsigned r) const { return defined_registers_[r]; }

  uint64_t get_parameter(uint64_t p) const { return parameters_.at(p); }
  bool is_valid_parameter(uint64_t p) const { return parameters_.count(p); }
  bool is_valid_parameter64(uint64_t p) const { return is_valid_parameter(p) && is_valid_parameter(p + 4); }

  static uint64_t make_u64(uint32_t lo, uint32_t hi) { return (uint64_t(hi) << 32) | lo; }
  static std::tuple<uint32_t, uint32_t> split_u64(uint64_t v) { return {uint32_t(v), uint32_t(v >> 32)}; }
};