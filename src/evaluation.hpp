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

namespace uarch {
class uarch;
}
class basic_block;

class evaluation_context {
  std::unordered_map<uint64_t, uint64_t> parameters_;

  std::vector<uint64_t> registers_;
  std::vector<bool> defined_registers_;

public:
  using value_type = uint64_t;

  explicit evaluation_context(uarch::uarch const& ua, std::unordered_map<uint64_t, uint64_t> const& params);

  uint64_t get_register(unsigned r) const;
  void set_register(unsigned r, uint64_t v);
  bool is_register_defined(unsigned r) const;

  uint64_t get_parameter(uint64_t p) const;
  bool is_valid_parameter(uint64_t p) const;
  bool is_valid_parameter64(uint64_t p) const;

  static uint64_t make_u64(uint32_t lo, uint32_t hi);
  static std::tuple<uint32_t, uint32_t> split_u64(uint64_t v);
};

std::optional<std::vector<uint64_t>> evaluate(uarch::uarch const& ua, basic_block& bb,
                                              std::unordered_map<uint64_t, uint64_t> const& params,
                                              std::vector<std::pair<unsigned, uint64_t>> in,
                                              std::vector<unsigned> const& out);
