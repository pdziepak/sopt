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

#include <bitset>
#include <cassert>
#include <cstdint>
#include <map>
#include <unordered_map>
#include <vector>

namespace uarch {
class uarch;
}
class basic_block;

class value {
  bool defined_ = false;
  uint64_t integer_{};

public:
  constexpr value() = default;
  constexpr explicit value(uint64_t v) : defined_(true), integer_(v) {}

  constexpr bool is_undefined() const { return !defined_; }

  void trim() {
    if (!is_undefined()) { integer_ = uint32_t(integer_); }
  }
  constexpr uint32_t as_i32() const {
    assert(!is_undefined());
    return integer_;
  }
  constexpr uint64_t as_i64() const {
    assert(!is_undefined());
    return integer_;
  }

  friend value operator+(value const& a, uint64_t b) {
    if (a.is_undefined()) { return {}; }
    return value(a.integer_ + b);
  }
  friend value operator+(value const& a, value const& b) {
    if (a.is_undefined() || b.is_undefined()) { return {}; }
    return value(a.integer_ + b.integer_);
  }

  friend value operator*(value const& a, value const& b) {
    if (a.is_undefined() || b.is_undefined()) { return {}; }
    return value(a.integer_ * b.integer_);
  }

  friend bool operator==(value const& a, value const& b) {
    return a.defined_ == b.defined_ && a.integer_ == b.integer_;
  }
  friend bool operator!=(value const& a, value const& b) { return !(a == b); }

  friend std::ostream& operator<<(std::ostream&, value const&);
};

class evaluation_context {
  uarch::uarch const* ua_;
  std::map<uint64_t, value> const* parameters_;

  unsigned zero_register_;
  std::vector<std::vector<value>> registers_;
  std::bitset<256> defined_registers_;

  struct pending_op {
    unsigned lane;
    unsigned reg;
    value val;
  };
  std::vector<pending_op> pending_register_writes_;

public:
  using value_type = value;

  explicit evaluation_context(uarch::uarch const& ua, std::map<uint64_t, value> const& params);
  void reset(uarch::uarch const& ua, std::map<uint64_t, value> const& params);

  value get_register(unsigned r, unsigned lane) const;
  value get_register(unsigned r, value lane) const;
  void set_register(unsigned r, value v, unsigned lane);
  bool is_register_defined(unsigned r) const;

  void commit_pending_operations();

  value get_parameter(value p) const;
  bool is_valid_parameter(value p) const;
  bool is_valid_parameter64(value p) const;

  static value make_u64(value lo, value hi);
  static std::tuple<value, value> split_u64(value v);
};

std::optional<std::vector<value>> evaluate(uarch::uarch const& ua, basic_block& bb,
                                           std::map<uint64_t, value> const& params,
                                           std::vector<std::pair<unsigned, std::vector<value>>> const& in,
                                           std::vector<unsigned> const& out);
