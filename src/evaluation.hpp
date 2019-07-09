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
#include <vector>

class evaluation_context {
  std::vector<uint64_t> registers_;
  std::vector<bool> defined_registers_;

public:
  evaluation_context() : registers_(9), defined_registers_(9) {}

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
};