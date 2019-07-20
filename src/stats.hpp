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

#include <fmt/format.h>
#include <fmt/ostream.h>

class stats {
  uint64_t candidates_ = 0;
  uint64_t smt_queries_ = 0;

public:
  void on_new_candidate() { ++candidates_; }
  void on_smt_query() { ++smt_queries_; }

  friend stats& operator+=(stats& a, stats const& b) {
    a.candidates_ += b.candidates_;
    a.smt_queries_ += b.smt_queries_;
    return a;
  }

  friend std::ostream& operator<<(std::ostream& os, stats const& st) {
    fmt::print(os, "{} candidates, {} smt queries", st.candidates_, st.smt_queries_);
    return os;
  }
};
