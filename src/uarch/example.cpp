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

#include "uarch.hpp"

namespace uarch {

std::unique_ptr<uarch> make_example() {
  using namespace operand_descriptors;
  return uarch_builder{}                                                                                            //
  ("add", operands{dst_reg, reg, reg | imm | param}, [](auto dst, auto src1, auto src2) { dst = src1 + src2; })     //
      ("mul", operands{dst_reg, reg, reg | imm | param}, [](auto dst, auto src1, auto src2) { dst = src1 * src2; }) //
      ("mov", operands{dst_reg, reg | imm | param}, [](auto dst, auto src) { dst = src; })                          //
          .build();
}

} // namespace uarch
