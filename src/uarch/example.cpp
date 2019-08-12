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
  return uarch_builder{}    //
      .lanes(4)             //
      .gp_registers(16)     //
      .zero_gp_register(15) //
      ("add", operands{dst_reg, reg, reg | imm | param}, latency(2),
       [](auto dst, auto src1, auto src2) { dst = src1 + src2; }) //
      ("add.64", operands{dst_reg2, reg2, reg2}, latency(3),
       [](auto dst, auto src1, auto src2) { dst.set64(src1.value64() + src2.value64()); }) //
      ("mul", operands{dst_reg, reg, reg | imm | param}, latency(4),
       [](auto dst, auto src1, auto src2) { dst = src1 * src2; })                                                 //
      ("mov", operands{dst_reg, reg | imm | param}, latency(1), [](auto dst, auto src) { dst = src; })            //
      ("ld.param", operands{dst_reg, param_addr(reg)}, latency(8), [](auto dst, auto src) { dst = params(src); }) //
      ("ld.param.64", operands{dst_reg2, param2_addr(reg)}, latency(10),
       [](auto dst, auto src) { dst.set64(params64(src)); }) //
      ("shuf.down", operands{dst_reg, reg, imm}, latency(4),
       [](auto dst, auto src1, auto src2) { dst = src1.lane(src2); }) //
      .build();
}

} // namespace uarch
