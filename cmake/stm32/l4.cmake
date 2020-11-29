# MIT License

# Copyright (c) 2012-2017 Konstantin Oblaukhov

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
# See https://github.com/ObKo/stm32-cmake for more information

set(STM32_L4_TYPES 
    L412xx L422xx L431xx L432xx L433xx L442xx 
    L443xx L451xx L452xx L462xx L471xx L475xx 
    L476xx L485xx L486xx L496xx L4A6xx L4P5xx 
    L4Q5xx L4R5xx L4R7xx L4R9xx L4S5xx L4S7xx 
    L4S9xx
)
set(STM32_L4_TYPE_MATCH 
   "L412.." "L422.." "L431.." "L432.." "L433.." "L442.." 
   "L443.." "L451.." "L452.." "L462.." "L471.." "L475.." 
   "L476.." "L485.." "L486.." "L496.." "L4A6.." "L4P5.." 
   "L4Q5.." "L4R5.." "L4R7.." "L4R9.." "L4S5.." "L4S7.." 
   "L4S9.."
)

set(STM32_L4_RAM_SIZES 
     40K  40K  64K  64K  64K  64K
     64K 160K 160K 160K 128K 128K
    128K 128K 128K 320K 320K 320K 
    320K 640K 640K 640K 640K 640K
    640K
)
set(STM32_L4_CCRAM_SIZES 
      8K   8K  16K  16K  16K  16K
     16K  32K  32K  32K  32K  32K
     32K  32K  32K  64K  64K  64K
     64K  64K  64K  64K  64K  64K
     64K
)

stm32_util_create_family_targets(L4)

target_compile_options(STM32::L4 INTERFACE 
    -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard
)
target_link_options(STM32::L4 INTERFACE 
    -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard
)
