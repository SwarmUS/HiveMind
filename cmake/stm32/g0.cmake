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

set(STM32_G0_TYPES 
    G030xx G031xx G041xx G070xx G071xx G081xx   
)
set(STM32_G0_TYPE_MATCH 
    "G030.." "G031.." "G041.." "G070.." "G071.." "G081.." 
)
set(STM32_G0_RAM_SIZES 
     8K  8K  8K 36K 36K 36K
)
set(STM32_G0_CCRAM_SIZES 
     0K  0K  0K  0K  0K  0K
)

stm32_util_create_family_targets(G0)

target_compile_options(STM32::G0 INTERFACE 
    -mcpu=cortex-m0plus
)
target_link_options(STM32::G0 INTERFACE 
    -mcpu=cortex-m0plus
)
