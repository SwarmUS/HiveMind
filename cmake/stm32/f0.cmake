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

set(STM32_F0_TYPES 
    F030x6 F030x8 F031x6 F038xx F042x6 F048xx F051x8 F058xx 
    F070x6 F070xB F071xB F072xB F078xx F091xC F098xx F030xC
)
set(STM32_F0_TYPE_MATCH 
    "F030.[46]" "F030.8" "F031.[46]" "F038.." "F042.[46]" "F048.." "F051.[468]" "F058.."
    "F070.6" "F070.B" "F071.[8B]" "F072.[8B]" "F078.." "F091.[BC]" "F098.." "F030.C"
)
set(STM32_F0_RAM_SIZES 
     4K  8K  4K  4K  6K  6K  8K  8K
     6K 16K 16K 16K 16K 32K 32K 32K
)
set(STM32_F0_CCRAM_SIZES 
     0K  0K  0K  0K  0K  0K  0K  0K
     0K  0K  0K  0K  0K  0K  0K  0K
)

stm32_util_create_family_targets(F0)

target_compile_options(STM32::F0 INTERFACE 
    -mcpu=cortex-m0
)
target_link_options(STM32::F0 INTERFACE 
    -mcpu=cortex-m0
)
