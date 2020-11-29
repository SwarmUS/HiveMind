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

set(STM32_F2_TYPES 
    F205xx F215xx F207xx F217xx
)
set(STM32_F2_TYPE_MATCH 
    "F205.." "F215.." "F207.." "F217.."
)
set(STM32_F2_RAM_SIZES 
      0K 128K 128K 128K 
)
set(STM32_F2_CCRAM_SIZES 
      0K   0K   0K   0K
)

stm32_util_create_family_targets(F2)

target_compile_options(STM32::F2 INTERFACE 
    -mcpu=cortex-m3
)
target_link_options(STM32::F2 INTERFACE 
    -mcpu=cortex-m3
)

function(stm32f2_get_memory_info DEVICE TYPE FLASH_SIZE RAM_SIZE)
    string(REGEX REPLACE "F2[0-9][0-9].([468BCDEFGHI])" "\\1" SIZE_CODE ${DEVICE})
    
    if(TYPE STREQUAL "F205xx")
        if(SIZE_CODE STREQUAL "B")
            set(RAM "64K")
        elseif(SIZE_CODE STREQUAL "C")
            set(RAM "96K")
        else()
            set(RAM "128K")
        endif()
    endif()
    
    if(RAM)
        set(${RAM_SIZE} ${RAM} PARENT_SCOPE)
    endif()
endfunction()
