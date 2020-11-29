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

set(STM32_L1_TYPES 
    L100xB  L100xBA L100xC  L151xB  L151xBA L151xC  L151xCA L151xD 
    L151xDX L151xE  L152xB  L152xBA L152xC  L152xCA L152xD  L152xDX
    L152xE  L162xC  L162xCA L162xD  L162xDX L162xE
)
set(STM32_L1_TYPE_MATCH 
    "L100.[68B]" "L100.[68B]A" "L100.C" "L151.[68B]" "L151.[68B]A" "L151.C" "L151.CA" "L151.D"
    "L151.DX" "L151.E" "L152.[68B]" "L152.[68B]A" "L152.C" "L152.CA" "L152.D" "L152.DX"
    "L152.E" "L162.C" "L162.CA" "L162.D" "L162.DX" "L162.E"
)
set(STM32_L1_RAM_SIZES 
     0K  0K 16K  0K  0K 32K 32K 48K
    80K 80K  0K  0K 32K 32K 48K 80K
    80K 32K 32K 48K 80K 80K
)
set(STM32_L1_CCRAM_SIZES 
     0K  0K  0K  0K  0K  0K  0K  0K
     0K  0K  0K  0K  0K  0K  0K  0K
     0K  0K  0K  0K  0K  0K
)

stm32_util_create_family_targets(L1)

target_compile_options(STM32::L1 INTERFACE 
    -mcpu=cortex-m3
)
target_link_options(STM32::L1 INTERFACE 
    -mcpu=cortex-m3
)

function(stm32l1_get_memory_info DEVICE TYPE FLASH_SIZE RAM_SIZE)
    string(REGEX REPLACE "L1[0-9][0-9].([68BCDE])" "\\1" SIZE_CODE ${DEVICE})
    
    unset(RAM)
    
    if((TYPE STREQUAL "L100xB"))
        if(SIZE_CODE STREQUAL "6")
            set(RAM "4K")
        elseif(SIZE_CODE STREQUAL "8")
            set(RAM "8K")
        elseif(SIZE_CODE STREQUAL "B")
            set(RAM "10K")
        endif()
    elseif((TYPE STREQUAL "L100xBA"))
        if(SIZE_CODE STREQUAL "6")
            set(RAM "4K")
        elseif(SIZE_CODE STREQUAL "8")
            set(RAM "8K")
        elseif(SIZE_CODE STREQUAL "B")
            set(RAM "16K")
        endif()
    elseif((TYPE STREQUAL "L151xB"))
        if(SIZE_CODE STREQUAL "6")
            set(RAM "10K")
        elseif(SIZE_CODE STREQUAL "8")
            set(RAM "10K")
        elseif(SIZE_CODE STREQUAL "B")
            set(RAM "16K")
        endif()
    elseif((TYPE STREQUAL "L151xBA"))
        if(SIZE_CODE STREQUAL "6")
            set(RAM "16K")
        elseif(SIZE_CODE STREQUAL "8")
            set(RAM "32K")
        elseif(SIZE_CODE STREQUAL "B")
            set(RAM "32K")
        endif()
    elseif((TYPE STREQUAL "L152xB"))
        if(SIZE_CODE STREQUAL "6")
            set(RAM "10K")
        elseif(SIZE_CODE STREQUAL "8")
            set(RAM "10K")
        elseif(SIZE_CODE STREQUAL "B")
            set(RAM "16K")
        endif()
    elseif((TYPE STREQUAL "L152xBA"))
        if(SIZE_CODE STREQUAL "6")
            set(RAM "16K")
        elseif(SIZE_CODE STREQUAL "8")
            set(RAM "32K")
        elseif(SIZE_CODE STREQUAL "B")
            set(RAM "32K")
        endif()
    endif()
    
    if(RAM)
        set(${RAM_SIZE} ${RAM} PARENT_SCOPE)
    endif()
endfunction()
