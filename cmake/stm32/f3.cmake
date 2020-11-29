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

set(STM32_F3_TYPES 
    F301x8 F302x8 F302xC F302xE F303x8 F303xC 
    F303xE F318xx F328xx F334x8 F358xx F373xC 
    F378xx F398xx 
)
set(STM32_F3_TYPE_MATCH 
    "301.[68]" "302.[68]" "302.[BC]" "302.[ED]" "303.[68]" "303.[BC]" 
    "303.[ED]" "318.." "328.." "334.[468]" "358.." "373.[8BC]"
    "378.." "398.."    
)
set(STM32_F3_RAM_SIZES 
     16K  16K   0K  64K  16K   0K  
     80K  16K  16K  16K  48K   0K
     32K  80K
)
set(STM32_F3_CCRAM_SIZES 
      0K   0K   0K   0K   0K   0K  
      0K   0K   0K   0K   0K   0K
      0K   0K
)

stm32_util_create_family_targets(F3)

target_compile_options(STM32::F3 INTERFACE 
    -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard
)
target_link_options(STM32::F3 INTERFACE 
    -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard
)

function(stm32f3_get_memory_info DEVICE TYPE FLASH_SIZE RAM_SIZE)
    string(REGEX REPLACE "F3[0-9][0-9].([468BCDEFGHI])" "\\1" SIZE_CODE ${DEVICE})
    
    if(TYPE STREQUAL "F302xC")
        if(SIZE_CODE STREQUAL "C")
            set(RAM "40K")
        else()
            set(RAM "32K")
        endif()
    elseif(TYPE STREQUAL "F303xC")
        if(SIZE_CODE STREQUAL "C")
            set(RAM "48K")
        else()
            set(RAM "40K")
        endif()
    elseif(TYPE STREQUAL "F373xC")
        if(SIZE_CODE STREQUAL "B")
            set(RAM "24K")
        elseif(SIZE_CODE STREQUAL "C")
            set(RAM "32K")
        else()
            set(RAM "16K")
        endif()
    endif()
    
    if(RAM)
        set(${RAM_SIZE} ${RAM} PARENT_SCOPE)
    endif()
endfunction()
