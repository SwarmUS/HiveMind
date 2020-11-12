get_filename_component(STM32_CMAKE_DIR ${CMAKE_CURRENT_LIST_FILE} DIRECTORY)

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
find_program(CMAKE_C_COMPILER NAMES ${STM32_TARGET_TRIPLET}-gcc PATHS ${TOOLCHAIN_BIN_PATH})
find_program(CMAKE_CXX_COMPILER NAMES ${STM32_TARGET_TRIPLET}-g++ PATHS ${TOOLCHAIN_BIN_PATH})
find_program(CMAKE_ASM_COMPILER NAMES ${STM32_TARGET_TRIPLET}-gcc PATHS ${TOOLCHAIN_BIN_PATH})


add_compile_options(
    --specs=nosys.specs
    -mcpu=cortex-m4 
    -mfpu=fpv4-sp-d16 
    -mfloat-abi=hard
)

add_link_options(
    --specs=nosys.specs
    -mcpu=cortex-m4 
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard
)

include(stm32/common)

set(COMPILE_STM32_F429ZI 1)
set(ENABLE_TESTS OFF)
set(OPENOCD_CFG_PATH ${STM32_CMAKE_DIR}/openocd/stm32_f4.cfg)



