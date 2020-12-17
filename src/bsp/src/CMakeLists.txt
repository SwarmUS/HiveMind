add_subdirectory(common)

if (${COMPILE_STM32_F429ZI})
    add_subdirectory(stm32f4)
else()
    add_subdirectory(posix)
endif()
