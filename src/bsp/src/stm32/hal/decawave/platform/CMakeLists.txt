
#TODO: WIP deca port
if (${COMPILE_STM32})
    add_subdirectory(stm32f429zi)
else()
    message(FATAL_ERROR "The current board has no decawave port")
endif()
