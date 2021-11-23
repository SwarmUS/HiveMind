include(FetchContent)

# Adding modules
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR})
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} PARENT_SCOPE)

function(freertos_fetch_kernel)
    FetchContent_Declare(
        FreeRTOSKernel
        GIT_REPOSITORY https://github.com/FreeRTOS/FreeRTOS-Kernel
        GIT_TAG        V10.4.1-kernel-only
        GIT_PROGRESS   TRUE
    )
    set(FREERTOS_KERNEL FreeRTOSKernel)
    string(TOLOWER ${FREERTOS_KERNEL} FREERTOS_KERNEL_L)

    if(FREERTOS_KERNEL_PATH)
        message(INFO "FREERTOS_KERNEL_PATH specified, skipping fetch")
    endif()


    FetchContent_GetProperties(FREERTOS_KERNEL POPULATED FREERTOS_KERNEL_POPULATED)
    if(NOT FREERTOS_KERNEL_POPULATED)
        message("Cloning FreeRTOS Kernel")
        set(FETCHCONTENT_QUIET FALSE) # To see progress
        FetchContent_Populate(${FREERTOS_KERNEL})
    endif()

    set(FREERTOS_KERNEL_PATH ${${FREERTOS_KERNEL_L}_SOURCE_DIR} PARENT_SCOPE)

    # Creates a .clang-tidy file with no virtually no checks
    if (DISABLE_EXTERNAL_WARNINGS) 
        file(WRITE "${${FREERTOS_KERNEL_L}_SOURCE_DIR}/.clang-tidy" 
"
---
Checks: '-*,llvm-twine-local'
...
"
        )
    endif()

endfunction()
