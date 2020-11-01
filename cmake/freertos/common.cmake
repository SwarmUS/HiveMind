include(FetchContent)

function(freertos_fetch)
    FetchContent_Declare(
        FreeRTOS
        GIT_REPOSITORY https://github.com/FreeRTOS/FreeRTOS-Kernel
        GIT_TAG        V10.4.1-kernel-only
        GIT_PROGRESS   TRUE
    )
    set(FREERTOS FreeRTOS)
    string(TOLOWER ${FREERTOS} FREERTOS_L)

    if(FREERTOS_KERNEL_PATH)
        message(INFO "FREERTOS_KERNEL_PATH specified, skipping fetch")
    endif()


    FetchContent_GetProperties(FREERTOS POPULATED FREERTOS_POPULATED)
    if(NOT CUBE_POPULATED)
        message("Cloning FreeRTOS")
        set(FETCHCONTENT_QUIET FALSE) # To see progress
        FetchContent_Populate(${FREERTOS})
    endif()

    set(FREERTOS_KERNEL_PATH ${${FREERTOS_L}_SOURCE_DIR} PARENT_SCOPE)
    message("End function: Path = ${FREERTOS_KERNEL_PATH}, source=${${FREERTOS_L}_SOURCE_DIR}")
endfunction()