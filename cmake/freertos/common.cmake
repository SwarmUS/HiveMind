include(FetchContent)

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
endfunction()

macro(disable_freertos_warnings)
    if(FreeRTOS_SOURCE_DIR STREQUAL "")
      message(FATAL_ERROR "FreeRTOS_SOURCE_DIR is not set, call this macro after fetching freertos kernel")
    endif()
    # We need to do this since the FreeRTOS::LIB_NAME are interfaces and not static libraries
    # Using GLOB_RECURSE is bad practice, but since it's an external lib and we don't add files during dev, it's fine
    file(GLOB_RECURSE ALL_FREERTOS_FILES
        ${FreeRTOS_SOURCE_DIR}/*.[ch]pp
        ${FreeRTOS_SOURCE_DIR}/*.[ch]
        ${FreeRTOS_SOURCE_DIR}/*.[h]pp
        ${FreeRTOS_SOURCE_DIR}/*.[h]
    )
    set_source_files_properties(
        ${ALL_FREERTOS_FILES}
        PROPERTIES COMPILE_FLAGS
        -w #Disables all warnings
    )
endmacro()
