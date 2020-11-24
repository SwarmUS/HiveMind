set(LIB_NAME "freertos")
set(LIB_ALIAS "SwarmUS::HiveMind::FreeRTOS")

include(freertos/common)

freertos_fetch_kernel()
add_subdirectory(platforms)


set(LIB_HEADERS
    include/freertos/FreeRTOSConfig.h
)

add_library(${LIB_NAME} STATIC ${LIB_HEADERS})
add_library(${LIB_ALIAS} ALIAS ${LIB_NAME})


find_package(FreeRTOS COMPONENTS
            NONE
            REQUIRED)



target_include_directories(${LIB_NAME}
    PUBLIC
       $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>

       # Allows freertos to build properly
       $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/freertos>
       $<INSTALL_INTERFACE:include>

)

target_link_libraries(${LIB_NAME}
    PUBLIC
        FreeRTOS::Timers
        FreeRTOS::Heap::1
        SwarmUS::HiveMind::FreeRTOS::Platform
)




# Removing warnings from freertos compilation on executable target
if (DISABLE_EXTERNAL_WARNINGS) 
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
        TARGET_DIRECTORY
        ${EXECUTABLE_NAME}
        PROPERTIES COMPILE_FLAGS
        -w #Disables all warnings
    )
endif()
