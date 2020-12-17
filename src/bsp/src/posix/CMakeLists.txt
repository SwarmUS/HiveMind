set(LIB_NAME "swarmus-hivemind-bsp-posix")
set(LIB_ALIAS "SwarmUS::HiveMind::BSP::Platform")

set(LIB_SOURCES
    src/BSP.cpp
    src/UserInterface.cpp
)

set(LIB_PRIVATE_HEADERS

)

add_library(${LIB_NAME} ${LIB_SOURCES} ${LIB_PRIVATE_HEADERS})
add_library(${LIB_ALIAS} ALIAS ${LIB_NAME})

target_include_directories(${LIB_NAME} 
    PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        ${BSP_CMAKE_DIR}/include
)


target_link_libraries(${LIB_NAME}
    PUBLIC
        # Defined libraries
        SwarmUS::HiveMind::FreeRTOS
)

target_compile_options(${LIB_NAME}
    PRIVATE
        $<$<BOOL:${ENABLE_WARNINGS}>:${WARNING_FLAG}>
        $<$<BOOL:${ENABLE_WARNINGS_AS_ERROR}>:${WARNING_AS_ERROR_FLAG}>
)


# Mock linker library
add_library(posix-linker INTERFACE)
add_library(${LIB_BSP_LINKER_ALIAS} ALIAS posix-linker)

add_custom_target(
    run
    COMMAND $<TARGET_FILE:${EXECUTABLE_NAME}>
)
