set(EXECUTABLE_NAME ${PROJECT_NAME})
if(${CMAKE_CROSSCOMPILING})
    set(EXECUTABLE_NAME ${EXECUTABLE_NAME}.elf)
endif()

set(PROJECT_SOURCES
    Main.cpp
)

propolis_fetch_populate()

add_executable(${EXECUTABLE_NAME} ${PROJECT_SOURCES})
target_link_libraries(${EXECUTABLE_NAME}
    PUBLIC
      SwarmUS::HiveMind::BSP::Linker
      SwarmUS::HiveMind::BSP 
      SwarmUS::HiveMind::OS
      SwarmUS::HiveMind::Logger
      SwarmUS::HiveMind::BittyBuzz
      SwarmUS::HiveMind::MessageHandler
      SwarmUS::HiveMind::Interloc
      SwarmUS::HiveMind::ApplicationInterface
)

if (${CMAKE_CROSSCOMPILING} AND (ENABLE_HARDWARE_TESTS OR DECAWAVE_TESTS))
    add_subdirectory(hardware-tests)
endif()

set_default(MAX_AGENTS_IN_SWARM 6)

add_subdirectory(bittybuzz)
add_subdirectory(bsp)
add_subdirectory(os)
add_subdirectory(logger)
add_subdirectory(message-handler)
add_subdirectory(interloc)
add_subdirectory(application-interface)

# Removing warnings from interface libraries
if (DISABLE_EXTERNAL_WARNINGS) 

    get_target_property(LINKER_SOURCES SwarmUS::HiveMind::BSP::Linker INTERFACE_SOURCES)
    set_source_files_properties(
        ${LINKER_SOURCES}
        PROPERTIES COMPILE_FLAGS
        -w #Disables all warnings
    )
    set_target_properties(
        ${EXECUTABLE_NAME}
        PROPERTIES C_CLANG_TIDY ""
    )
endif()
