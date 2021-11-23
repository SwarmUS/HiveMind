find_program (CLANG_TIDY_EXE NAMES "clang-tidy")
if(CLANG_TIDY_EXE AND ENABLE_CLANG_TIDY_CHECK)
    set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

    # This dreadful mess is to communicate to clang-tidy the C++ system includes. It seems that CMake
    # doesn't support using its own compile_commands.json database, and that clang-tidy doesn't
    # pick up non-default system headers.
    # Source: https://gitlab.kitware.com/cmake/cmake/-/issues/20912#note_793338
    execute_process(
            COMMAND bash -c "${CMAKE_CXX_COMPILER} -x c++ -Wp,-v /dev/null 2>&1 > /dev/null | grep '^ /'"
            OUTPUT_VARIABLE COMPILER_HEADERS
            OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    string(REGEX REPLACE "[ \n\t]+" ";" INCLUDE_COMPILER_HEADERS ${COMPILER_HEADERS})

    set(CMAKE_CXX_STANDARD_INCLUDE_DIRECTORIES ${CMAKE_CXX_STANDARD_INCLUDE_DIRECTORIES} ${INCLUDE_COMPILER_HEADERS})

    execute_process(
        COMMAND bash -c "${CMAKE_C_COMPILER} -x c -Wp,-v /dev/null 2>&1 > /dev/null | grep '^ /'"
        OUTPUT_VARIABLE COMPILER_HEADERS
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    string(REGEX REPLACE "[ \n\t]+" ";" INCLUDE_COMPILER_HEADERS ${COMPILER_HEADERS})
    set(CMAKE_C_STANDARD_INCLUDE_DIRECTORIES ${CMAKE_CXX_STANDARD_INCLUDE_DIRECTORIES} ${INCLUDE_COMPILER_HEADERS})
    
    #Setting clang tidy
    set(CMAKE_CXX_CLANG_TIDY "${CLANG_TIDY_EXE};--extra-arg=-gcc-toolchain" CACHE STRING "" FORCE)
    set(CMAKE_C_CLANG_TIDY "${CLANG_TIDY_EXE};--extra-arg=-gcc-toolchain" CACHE STRING "" FORCE)

else()
    message(STATUS "clang-tidy NOT found!")

    if(ENABLE_ERROR_ON_MISSING_TOOL AND ENABLE_CLANG_TIDY_CHECK)
        message(FATAL_ERROR "Install clang-tidy or disable ENABLE_ERROR_ON_MISSING_TOOL ")
    endif()
endif()

