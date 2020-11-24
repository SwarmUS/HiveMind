 find_program (CLANG_TIDY_EXE NAMES "clang-tidy")
if(CLANG_TIDY_EXE)
    set(CMAKE_CXX_CLANG_TIDY "${CLANG_TIDY_EXE}" CACHE STRING "" FORCE)
else()
    message(STATUS "clang-tidy NOT found!")
    set(CMAKE_CXX_CLANG_TIDY "" CACHE STRING "" FORCE) # clear it
endif()


find_program(CLANG_TIDY "clang-tidy")
if(CLANG_TIDY)
    set(SOURCE_DIR ${PROJECT_SOURCE_DIR}/src)
    set(CMAKE_EXPORT_COMPILE_COMMANDS 1) # export compilation database

    file(GLOB_RECURSE ALL_SOURCE_FILES
        ${SOURCE_DIR}/*.[ch]pp
        ${SOURCE_DIR}/*.[ch]
        ${SOURCE_DIR}/*.[h]pp
        ${SOURCE_DIR}/*.[h]
    )

    add_custom_target(
        tidy
        COMMAND clang-tidy
        --format-style=file
        -p ${CMAKE_CURRENT_BINARY_DIR}
        ${ALL_SOURCE_FILES})

endif()
