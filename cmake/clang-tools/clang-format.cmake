find_program(CLANG_FORMAT "clang-format")

if(CLANG_FORMAT)
    set(SOURCE_DIR ${PROJECT_SOURCE_DIR}/src)

    file(GLOB_RECURSE ALL_SOURCE_FILES
        ${SOURCE_DIR}/*.[ch]pp
        ${SOURCE_DIR}/*.[ch]
        ${SOURCE_DIR}/*.[h]pp
        ${SOURCE_DIR}/*.[h]
    )

    add_custom_target(
        format
        COMMAND clang-format
        -i
        -style=file
        ${ALL_SOURCE_FILES})

endif()
