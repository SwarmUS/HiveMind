find_program(CLANG_FORMAT "clang-format")
if(CLANG_FORMAT)
    set(SOURCE_DIR ${PROJECT_SOURCE_DIR}/src)

    add_custom_target(
        format
        COMMAND 
        ${PROJECT_SOURCE_DIR}/tools/run-clang-format.py
        --recursive
        --in-place
        --style=file
        ${SOURCE_DIR})

    add_custom_target(
        check-format
        COMMAND 
        ${PROJECT_SOURCE_DIR}/tools/run-clang-format.py
        --recursive
        --style=file
        ${SOURCE_DIR})

endif()
