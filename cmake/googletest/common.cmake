include(FetchContent)

function(googletest_get_populate)
    FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG        release-1.8.0
    )

    FetchContent_GetProperties(googletest)
    if(NOT googletest_POPULATED)
        FetchContent_Populate(googletest)
        add_subdirectory(${googletest_SOURCE_DIR} ${googletest_BINARY_DIR})
    endif()

    # Disable warnings only if external warnings is enabled
    if(NOT ENABLE_EXTERNAL_WARNINGS)
        target_compile_options(gtest PRIVATE -w)
        target_compile_options(gtest_main PRIVATE -w)
        target_compile_options(gmock PRIVATE -w)
        target_compile_options(gmock_main PRIVATE -w)
    endif()

endfunction()

