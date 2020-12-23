include(FetchContent)

function(googletest_get_populate)
    FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG        release-1.10.0
    )

    if (TARGET gtest)
        message("-- gtest already provided by catkin. Skipping install")
    else()
        FetchContent_GetProperties(googletest)
        if(NOT googletest_POPULATED)
            FetchContent_Populate(googletest)
            add_subdirectory(${googletest_SOURCE_DIR} ${googletest_BINARY_DIR})
        endif()
    endif()

    # Disable compiler warnings and clang-tidy
    if(DISABLE_EXTERNAL_WARNINGS)
        target_compile_options(gtest PRIVATE -w)
        set_target_properties(gtest PROPERTIES CXX_CLANG_TIDY "")

        target_compile_options(gtest_main PRIVATE -w)
        set_target_properties(gtest_main PROPERTIES CXX_CLANG_TIDY "")

        target_compile_options(gmock PRIVATE -w)
        set_target_properties(gmock PROPERTIES CXX_CLANG_TIDY "")

        target_compile_options(gmock_main PRIVATE -w)
        set_target_properties(gmock_main PROPERTIES CXX_CLANG_TIDY "")
    endif()

endfunction()

