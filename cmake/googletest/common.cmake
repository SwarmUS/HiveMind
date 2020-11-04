include(FetchContent)


FetchContent_GetProperties(googletest)
if(NOT googletest_POPULATED)
  FetchContent_Populate(googletest)
  add_subdirectory(${googletest_SOURCE_DIR} ${googletest_BINARY_DIR})
endif()

function(googletest_fetch)
    FetchContent_Declare(
        GoogleTest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG        release-1.10.0
        GIT_PROGRESS   TRUE
    )

    set(GOOGLE_TEST GoogleTest)
    string(TOLOWER ${GOOGLE_TEST} GOOGLE_TEST_L)

    if(GOOGLE_TEST_PATH)
        message(INFO "GOOGLE_TEST_PATH specified, skipping fetch")
    endif()


    FetchContent_GetProperties(GOOGLE_TEST POPULATED GOOGLE_TEST_POPULATED)
    if(NOT GOOGLE_TEST_POPULATED)
        message("Cloning GoogleTest")
        set(FETCHCONTENT_QUIET FALSE) # To see progress
        FetchContent_Populate(${GOOGLE_TEST})
    endif()

    set(GOOGLE_TEST_PATH ${${GOOGLE_TEST_L}_SOURCE_DIR} PARENT_SCOPE)
endfunction()
