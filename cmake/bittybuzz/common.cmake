include(FetchContent)

function(bittybuzz_fetch)
    FetchContent_Declare(
    bittybuzz
    GIT_REPOSITORY https://github.com/SwarmUS/BittyBuzz.git
    )

    FetchContent_GetProperties(bittybuzz)
    if(NOT bittybuzz_POPULATED)
        FetchContent_Populate(bittybuzz)
        set(BITTYBUZZ_SRC_PATH ${bittybuzz_SOURCE_DIR} PARENT_SCOPE)
    endif()

endfunction()
