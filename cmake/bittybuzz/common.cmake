include(FetchContent)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR})
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} PARENT_SCOPE)

function(bittybuzz_fetch)
    FetchContent_Declare(
        bittybuzz
        GIT_REPOSITORY https://github.com/SwarmUS/BittyBuzz.git
        GIT_PROGRESS   TRUE
    )

    FetchContent_GetProperties(bittybuzz)
    if(NOT bittybuzz_POPULATED)
        FetchContent_Populate(bittybuzz)
        set(BITTYBUZZ_SRC_PATH ${bittybuzz_SOURCE_DIR} PARENT_SCOPE)
    endif()

endfunction()