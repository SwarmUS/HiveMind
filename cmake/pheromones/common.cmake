include(FetchContent)

# Adding modules
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR})
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH})

function(pheromones_fetch_populate)
    if(PHEROMONES_PATH)
        message(INFO "PHEROMONES_PATH specified, skipping fetch")
    endif()

    FetchContent_Declare(
        ${PROJECT_NAME}_pheromones

        GIT_REPOSITORY https://github.com/SwarmUS/Pheromones
        GIT_TAG        c93b2ac
        GIT_PROGRESS   TRUE
    )

    set(PHEROMONES ${PROJECT_NAME}_pheromones)
    string(TOLOWER ${PHEROMONES} PHEROMONES_L)

    FetchContent_GetProperties(${PHEROMONES} POPULATED PHEROMONES_POPULATED)
    if(NOT PHEROMONES_POPULATED)
        message("Cloning pheromones library")
        set(FETCHCONTENT_QUIET FALSE) # To see progress
        FetchContent_Populate(${PHEROMONES})
        add_subdirectory(${${PHEROMONES_L}_SOURCE_DIR} ${${PHEROMONES_L}_BINARY_DIR})
    endif()

    # Removing warnings
    if (DISABLE_EXTERNAL_WARNINGS) 
        set(LIB_LIST
          example
          protobuf-nanopb-static
          swarmus-pheromones-example
          swarmus-pheromones-hivemind-host)

        foreach(LIB ${LIB_LIST})
            target_compile_options(${LIB} PRIVATE -w)
            set_target_properties(${LIB} PROPERTIES C_CLANG_TIDY "")
            set_target_properties(${LIB} PROPERTIES CXX_CLANG_TIDY "")
        endforeach()
    endif()
endfunction()
