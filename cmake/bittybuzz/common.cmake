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

function(bittybuzz_generate_bytecode _TARGET bzz_source bzz_includes)
    get_filename_component(BZZ_BASENAME ${bzz_source} NAME_WE)
    set(BZZ_BASEPATH "${CMAKE_CURRENT_BINARY_DIR}/${BZZ_BASENAME}")

    set(BO_FILE   ${BZZ_BASEPATH}.bo)
    set(BDB_FILE  ${BZZ_BASEPATH}.bdb)    
    set(BASM_FILE ${BZZ_BASEPATH}.basm)
    set(BHEADER_FILE  ${BZZ_BASEPATH}_bytecode.h)    


    # Parsing buzz file
    add_custom_target(${_TARGET}_bzz_parse
            COMMAND ${BZZPAR} ${bzz_source} ${BASM_FILE} ${BITTYBUZZ_SRC_PATH}/src/bittybuzz/util/BittyBuzzStrings.bst)

    # Compiling buzz file
    add_custom_target(${_TARGET}_bzz_compile
      COMMAND ${BZZASM} ${BASM_FILE} ${BO_FILE} ${BDB_FILE}
      DEPENDS ${_TARGET}_bzz_parse)

    # Cross compiling
    add_custom_target(${_TARGET}_bzz_cross_compile
      COMMAND zooids_bcodegen ${BO_FILE} ${BHEADER_FILE}
      DEPENDS zooids_bcodegen bo2bbo ${_TARGET}_bzz_compile)

    # Create library with file
    add_library(${_TARGET} INTERFACE)
    target_include_directories(${_TARGET} INTERFACE ${CMAKE_CURRENT_BINARY_DIR})
    add_dependencies(${_TARGET} ${_TARGET}_bzz_cross_compile)

 endfunction()
