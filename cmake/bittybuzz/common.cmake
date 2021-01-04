include(FetchContent)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR})
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} PARENT_SCOPE)

function(bittybuzz_fetch)
    FetchContent_Declare(
        bittybuzz
        GIT_REPOSITORY https://github.com/MISTLab/BittyBuzz.git
        GIT_TAG        beacf622757f1439c4adfe012005a82c9e2bde5d # Fixed commit since BittyBuzz doesn't have tags
        GIT_PROGRESS   TRUE
    ) 

    FetchContent_GetProperties(bittybuzz)
    if(NOT bittybuzz_POPULATED)
        message("Cloning BittyBuzz")
        FetchContent_Populate(bittybuzz)
        set(BITTYBUZZ_SRC_PATH ${bittybuzz_SOURCE_DIR} PARENT_SCOPE)
    endif()

endfunction()

function(bittybuzz_generate_bytecode _TARGET bzz_source bzz_includes)
    get_filename_component(BZZ_BASENAME ${bzz_source} NAME_WE)

    set(BO_FILE   ${BZZ_BASEPATH}.bo)
    set(BDB_FILE  ${BZZ_BASEPATH}.bdb)    
    set(BASM_FILE ${BZZ_BASEPATH}.basm)
    set(BHEADER_FILE  ${BZZ_BASEPATH}_bytecode.h)
    set(BHEADER_FILE_TMP ${BHEADER_FILE}.tmp)


    # Parsing buzz file
    add_custom_target(${_TARGET}_bzz_parse
            COMMAND ${BZZPAR} ${bzz_source} ${BASM_FILE} ${BITTYBUZZ_SRC_PATH}/src/bittybuzz/util/BittyBuzzStrings.bst)

    # Compiling buzz file
    add_custom_target(${_TARGET}_bzz_compile
      COMMAND ${BZZASM} ${BASM_FILE} ${BO_FILE} ${BDB_FILE}
      DEPENDS ${_TARGET}_bzz_parse)


    # Cross compiling and verifying that the file changed to prevent recompiling
    add_custom_target(${_TARGET}_bzz_cross_compile
      COMMAND ${BBZ_zooids_bcodegen} ${BO_FILE} ${BHEADER_FILE_TMP}
      COMMAND cmp --silent ${BHEADER_FILE_TMP} ${BHEADER_FILE} || cp ${BHEADER_FILE_TMP} ${BHEADER_FILE} 
      WORKING_DIRECTORY ${BBZ_BINARY_PATH}
      DEPENDS zooids_bcodegen bo2bbo ${_TARGET}_bzz_compile)

    # Create library with file
    add_library(${_TARGET} INTERFACE)
    target_include_directories(${_TARGET} INTERFACE ${CMAKE_CURRENT_BINARY_DIR})
    add_dependencies(${_TARGET} ${_TARGET}_bzz_cross_compile)

 endfunction()
