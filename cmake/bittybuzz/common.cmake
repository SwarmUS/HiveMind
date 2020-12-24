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

    # Compiling buzz files
    add_custom_command(OUTPUT ${BO_FILE} ${BDB_FILE} ${BASM_FILE}
            COMMAND ${BZZC} -I ${bzz_includes} -b ${BO_FILE} -d ${BDB_FILE} -a ${BASM_FILE} ${bzz_source})

    # Cross compiling bzz object to bbz object
    add_custom_command(OUTPUT ${BHEADER_FILE}
      COMMAND zooids_bcodegen ${BO_FILE} ${BHEADER_FILE}
      DEPENDS zooids_bcodegen bo2bbo ${BO_FILE})

    add_library(${_TARGET} INTERFACE)
    target_include_directories(${_TARGET} INTERFACE ${CMAKE_CURRENT_BINARY_DIR})
    add_dependencies(${_TARGET} ${BHEADER_FILE})

 endfunction()
