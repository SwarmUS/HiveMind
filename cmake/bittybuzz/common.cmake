include(FetchContent)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR})
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} PARENT_SCOPE)

function(bittybuzz_fetch)
    FetchContent_Declare(
        bittybuzz
        GIT_REPOSITORY https://github.com/xgroleau/BittyBuzz
        GIT_TAG        6ec2c2bc55b981be6271043f8383f3df8493eb3f
        GIT_PROGRESS   TRUE
    ) 

    FetchContent_GetProperties(bittybuzz)
    if(NOT bittybuzz_POPULATED)
        message("Cloning BittyBuzz")
        set(FETCHCONTENT_QUIET FALSE) # To see progress
        FetchContent_Populate(bittybuzz)
        set(BITTYBUZZ_SRC_PATH ${bittybuzz_SOURCE_DIR} PARENT_SCOPE)
    endif()

endfunction()

# Generate an interface target with the included path for the string and bytecode generated file
# Need the target name
# The bzz source file
# A list of included directories
# A list of bst files 
function(bittybuzz_generate_bytecode _TARGET bzz_source bzz_include_list bzz_bst_list)
    find_package(Buzz)
    if(NOT BUZZ_FOUND)
        message(FATAL_ERROR "Buzz was not found")
    endif()

    get_filename_component(BZZ_BASENAME ${bzz_source} NAME_WE)
    set(BUZZ_INCLUDE_PATH "$ENV{BUZZ_INCLUDE_PATH}:${BUZZ_BZZ_INCLUDE_DIR}:${bzz_include_list}")

    # Settings vars
    set(BO_FILE   ${CMAKE_CURRENT_BINARY_DIR}/${BZZ_BASENAME}.bo)
    set(BDB_FILE  ${CMAKE_CURRENT_BINARY_DIR}/${BZZ_BASENAME}.bdb)    
    set(BASM_FILE ${CMAKE_CURRENT_BINARY_DIR}/${BZZ_BASENAME}.basm)
    set(BST_FILE  ${CMAKE_CURRENT_BINARY_DIR}/${BZZ_BASENAME}.bst)
    set(BHEADER_FILE  ${CMAKE_CURRENT_BINARY_DIR}/${BZZ_BASENAME}_bytecode.h)
    set(BHEADER_FILE_TMP ${BHEADER_FILE}.tmp)
    set(BHEADER_STRING_FILE  ${CMAKE_CURRENT_BINARY_DIR}/${BZZ_BASENAME}_string.h)
    set(BHEADER_STRING_FILE_TMP  ${BHEADER_STRING_FILE}.tmp)

    # Concatenating BST files we configure/copy so cmake reruns when bst are changed
    configure_file(${BITTYBUZZ_SRC_PATH}/src/bittybuzz/util/BittyBuzzStrings.bst ${BST_FILE} COPYONLY)
    foreach(BST ${bzz_bst_list})
        get_filename_component(BST_FILENAME ${BST} NAME)
        set(BST_FILENAME ${CMAKE_CURRENT_BINARY_DIR}/${_TARGET}-${BST_FILENAME})
        configure_file(${BST} ${BST_FILENAME} COPYONLY)
        file(READ ${BST_FILENAME} CONTENTS)
        file(APPEND ${BST_FILE} "${CONTENTS}")
    endforeach()

    # TODO: bzzparse return 0 if a file is not found during parsing, but the build still passes, this should be fixed in the buzz compiler
    # Parsing buzz file
    add_custom_target(${_TARGET}_bzz_parse
        COMMAND export BUZZ_INCLUDE_PATH=${BUZZ_INCLUDE_PATH} && ${BUZZ_PARSER} ${bzz_source} ${BASM_FILE} ${BST_FILE}
        DEPENDS ${BST_FILE})

    # Compiling buzz file
    add_custom_target(${_TARGET}_bzz_compile
        COMMAND ${BUZZ_ASSEMBLER} ${BASM_FILE} ${BO_FILE} ${BDB_FILE}
        DEPENDS ${_TARGET}_bzz_parse)


    # Cross compiling and verifying that the file changed to prevent recompiling
    add_custom_target(${_TARGET}_bzz_cross_compile
        COMMAND ${BBZ_zooids_bcodegen} ${BO_FILE} ${BHEADER_FILE_TMP}
        COMMAND cmp --silent ${BHEADER_FILE_TMP} ${BHEADER_FILE} || cp ${BHEADER_FILE_TMP} ${BHEADER_FILE}
        WORKING_DIRECTORY ${BBZ_BINARY_PATH}
        DEPENDS zooids_bcodegen bo2bbo ${_TARGET}_bzz_compile)

    # Creating string header file
    add_custom_target(${_TARGET}_bzz_string
        COMMAND ${PROJECT_SOURCE_DIR}/tools/extract_bzz_strings.sh ${BASM_FILE} ${BST_FILE} ${BHEADER_FILE} ${BHEADER_STRING_FILE_TMP}
        COMMAND cmp --silent ${BHEADER_STRING_FILE_TMP} ${BHEADER_STRING_FILE} || cp ${BHEADER_STRING_FILE_TMP} ${BHEADER_STRING_FILE}
        COMMAND cmp ${BHEADER_STRING_FILE_TMP} ${BHEADER_STRING_FILE}
        DEPENDS ${_TARGET}_bzz_cross_compile)

    # Create library with file
    add_library(${_TARGET} INTERFACE)
    target_include_directories(${_TARGET} INTERFACE ${CMAKE_CURRENT_BINARY_DIR})
    add_dependencies(${_TARGET} ${_TARGET}_bzz_cross_compile ${_TARGET}_bzz_string)

 endfunction()
