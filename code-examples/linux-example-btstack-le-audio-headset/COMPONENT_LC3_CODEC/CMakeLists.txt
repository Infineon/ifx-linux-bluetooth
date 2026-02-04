cmake_minimum_required(VERSION 3.10)

#set the project name
project(lc3_codec)

set (BTSTACK_INCLUDE            ${BASE_LIB_DIR_PATH}/btstack/wiced_include)
set (PORTING_LAYER_INCLUDE      ${BASE_LIB_DIR_PATH}/bluetooth-linux)

include_directories(${BTSTACK_INCLUDE})
include_directories(${PORTING_LAYER_INCLUDE})

message("-- Detecting LC3 CODEC Static Library")
if(ITTIAMLC3)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${LC3_FOLDER}/libia_lc3_enc_dec_lib.a")
	message("-- Build CE with ITTIAM LC3 CODEC")
else()
	message("-- !!! Build ITTIAM LC3 CODEC, No Find static library, Please use GOOGLE LC3 CODEC !!!")
endif()

link_directories(${CMAKE_CURRENT_SOURCE_DIR}/${LC3_FOLDER}/)

add_library(
    ${PROJECT_NAME}
    STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/${LC3_FOLDER}/ialc3.c)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/)

target_link_libraries(${PROJECT_NAME} PUBLIC ia_lc3_enc_dec_lib)

elseif(GOOGLELC3)

set (GOOGLE_LC3_LIB             ${BASE_LIB_DIR_PATH}/liblc3)
set (GOOGLE_LC3_LIB_INCLUDE     ${GOOGLE_LC3_LIB}/include)
set (GOOGLE_LC3_LIB_BIN         ${GOOGLE_LC3_LIB}/bin)

include(ExternalProject)

ExternalProject_Add(liblc3_project
    SOURCE_DIR        "${GOOGLE_LC3_LIB}"
    CONFIGURE_COMMAND make clean -C "${GOOGLE_LC3_LIB}"
    BUILD_COMMAND     make CC="${CMAKE_C_COMPILER}" -C "${GOOGLE_LC3_LIB}"
    INSTALL_COMMAND   ""
    BUILD_IN_SOURCE   1
)

find_library(
      LC3
      lc3 
      HINTS
      ${GOOGLE_LC3_LIB_BIN})

# Fallback to default path if this is the first time liblc3 is built
if (NOT LC3)
    set(LC3 ${GOOGLE_LC3_LIB_BIN}/liblc3.a)
endif()

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/)
include_directories(${GOOGLE_LC3_LIB_INCLUDE})

add_library(
    ${PROJECT_NAME}
    STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/${LC3_FOLDER}/lc3_codec.c)

target_link_libraries(${PROJECT_NAME} PRIVATE ${LC3} m)
add_dependencies(${PROJECT_NAME} liblc3_project)
endif()
