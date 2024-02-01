cmake_minimum_required(VERSION 3.10)

#set the project name
project(lc3_codec)
set (BASE_LIB_DIR_PATH          ${CMAKE_CURRENT_SOURCE_DIR}/../../../)
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


if(EXISTS "${GOOGLE_LC3_LIB_BIN}/liblc3.a")
	message("-- Build CE with Google LC3 CODEC")
else()
	if(EXISTS "${GOOGLE_LC3_LIB}")
		message("-- !!! Build Google LC3 Codec No Find Static Library !!!")
		message("-- !!! Follow the instruction below to build Google LC3 library first !!!")
		message("-- !!! cd COMPONENT_LC3_CODEC/google_lc3 ")
		message("-- !!! ./build_google_lc3.sh ")
	else()
		message("-- !!! No find Google LC3 Codec library source code !!!")
		message("-- !!! Follow README to get Google LC3 Codec !!!")
		message("-- !!! Google LC3 github repo https://github.com/google/liblc3 !!!")
		message("-- !!! Check Google LC3 library at " ${GOOGLE_LC3_LIB})
	endif()

endif()

find_library(
      LC3
      lc3 
      HINTS
      ${GOOGLE_LC3_LIB_BIN})

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/)
include_directories(${GOOGLE_LC3_LIB_INCLUDE})

add_library(
    ${PROJECT_NAME}
    STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/${LC3_FOLDER}/lc3_codec.c)

target_link_libraries(${PROJECT_NAME} PRIVATE ${LC3} m)
endif()


