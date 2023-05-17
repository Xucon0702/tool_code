cmake_minimum_required(VERSION 3.10)

project(targetSlot)

include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/include/base
    ${PROJECT_SOURCE_DIR}/include/common
    ${PROJECT_SOURCE_DIR}/include/log
    ${PROJECT_SOURCE_DIR}/include/alg/psd
    ${PROJECT_SOURCE_DIR}/include/hmi
)

file(GLOB SOURCE_FILES  
    ${PROJECT_SOURCE_DIR}/src/*.c
    ${PROJECT_SOURCE_DIR}/src/*.cpp
    ${PROJECT_SOURCE_DIR}/src/hmi/*.c
    ${PROJECT_SOURCE_DIR}/src/hmi/*.cpp

    ${PROJECT_SOURCE_DIR}/include/base/*.c
    ${PROJECT_SOURCE_DIR}/include/base/*.cpp
)

add_executable(${PROJECT_NAME} ${SOURCE_FILES})

install(TARGETS ${PROJECT_NAME} DESTINATION  ${PROJECT_SOURCE_DIR}/bin)
