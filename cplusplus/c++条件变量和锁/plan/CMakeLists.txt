cmake_minimum_required(VERSION 3.10)
project(mv_plan)

set(CMAKE_CXX_FLAGS "-O3 -fpermissive")

include_directories(
    ${MVINC_ROOT}
    ${MVALG_ROOT}/psd/ApaAlgCommonLib
    ${MVALG_ROOT}/psd/freespace
    ${MVALG_ROOT}/psd/libRPNcalib/include
    ${MVALG_ROOT}/psd/detect_slot
    ${PROJECT_SOURCE_DIR}/src
    ${PROJECT_SOURCE_DIR}/src/lib
    ${PROJECT_SOURCE_DIR}/src/base
    ${PROJECT_SOURCE_DIR}/src/mv_plan
    ${PROJECT_SOURCE_DIR}/src/common
    ${PROJECT_SOURCE_DIR}/src/OpenSpacePlanning
    ${PROJECT_SOURCE_DIR}/src/pps_plan
    ${PROJECT_SOURCE_DIR}/src/PnCInteract
    ${MVHAL_ROOT}/pps_ipc/src/pps_com
    ${MVHAL_ROOT}/pps_ipc/src/motovis_inc
    ${MVHAL_ROOT}/pps_ipc/src/gac_inc
    )

link_directories(
    ${INSTALL_ROOT}/lib
    ${MVHAL_ROOT}/pps_ipc/src/common_utils/service_sdk/lib
    )

file(GLOB_RECURSE SRC_FILES ${PROJECT_SOURCE_DIR} *.cpp)
list(FILTER SRC_FILES EXCLUDE REGEX "input_manager")

add_executable(${PROJECT_NAME} ${SRC_FILES})
target_link_libraries(${PROJECT_NAME}
    pps_com
    pps_ipc
    vx_framework
    vx_platform_psdk_j7_qnx
    app_utils_console_io
    gaclog
    )

install(TARGETS ${PROJECT_NAME} DESTINATION ${CMAKE_SOURCE_DIR}/install/bin)
