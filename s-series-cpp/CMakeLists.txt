cmake_minimum_required(VERSION 3.16)
project(SSERIAL_Samples LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# 仓库源码和 SDK 头文件统一使用 UTF-8；避免 MSVC 按系统代码页解析中文。
if(MSVC)
    add_compile_options(/utf-8)
endif()

# HR_Pro.h 与案例源码位于同一个目录。
set(HUAYAN_SAMPLES_DIR "${CMAKE_SOURCE_DIR}/samples")
set(HUAYAN_LIB_DIR "${CMAKE_SOURCE_DIR}/lib")

add_library(HuayanRobot::HR_Pro SHARED IMPORTED GLOBAL)

if(WIN32 AND MSVC)
    # MSVC: Release 使用 HR_Pro，Debug 使用 HR_Prod。
    set_target_properties(HuayanRobot::HR_Pro PROPERTIES
        IMPORTED_CONFIGURATIONS "DEBUG;RELEASE"
        IMPORTED_LOCATION_DEBUG "${HUAYAN_LIB_DIR}/HR_Prod.dll"
        IMPORTED_IMPLIB_DEBUG "${HUAYAN_LIB_DIR}/HR_Prod.lib"
        IMPORTED_LOCATION_RELEASE "${HUAYAN_LIB_DIR}/HR_Pro.dll"
        IMPORTED_IMPLIB_RELEASE "${HUAYAN_LIB_DIR}/HR_Pro.lib"
        MAP_IMPORTED_CONFIG_RELWITHDEBINFO RELEASE
        MAP_IMPORTED_CONFIG_MINSIZEREL RELEASE
    )
elseif(WIN32 AND MINGW)
    # MinGW: Release 使用 libHR_Pro，Debug 使用 libHR_Prod。
    set_target_properties(HuayanRobot::HR_Pro PROPERTIES
        IMPORTED_CONFIGURATIONS "DEBUG;RELEASE"
        IMPORTED_LOCATION_DEBUG "${HUAYAN_LIB_DIR}/libHR_Prod.dll"
        IMPORTED_IMPLIB_DEBUG "${HUAYAN_LIB_DIR}/libHR_Prod.dll.a"
        IMPORTED_LOCATION_RELEASE "${HUAYAN_LIB_DIR}/libHR_Pro.dll"
        IMPORTED_IMPLIB_RELEASE "${HUAYAN_LIB_DIR}/libHR_Pro.dll.a"
        MAP_IMPORTED_CONFIG_RELWITHDEBINFO RELEASE
        MAP_IMPORTED_CONFIG_MINSIZEREL RELEASE
    )
elseif(UNIX AND NOT APPLE)
    # Linux 只有一个 so，Debug 和 Release 均链接同一文件。
    set_target_properties(HuayanRobot::HR_Pro PROPERTIES
        IMPORTED_LOCATION "${HUAYAN_LIB_DIR}/libHR_Pro.so"
    )
else()
    message(FATAL_ERROR "Unsupported platform or compiler")
endif()

function(copy_huayan_runtime target_name)
    add_custom_command(TARGET ${target_name} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
                $<TARGET_FILE:HuayanRobot::HR_Pro>
                $<TARGET_FILE_DIR:${target_name}>
        COMMENT "Copying HR_Pro runtime library for ${target_name}"
    )
    if(WIN32 AND MSVC AND EXISTS "${HUAYAN_LIB_DIR}/pthreadVC2.dll")
        add_custom_command(TARGET ${target_name} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    "${HUAYAN_LIB_DIR}/pthreadVC2.dll"
                    $<TARGET_FILE_DIR:${target_name}>
        )
    endif()
endfunction()

# 案例目标
# --- showcase_stateMachineManage ---
add_executable(showcase_stateMachineManage samples/showcase_stateMachineManage.cpp)
target_link_libraries(showcase_stateMachineManage PRIVATE HuayanRobot::HR_Pro)
copy_huayan_runtime(showcase_stateMachineManage)
# --- showcase_constantforce ---
add_executable(showcase_constantforce samples/showcase_constantforce.cpp)
target_link_libraries(showcase_constantforce PRIVATE HuayanRobot::HR_Pro)
copy_huayan_runtime(showcase_constantforce)
# --- showcase_freedrive ---
add_executable(showcase_freedrive samples/showcase_freedrive.cpp)
target_link_libraries(showcase_freedrive PRIVATE HuayanRobot::HR_Pro)
copy_huayan_runtime(showcase_freedrive)
# --- showcase_robotErrorReset ---
add_executable(showcase_robotErrorReset samples/showcase_robotErrorReset.cpp)
target_link_libraries(showcase_robotErrorReset PRIVATE HuayanRobot::HR_Pro)
copy_huayan_runtime(showcase_robotErrorReset)
# --- showcase_servoJ ---
add_executable(showcase_servoJ samples/showcase_servoJ.cpp)
target_link_libraries(showcase_servoJ PRIVATE HuayanRobot::HR_Pro)
copy_huayan_runtime(showcase_servoJ)
# --- showcase_servoP ---
add_executable(showcase_servoP samples/showcase_servoP.cpp)
target_link_libraries(showcase_servoP PRIVATE HuayanRobot::HR_Pro)
copy_huayan_runtime(showcase_servoP)
# --- showcase_servoJPro ---
add_executable(showcase_servoJPro samples/showcase_servoJPro.cpp)
target_link_libraries(showcase_servoJPro PRIVATE HuayanRobot::HR_Pro)
copy_huayan_runtime(showcase_servoJPro)
# --- showcase_servoPPro ---
add_executable(showcase_servoPPro samples/showcase_servoPPro.cpp)
target_link_libraries(showcase_servoPPro PRIVATE HuayanRobot::HR_Pro)
copy_huayan_runtime(showcase_servoPPro)
# --- showcase_speedJ ---
add_executable(showcase_speedJ samples/showcase_speedJ.cpp)
target_link_libraries(showcase_speedJ PRIVATE HuayanRobot::HR_Pro)
copy_huayan_runtime(showcase_speedJ)
# --- showcase_speedL ---
add_executable(showcase_speedL samples/showcase_speedL.cpp)
target_link_libraries(showcase_speedL PRIVATE HuayanRobot::HR_Pro)
copy_huayan_runtime(showcase_speedL)
# --- showcase_motion ---
add_executable(showcase_motion samples/showcase_motion.cpp)
target_link_libraries(showcase_motion PRIVATE HuayanRobot::HR_Pro)
copy_huayan_runtime(showcase_motion)
# --- showcase_motionrel ---
add_executable(showcase_motionrel samples/showcase_motionrel.cpp)
target_link_libraries(showcase_motionrel PRIVATE HuayanRobot::HR_Pro)
copy_huayan_runtime(showcase_motionrel)
# --- showcase_jog ---
add_executable(showcase_jog samples/showcase_jog.cpp)
target_link_libraries(showcase_jog PRIVATE HuayanRobot::HR_Pro)
copy_huayan_runtime(showcase_jog)
# --- showcase_TrajectoryPathJ ---
add_executable(showcase_TrajectoryPathJ samples/showcase_TrajectoryPathJ.cpp)
target_link_libraries(showcase_TrajectoryPathJ PRIVATE HuayanRobot::HR_Pro)
copy_huayan_runtime(showcase_TrajectoryPathJ)
# --- showcase_TrajectoryPathL ---
add_executable(showcase_TrajectoryPathL samples/showcase_TrajectoryPathL.cpp)
target_link_libraries(showcase_TrajectoryPathL PRIVATE HuayanRobot::HR_Pro)
copy_huayan_runtime(showcase_TrajectoryPathL)
# --- showcase_TrajectoryPathManage ---
add_executable(showcase_TrajectoryPathManage samples/showcase_TrajectoryPathManage.cpp)
target_link_libraries(showcase_TrajectoryPathManage PRIVATE HuayanRobot::HR_Pro)
copy_huayan_runtime(showcase_TrajectoryPathManage)
# --- showcase_MovePathJOL ---
add_executable(showcase_MovePathJOL samples/showcase_MovePathJOL.cpp)
target_link_libraries(showcase_MovePathJOL PRIVATE HuayanRobot::HR_Pro)
copy_huayan_runtime(showcase_MovePathJOL)
# --- showcase_motionStatusManagement ---
add_executable(showcase_motionStatusManagement samples/showcase_motionStatusManagement.cpp)
target_link_libraries(showcase_motionStatusManagement PRIVATE HuayanRobot::HR_Pro)
copy_huayan_runtime(showcase_motionStatusManagement)

# 所有案例都可以直接 include "HR_Pro.h"。
get_property(HUAYAN_TARGETS DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}" PROPERTY BUILDSYSTEM_TARGETS)
foreach(target_name IN LISTS HUAYAN_TARGETS)
    if(NOT target_name STREQUAL "HuayanRobot::HR_Pro")
        target_include_directories(${target_name} PRIVATE "${HUAYAN_SAMPLES_DIR}")
    endif()
endforeach()
