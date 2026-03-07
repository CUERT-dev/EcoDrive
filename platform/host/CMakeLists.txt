cmake_minimum_required(VERSION 3.20)

# Host platform module used by top-level CMake via add_subdirectory(platform/host).

set(DEVICE_DEFINITIONS
    USE_FULL_ASSERT
)

set(FREERTOS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/FreeRTOS/Source)
set(CMSIS_DSP_DIR ${CMAKE_SOURCE_DIR}/platform/host/CMSIS-DSP)
set(CMSISCORE ${CMAKE_SOURCE_DIR}/platform/stm32f4/Drivers/CMSIS)

set(HOST_COMMON_INCLUDES
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${FREERTOS_DIR}/include
    ${FREERTOS_DIR}/portable/ThirdParty/GCC/Posix
    ${FREERTOS_DIR}/portable/ThirdParty/GCC/Posix/utils
)

set(FreeRTOS_Src
    ${FREERTOS_DIR}/croutine.c
    ${FREERTOS_DIR}/event_groups.c
    ${FREERTOS_DIR}/list.c
    ${FREERTOS_DIR}/queue.c
    ${FREERTOS_DIR}/stream_buffer.c
    ${FREERTOS_DIR}/tasks.c
    ${FREERTOS_DIR}/timers.c
    ${FREERTOS_DIR}/portable/MemMang/heap_4.c
    ${FREERTOS_DIR}/portable/ThirdParty/GCC/Posix/port.c
    ${FREERTOS_DIR}/portable/ThirdParty/GCC/Posix/utils/wait_for_event.c
)

set(Platform_Src
    ${CMAKE_CURRENT_SOURCE_DIR}/platform.c
)

file(GLOB_RECURSE Driver_Core_Src
    ${CMAKE_CURRENT_SOURCE_DIR}/eldriver/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/eldriver/*.c
)

find_package(Threads REQUIRED)
add_subdirectory(${CMSIS_DSP_DIR}/Source ${CMAKE_CURRENT_BINARY_DIR}/cmsis_dsp_host)

add_library(host_platform STATIC)
target_sources(host_platform PRIVATE
    ${FreeRTOS_Src}
)
target_include_directories(host_platform PUBLIC ${HOST_COMMON_INCLUDES})
target_compile_definitions(host_platform PUBLIC
    ${DEVICE_DEFINITIONS}
    projCOVERAGE_TEST=0
    projENABLE_TRACING=0
)
target_link_libraries(host_platform PUBLIC Threads::Threads CMSISDSP)

add_library(host_platform_interface INTERFACE)
target_link_libraries(host_platform_interface INTERFACE
    host_platform
    Threads::Threads
    CMSISDSP
)
target_include_directories(host_platform_interface INTERFACE
    ${HOST_COMMON_INCLUDES}
    ${CMSIS_DSP_DIR}/Include
    ${CMSISCORE}/Include
)
target_compile_definitions(host_platform_interface INTERFACE
    ${DEVICE_DEFINITIONS}
    projCOVERAGE_TEST=0
    projENABLE_TRACING=0
)

# Export variables expected by the parent CMakeLists.txt.
set(PLATFORM_LIB host_platform PARENT_SCOPE)
set(PLATFORM_INTERFACE host_platform_interface PARENT_SCOPE)
set(PLATFORM_INCLUDES ${HOST_COMMON_INCLUDES} ${CMSIS_DSP_DIR}/Include ${CMSISCORE}/Include PARENT_SCOPE)
set(PLATFORM_DEFINITIONS ${DEVICE_DEFINITIONS} PARENT_SCOPE)
set(PLATFORM_SOURCE ${Driver_Core_Src} ${Platform_Src} PARENT_SCOPE)
