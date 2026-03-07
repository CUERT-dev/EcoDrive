cmake_minimum_required(VERSION 3.15)

project(freertos_smoke C)

set(FREERTOS_KERNEL_PATH "${CMAKE_CURRENT_LIST_DIR}/../FreeRTOS/Source")

add_library(freertos_config INTERFACE)
target_include_directories(freertos_config
    INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}
)
target_compile_definitions(freertos_config
    INTERFACE
        projCOVERAGE_TEST=0
        projENABLE_TRACING=0
)

# Use the kernel's POSIX simulator port and heap_4.
set(FREERTOS_HEAP "4" CACHE STRING "" FORCE)
set(FREERTOS_PORT "GCC_POSIX" CACHE STRING "" FORCE)

add_subdirectory(${FREERTOS_KERNEL_PATH} ${CMAKE_CURRENT_BINARY_DIR}/FreeRTOS-Kernel)

add_executable(freertos_smoke
    main.c
)

target_link_libraries(freertos_smoke
    PRIVATE
        freertos_kernel
        freertos_config
)
