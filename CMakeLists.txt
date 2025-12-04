cmake_minimum_required(VERSION 3.20)



####platform specific settings
# SET TOOLCHAIN BEFORE PROJECT - ADD THIS LINE
set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/platforms/stm32f4xx/cmake/gcc-arm-none-eabi.cmake)

add_subdirectory(platforms/stm32f4xx)

# Get platform libraries and interface
set(PLATFORM_LIB stm32_platform)
set(PLATFORM_INTERFACE stm32_interface)
####End platform specific settings

project(EcoDrive LANGUAGES C CXX ASM)

# Prevent in-source builds
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_BINARY_DIR)
    message(FATAL_ERROR "
    In-source builds are not allowed!
    Please use:
    mkdir build && cd build && cmake ..
    ")
endif()

# Setup cmake module path and compiler settings
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

message("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/build)

# Set C++ standard
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Control where files go WITHIN the build directory
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# Include directories
include_directories(
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/thirdparty/lwrb/lwrb
)

# Source files
file(GLOB_RECURSE SOURCES
    "application/*.cpp"
    "drivers/*.cpp"
    "drivers/*.c"
    "middleWare/*.cpp"
    "core/*.cpp"
    "core/*.c"
)

# Create executable
add_executable(${PROJECT_NAME} ${SOURCES})


# Link with platform
message(STATUS "LIBRARIES TO LINK: ${PLATFORM_LIB}")
target_link_libraries(EcoDrive PRIVATE 
                    ${PLATFORM_INTERFACE} 
                    ${PLATFORM_LIB}
                    ${TOOLCHAIN_LINK_LIBRARIES}

                    )

# ADD THIS: Show size automatically after every build
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E echo "=== Memory Usage ==="
    COMMAND ${CMAKE_SIZE} --format=berkeley $<TARGET_FILE:${PROJECT_NAME}>
    COMMENT "Displaying memory usage"
)

# Size target for checking memory usage
add_custom_target(size
    DEPENDS ${PROJECT_NAME}
    COMMAND ${CMAKE_COMMAND} -E echo "=== Detailed Memory Analysis ==="
    COMMAND ${CMAKE_SIZE} --format=berkeley $<TARGET_FILE:${PROJECT_NAME}>
)

# Convert output to hex and binary
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${PROJECT_NAME}> ${PROJECT_NAME}.hex
)
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${PROJECT_NAME}> ${PROJECT_NAME}.bin
)