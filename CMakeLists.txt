cmake_minimum_required(VERSION 3.20)
set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/build)  # ← THAT'S IT
# Setup cmake module path and compiler settings
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

message("Build type: "              ${CMAKE_BUILD_TYPE})

set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/build)

# Set C++ standard
set(CMAKE_C_STANDARD                11)
set(CMAKE_C_STANDARD_REQUIRED       ON)
set(CMAKE_C_EXTENSIONS              ON)
set(CMAKE_CXX_STANDARD              17)
set(CMAKE_CXX_STANDARD_REQUIRED     ON)
set(CMAKE_CXX_EXTENSIONS            ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS   ON)

set(CMAKE_TOOLCHAIN_FILE        ${CMAKE_SOURCE_DIR}/cmake/gcc-arm-none-eabi.cmake)

project(EcoDrive LANGUAGES C CXX ASM)

set(linker_script_SRC "${CMAKE_SOURCE_DIR}/linker_scripts/STM32F401XX_FLASH.ld")

include(Stm32F401xx)
# Include directories
include_directories(
    ${CMAKE_SOURCE_DIR}/application
    ${CMAKE_SOURCE_DIR}/drivers
    ${CMAKE_SOURCE_DIR}/middleWare
    ${CMAKE_SOURCE_DIR}/core
    ${CMAKE_SOURCE_DIR}/thirdParty
)

# Source files
file(GLOB_RECURSE SOURCES
    "application/*.cpp"
    "drivers/*.cpp"
    "drivers/*.c"
    "middleWare/*.cpp"
    "core/*.cpp"
)


message(STATUS "CMAKE_C_COMPILER: ${CMAKE_C_COMPILER}")
message(STATUS "CMAKE_C_COMPILER_ID: ${CMAKE_C_COMPILER_ID}")
message(STATUS "CMAKE_C_COMPILER_VERSION: ${CMAKE_C_COMPILER_VERSION}")

message(STATUS "CMAKE_CXX_COMPILER: ${CMAKE_CXX_COMPILER}")
message(STATUS "CMAKE_CXX_COMPILER_ID: ${CMAKE_CXX_COMPILER_ID}")
message(STATUS "CMAKE_CXX_COMPILER_VERSION: ${CMAKE_CXX_COMPILER_VERSION}")

message(STATUS "CMAKE_SYSTEM_NAME: ${CMAKE_SYSTEM_NAME}")
message(STATUS "CMAKE_SYSTEM_PROCESSOR: ${CMAKE_SYSTEM_PROCESSOR}")

# Create executable
add_executable(${PROJECT_NAME} ${SOURCES})

# Linker flags
target_link_options(${PROJECT_NAME} PRIVATE
	-T${linker_script_SRC}
	${CPU_PARAMETERS}
	-Wl,-Map=${CMAKE_PROJECT_NAME}.map
    --specs=nosys.specs
    #-u _printf_float
    #-u _scanf_float
    -Wl,--start-group
	-lc
	-lm
	-lstdc++
	-lsupc++
	-Wl,--end-group
	-Wl,--print-memory-usage
)


target_compile_options(${PROJECT_NAME} PRIVATE
	${CPU_PARAMETERS}
	-Wall
	-Wpedantic
	-Wno-unused-parameter
)


# Size target for checking memory usage
add_custom_target(size
    DEPENDS ${PROJECT_NAME}
    COMMAND ${CMAKE_SIZE} --format=berkeley $<TARGET_FILE:${PROJECT_NAME}>
)

# Convert output to hex and binary
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
	COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${PROJECT_NAME}> ${PROJECT_NAME}.hex
)
# Convert to bin file -> add conditional check?
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
	COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${PROJECT_NAME}> ${PROJECT_NAME}.bin
)