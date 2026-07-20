cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

# Define the build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME base)

# ThreadX configuration for Cortex-M3 / GCC
set(THREADX_ARCH cortex_m3)

# 机器人 & 板型 配置
set(ROBOT f103)
set(BOARD single)

# 加载配置
include(${CMAKE_SOURCE_DIR}/../../apps/config.cmake)

# 生成头文件到 build 目录
set(_generated_dir ${CMAKE_CURRENT_BINARY_DIR}/generated)
include(${CMAKE_SOURCE_DIR}/../../apps/generate_headers.cmake)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Add ThreadX RTOS
add_subdirectory(../../threadx threadx)

# Add shared project libraries
add_subdirectory(../../utils ${CMAKE_CURRENT_BINARY_DIR}/utils)
add_subdirectory(../../board/bsp ${CMAKE_CURRENT_BINARY_DIR}/bsp)
add_subdirectory(../../robot ${CMAKE_CURRENT_BINARY_DIR}/robot)
add_subdirectory(../../modules ${CMAKE_CURRENT_BINARY_DIR}/modules)
add_subdirectory(../../apps ${CMAKE_CURRENT_BINARY_DIR}/apps)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${_generated_dir}
    # Add user defined include paths
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    azrtos::threadx
    utils
    bsp
    robot
    modules
    app
    # Add user defined libraries
)
