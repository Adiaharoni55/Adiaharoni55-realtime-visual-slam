# Oldest CMake version this file works with (3.25 is needed for presets later)
cmake_minimum_required(VERSION 3.25)

# Project name, version, and language (CXX = C++)
project(rtvm VERSION 0.0.1 LANGUAGES CXX)

# Use C++17 for the whole project, and fail if the compiler can't do it
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)   # plain standard C++, no compiler-specific extras

# Write compile_commands.json so VS Code understands the project
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ---- Library: the reusable SLAM code ----
add_library(rtvm_core
    src/version.cpp
)

# Anyone using rtvm_core can #include "rtvm/..." from the include/ folder
target_include_directories(rtvm_core PUBLIC
    ${PROJECT_SOURCE_DIR}/include
)

# ---- App: the rtvm command-line program ----
add_executable(rtvm
    apps/main.cpp
)

# The app uses the library (and inherits its PUBLIC include folder)
target_link_libraries(rtvm PRIVATE rtvm_core)

# ---- Compiler warnings ----
# A small "interface" target that holds only settings, no code.
# Any target that links it gets these warning flags.
add_library(rtvm_warnings INTERFACE)
target_compile_options(rtvm_warnings INTERFACE
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
    $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -Wpedantic>
)

target_link_libraries(rtvm_core PRIVATE rtvm_warnings)
target_link_libraries(rtvm      PRIVATE rtvm_warnings)