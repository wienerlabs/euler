cmake_minimum_required(VERSION 3.16)
project(euler VERSION 0.1.0 LANGUAGES C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(EULER_BUILD_TESTS "Build unit tests" ON)
option(EULER_BUILD_SIM "Build for simulation (x86/x64)" ON)
option(EULER_BUILD_EMBEDDED "Build for embedded ARM target" OFF)
option(EULER_ENABLE_ASAN "Enable AddressSanitizer for debug builds" OFF)

# Compiler warnings - strict for safety-critical code
add_compile_options(
    -Wall -Wextra -Wpedantic -Werror
    -Wconversion -Wsign-conversion
    -Wdouble-promotion -Wformat=2
    -Wnull-dereference -Wuninitialized
    -Wstrict-prototypes -Wmissing-prototypes
    -fno-common
)

if(EULER_BUILD_EMBEDDED)
    set(CMAKE_SYSTEM_NAME Generic)
    set(CMAKE_SYSTEM_PROCESSOR arm)
    set(CMAKE_C_COMPILER arm-none-eabi-gcc)
    set(CMAKE_AR arm-none-eabi-ar)
    set(CMAKE_RANLIB arm-none-eabi-ranlib)
    
    add_compile_options(
        -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16
        -ffunction-sections -fdata-sections
        -Os
    )
    add_link_options(
        -Wl,--gc-sections
        -specs=nosys.specs
    )
    add_compile_definitions(EULER_EMBEDDED=1)
else()
    if(EULER_ENABLE_ASAN AND CMAKE_BUILD_TYPE STREQUAL "Debug")
        add_compile_options(-fsanitize=address -fno-omit-frame-pointer)
        add_link_options(-fsanitize=address)
    endif()
    add_compile_definitions(EULER_SIMULATION=1)
endif()

include_directories(${CMAKE_SOURCE_DIR}/include)

set(EULER_SOURCES
    src/euler_comm.c
    src/euler_control.c
    src/euler_mavlink.c
    src/euler_mission.c
)

add_library(euler STATIC ${EULER_SOURCES})
target_include_directories(euler PUBLIC ${CMAKE_SOURCE_DIR}/include)

if(NOT EULER_BUILD_EMBEDDED)
    target_link_libraries(euler PRIVATE pthread m)
endif()

if(EULER_BUILD_TESTS)
    enable_testing()
    
    add_executable(euler_tests
        tests/test_main.c
        tests/test_types.c
        tests/test_comm.c
        tests/test_control.c
    )
    target_link_libraries(euler_tests PRIVATE euler)
    
    if(NOT EULER_BUILD_EMBEDDED)
        target_link_libraries(euler_tests PRIVATE pthread m)
    endif()
    
    add_test(NAME euler_unit_tests COMMAND euler_tests)
endif()

add_custom_target(format
    COMMAND clang-format -i ${CMAKE_SOURCE_DIR}/src/*.c ${CMAKE_SOURCE_DIR}/include/*.h
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    COMMENT "Formatting source code"
)

add_custom_target(analyze
    COMMAND cppcheck --enable=all --std=c11 
            --suppress=missingIncludeSystem
            ${CMAKE_SOURCE_DIR}/src ${CMAKE_SOURCE_DIR}/include
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    COMMENT "Running static analysis"
)

