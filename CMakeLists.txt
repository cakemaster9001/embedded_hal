cmake_minimum_required(VERSION 3.5)
project(hal_registers VERSION 0.1.0 LANGUAGES CXX)


set (CMAKE_CXX_STANDARD 23)

set (SRCS tests/enum_test.cpp tests/gpio_test.cpp)

add_executable(hal_registers main.cpp ${SRCS} )
target_include_directories(hal_registers PRIVATE . enum tests peripherals concepts_lib bits)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})

