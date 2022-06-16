cmake_minimum_required(VERSION 3.10)

# set the project name
project(CT_EXAM)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# add the executable
add_executable(main main.c)

target_include_directories(main PUBLIC
                            "Device/Include/"
                            "HAL/Include/"
                            )