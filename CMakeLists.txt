cmake_minimum_required(VERSION 3.15)
project(mavproxy_adapter C)

set(CMAKE_C_STANDARD 99)
include_directories(/home/rmasl/Desktop/Evan_Personal_Use/SeniorD/mavproxy_adapter_base/c_library_v2)

add_executable(mavproxy_adapter main.c)
add_executable(client client.c)