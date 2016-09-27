project(serial)
cmake_minimum_required(VERSION 3.1)
add_library(serial serial.cpp)
target_include_directories(serial PUBLIC .)
