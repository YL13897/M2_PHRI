cmake_minimum_required(VERSION 3.3)
project(libFLNL VERSION 1.0.0 DESCRIPTION "Fast Light Network Library")

add_library(libFLNL STATIC  src/baseSocket.cpp
                            src/client.cpp
                            src/server.cpp
)

target_include_directories(libFLNL PRIVATE include)