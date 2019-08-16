cmake_minimum_required(VERSION 3.5)
project(mlx90640_library)

include(TargetArch.cmake)
target_architecture(CMAKE_TARGET_ARCHITECTURES)

message(STATUS ${CMAKE_TARGET_ARCHITECTURES})

set(CMAKE_CXX_STANDARD 14)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fpermissive")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fpermissive")

set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -flto")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -flto")

include_directories(mlx90640)

set(SOURCE_FILES
        mlx90640/main.cpp
        mlx90640/MLX90640_API.h
        mlx90640/MLX90640_I2C_Driver.h
        mlx90640/MLX90640_API.cpp
        mlx90640/MLX90640_linux_i2c_driver.cpp
        )

add_executable(mlx90640_library ${SOURCE_FILES})
