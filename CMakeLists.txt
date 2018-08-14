cmake_minimum_required(VERSION 3.12)
project(mlx90640_library)

set(CMAKE_CXX_STANDARD 14)

include_directories(examples)
include_directories(mlx90640)

add_executable(mlx90640_library
        examples/fb.h
        examples/fb.c
        examples/fbuf.cpp
        mlx90640/MLX90640_API_stub.cpp
        mlx90640/MLX90640_API.h
        mlx90640/MLX90640_I2C_Driver.h
        mlx90640/MLX90640_RPI_I2C_Driver_stub.cpp
        )
