cmake_minimum_required(VERSION 3.5)
project(mlx90640_library)

include(TargetArch.cmake)
target_architecture(CMAKE_TARGET_ARCHITECTURES)

message(STATUS ${CMAKE_TARGET_ARCHITECTURES})

set(CMAKE_CXX_STANDARD 14)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pg -g")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -pg -g")

include_directories(examples)
include_directories(mlx90640)

set(SOURCE_FILES
#    examples/fb.h
#    examples/fb.c
    examples/fbuf.cpp
    mlx90640/MLX90640_API.h
    mlx90640/MLX90640_I2C_Driver.h
    )

if (${CMAKE_TARGET_ARCHITECTURES} STREQUAL "x86_64")
    set(SOURCE_FILES ${SOURCE_FILES}
        mlx90640/MLX90640_API_stub.cpp
        mlx90640/MLX90640_RPI_I2C_Driver_stub.cpp
        )
else()

    set(SOURCE_FILES ${SOURCE_FILES}
            mlx90640/MLX90640_API.cpp
#            mlx90640/MLX90640_RPI_I2C_Driver.cpp
            mlx90640/MLX90640_linux_i2c_driver.cpp)

endif()

add_executable(mlx90640_library ${SOURCE_FILES})

if (${CMAKE_TARGET_ARCHITECTURES} STREQUAL "x86_64")
else()
#    target_link_libraries( mlx90640_library bcm2835 )
    target_link_libraries( mlx90640_library)
endif()
