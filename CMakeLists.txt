cmake_minimum_required(VERSION 3.28)

project(RRT_library LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

set(SFML_STATIC_LIBRARIES TRUE)

set(SFML_DIR "C:/Libraries/SFML-3.0.2-install/lib/cmake/SFML")
set(CMAKE_PREFIX_PATH "C:/Libraries/SFML-3.0.2-install")

find_package(SFML 3 REQUIRED COMPONENTS Graphics Window System Audio)

add_executable(RRT_library
        main.cpp
        RRT.h
        RRT_Visualizer.h
        Path_Refinement.h
)

target_link_libraries(RRT_library PRIVATE
        SFML::Graphics
        SFML::Window
        SFML::System
        SFML::Audio
)

target_include_directories(RRT_library PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
