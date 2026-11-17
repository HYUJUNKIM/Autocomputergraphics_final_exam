cmake_minimum_required(VERSION 3.20)

project(Drive-vehicle)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

find_package(Qt6 REQUIRED COMPONENTS Widgets OpenGL)
find_package(VTK REQUIRED)

add_executable(Drive-vehicle
    main.cpp
    MainWindow.h
    MainWindow.cpp
    PidActorManager.h
    PidActorManager.cpp
)

target_link_libraries(Drive-vehicle PRIVATE
    Qt6::Widgets
    Qt6::OpenGL
    ${VTK_LIBRARIES}
)

vtk_module_autoinit(
    TARGETS Drive-vehicle
    MODULES ${VTK_LIBRARIES}
)

if (MSVC)
  add_compile_options(/utf-8)
endif()
