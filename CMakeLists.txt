cmake_minimum_required(VERSION 3.10)

project(Assignment2)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# VTK path 
set(VTK_DIR "D:/VTK/Install/lib/cmake/vtk-9.2")

# OpenCV is installed
set(OpenCV_DIR "C:/opencv/build")

find_package(VTK REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(Assignment2 src/main.cpp)

# VTK
target_link_libraries(Assignment2 PRIVATE ${VTK_LIBRARIES})
target_include_directories(Assignment2 PRIVATE ${VTK_INCLUDE_DIRS})

vtk_module_autoinit(
  TARGETS Assignment2
  MODULES ${VTK_LIBRARIES}
)

# OpenCV
target_include_directories(Assignment2 PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(Assignment2 PRIVATE ${OpenCV_LIBS})
