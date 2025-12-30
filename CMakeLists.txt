cmake_minimum_required(VERSION 3.10)

project(VASTFinalProject)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(VTK_DIR "D:/VTK/Install/lib/cmake/vtk-9.2")
find_package(VTK REQUIRED)

set(OpenCV_DIR "C:/opencv/build")
find_package(OpenCV REQUIRED)

include_directories(
    ${VTK_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

file(GLOB SOURCES "src/*.cpp" "src/*.cxx" "src/*.h" "src/*.hpp")

add_executable(VASTFinalProject ${SOURCES})

target_link_libraries(VASTFinalProject
    ${VTK_LIBRARIES}
    ${OpenCV_LIBS}
)

vtk_module_autoinit(
    TARGETS VASTFinalProject
    MODULES ${VTK_LIBRARIES}
)
