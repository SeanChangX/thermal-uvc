cmake_minimum_required(VERSION 3.10)
project(thermal_uvc)

set(CMAKE_CXX_STANDARD 14)

find_package(OpenCV REQUIRED)

include_directories(
    ${OpenCV_INCLUDE_DIRS}
    /usr/local/include/libuvc
)

link_directories(/usr/local/lib)

add_executable(thermal_uvc main.cpp)
target_link_libraries(thermal_uvc uvc usb-1.0 ${OpenCV_LIBS})
