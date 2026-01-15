cmake_minimum_required(VERSION 3.16)
project(picam-capture)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBCAMERA REQUIRED libcamera)
pkg_check_modules(LIBGPIOD REQUIRED libgpiod)
pkg_check_modules(TURBOJPEG REQUIRED libturbojpeg)

add_executable(picam-capture main.cpp)

target_include_directories(picam-capture PRIVATE
    ${LIBCAMERA_INCLUDE_DIRS}
    ${LIBGPIOD_INCLUDE_DIRS}
    ${TURBOJPEG_INCLUDE_DIRS}
)

target_link_libraries(picam-capture
    ${LIBCAMERA_LIBRARIES}
    ${LIBGPIOD_LIBRARIES}
    ${TURBOJPEG_LIBRARIES}
)

target_compile_options(picam-capture PRIVATE
    ${LIBCAMERA_CFLAGS_OTHER}
)
