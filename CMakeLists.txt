cmake_minimum_required(VERSION 3.16)
project(picam-capture)

# Use ccache if available
find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM)
    set(CMAKE_CXX_COMPILER_LAUNCHER ${CCACHE_PROGRAM})
    message(STATUS "Using ccache: ${CCACHE_PROGRAM}")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBCAMERA REQUIRED libcamera)
pkg_check_modules(LIBGPIOD REQUIRED libgpiod)
pkg_check_modules(TURBOJPEG REQUIRED libturbojpeg)
pkg_check_modules(EXIV2 REQUIRED exiv2)

# LCD HAT library sources
set(LCD_HAT_DIR ${CMAKE_SOURCE_DIR}/1.3inch_LCD_HAT_code/1.3inch_LCD_HAT_code/c)
set(LCD_HAT_SOURCES
    ${LCD_HAT_DIR}/lib/Config/DEV_Config.c
    ${LCD_HAT_DIR}/lib/LCD/LCD_1in3.c
    ${LCD_HAT_DIR}/lib/GUI/GUI_Paint.c
)

add_executable(picam-capture main.cpp ${LCD_HAT_SOURCES})

target_include_directories(picam-capture PRIVATE
    ${LIBCAMERA_INCLUDE_DIRS}
    ${LIBGPIOD_INCLUDE_DIRS}
    ${TURBOJPEG_INCLUDE_DIRS}
    ${EXIV2_INCLUDE_DIRS}
    ${LCD_HAT_DIR}/lib/Config
    ${LCD_HAT_DIR}/lib/LCD
    ${LCD_HAT_DIR}/lib/GUI
    ${LCD_HAT_DIR}/lib/Fonts
)

target_link_libraries(picam-capture
    ${LIBCAMERA_LIBRARIES}
    ${LIBGPIOD_LIBRARIES}
    ${TURBOJPEG_LIBRARIES}
    ${EXIV2_LIBRARIES}
    lgpio
)

target_compile_options(picam-capture PRIVATE
    ${LIBCAMERA_CFLAGS_OTHER}
)

target_compile_definitions(picam-capture PRIVATE USE_DEV_LIB)
