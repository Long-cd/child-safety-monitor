cmake_minimum_required(VERSION 3.10)

project(yolo_pose_seg_rk3588)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_definitions(-DRKNPU2)

# OpenCV
find_package(OpenCV REQUIRED)
if (NOT OpenCV_FOUND)
    message(FATAL_ERROR "OpenCV not found, install: sudo apt install libopencv-dev")
endif()
message(STATUS "Found OpenCV: ${OpenCV_VERSION}")

# ximgproc (required for stereo WLS filter)
find_package(ximgproc QUIET)
if (ximgproc_FOUND)
    message(STATUS "Found ximgproc")
else()
    message(STATUS "ximgproc not found as separate package, assuming included in OpenCV")
endif()

# Threads
find_package(Threads REQUIRED)
message(STATUS "Found Threads: ${CMAKE_THREAD_LIBS_INIT}")

# ---- include directories ----
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${OpenCV_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/utils
    ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/stb_image
    ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/jpeg_turbo/include
    ${CMAKE_CURRENT_SOURCE_DIR}/timer
    ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/timer
    ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/librga/include
    ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/allocator/dma
    ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/allocator/drm
    ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/rknpu2/include
    ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/json
)

# ---- library search paths (defaults for RK3588 aarch64) ----
set(RKNN_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/rknn_lib CACHE PATH "RKNN runtime library directory")
set(RGA_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/librga/Linux/aarch64 CACHE PATH "RGA library directory")
set(TURBOJPEG_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/jpeg_turbo/Linux/aarch64 CACHE PATH "TurboJPEG library directory")

link_directories(${RKNN_LIB_DIR})
link_directories(${RGA_LIB_DIR})
link_directories(${TURBOJPEG_LIB_DIR})
link_directories(/usr/lib/aarch64-linux-gnu)

# ---- RPATH ----
set(CMAKE_BUILD_RPATH
    ${RKNN_LIB_DIR}
    ${RGA_LIB_DIR}
    ${TURBOJPEG_LIB_DIR}
    /usr/lib/aarch64-linux-gnu
)

# ---- utils subdirectory ----
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/utils/ utils.out)

# ---- executable ----
add_executable(${PROJECT_NAME}
    src/main.cc
    src/yolov8_pose.cc
    src/yolov8_seg.cc
    src/postprocess_pose.cc
    src/postprocess_seg.cc
    src/behavior_analyzer.cc
    src/stereo_camera.cc
    src/danger_monitor.cc
    src/network_sender.cc
    src/sys_monitor.cc
)

# ---- link libraries ----
target_link_libraries(${PROJECT_NAME}
    imageutils
    fileutils
    imagedrawing
    ${OpenCV_LIBS}
    rknnrt
    rga
    turbojpeg
    dl
    pthread
    Threads::Threads
    stdc++fs
    m
)

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# ---- install ----
install(TARGETS ${PROJECT_NAME} DESTINATION .)
install(DIRECTORY DESTINATION inputimage)
install(DIRECTORY DESTINATION output_json)

file(GLOB RKNN_FILES "${CMAKE_CURRENT_SOURCE_DIR}/model/*.rknn")
if (RKNN_FILES)
    install(FILES ${RKNN_FILES} DESTINATION model)
    message(STATUS "Found RKNN models: ${RKNN_FILES}")
else()
    message(WARNING "No RKNN model files found in ./model/")
endif()

file(GLOB LABEL_FILES "${CMAKE_CURRENT_SOURCE_DIR}/model/*labels_list.txt")
if (LABEL_FILES)
    install(FILES ${LABEL_FILES} DESTINATION model)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2")

if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION LESS 9)
    target_link_libraries(${PROJECT_NAME} stdc++fs)
endif()
