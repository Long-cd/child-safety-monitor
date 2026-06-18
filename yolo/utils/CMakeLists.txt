# 最低CMake版本（兼容RV1126板端，降低至3.10以匹配主工程）
cmake_minimum_required(VERSION 3.10)

# 项目名称（简化，仅标识工具库）
project(rknn_rv1126_utils)

# ===================== 核心：查找系统OpenCV（RV1126必需） =====================
find_package(OpenCV REQUIRED)
if (NOT OpenCV_FOUND)
    message(FATAL_ERROR "❌ 未找到系统OpenCV，请先安装：sudo apt install libopencv-dev")
endif()
message(STATUS "✅ utils库：找到系统OpenCV ${OpenCV_VERSION}")

# ===================== 仅保留RV1126必需的工具库 =====================
# 1. 文件工具库（fileutils）：文件/路径解析，无冗余依赖
add_library(fileutils STATIC
    file_utils.c
)
target_include_directories(fileutils PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}  # 暴露头文件，主工程可直接include
)

# 2. 绘图工具库（imagedrawing）：推理结果可视化（画检测框/标签）
add_library(imagedrawing STATIC
    image_drawing.c
)
target_include_directories(imagedrawing PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)
# 链接系统OpenCV（绘图依赖OpenCV的绘图函数）
target_link_libraries(imagedrawing PUBLIC ${OpenCV_LIBS})

# 3. 图像工具库（imageutils）：RV1126图像预处理（缩放/格式转换/归一化）
add_library(imageutils STATIC
    image_utils.c
)
target_include_directories(imageutils PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}          # 本地头文件
    ${OpenCV_INCLUDE_DIRS}               # 系统OpenCV头文件
)
# 链接系统OpenCV（图像预处理依赖OpenCV核心/图像处理模块）
target_link_libraries(imageutils PUBLIC ${OpenCV_LIBS})

# ===================== 剔除所有冗余：删除音频/其他芯片/RGA相关逻辑 =====================
# 注：RV1126推理无需音频工具（audioutils），直接删除；RGA为其他芯片特性，也剔除

