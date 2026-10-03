cmake_minimum_required(VERSION 3.5.0)

# 交叉编译工具链
include(cross.cmake)

# 基础路径与项目名称配置
set(CURRENT_DIR ${CMAKE_CURRENT_LIST_DIR})
get_filename_component(PARENT_DIR ${CURRENT_DIR}/.. ABSOLUTE)
get_filename_component(PROJ_NAME ${PARENT_DIR} NAME)           # 取上级目录名作为项目名
project(${PROJ_NAME} VERSION 0.1.0 LANGUAGES C CXX)

# 编译参数（龙芯LoongArch适配）
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD_REQUIRED ON)

# 保守编译优化，禁用 LSX/LASX 向量扩展，防止板子 Illegal instruction
set(COMMON_FLAGS "-O2 -march=loongarch64 -mno-lsx -mno-lasx -ffunction-sections -fdata-sections -Wall")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${COMMON_FLAGS} -pthread -Wno-delete-non-virtual-dtor -Wno-placement-new")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${COMMON_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--gc-sections")

# OpenCV：ips200_fb.hpp / tft180_fb.hpp 间接 include 了 zf_device_uvc.hpp（内含 opencv2 头），
# 且 zf_device_uvc.cpp（USB 摄像头）直接调用 OpenCV 函数，因此需要 find_package 并链接 OpenCV 库。
set(CMAKE_PREFIX_PATH "/opt/ls_2k0300_env/opencv_4_10_build")
find_package(OpenCV REQUIRED)

# 头文件路径
include_directories(
    ${CURRENT_DIR}
    ${CURRENT_DIR}/../code
    ${CURRENT_DIR}/../../libraries/zf_common
    ${CURRENT_DIR}/../../libraries/zf_driver
    ${CURRENT_DIR}/../../libraries/zf_device
    ${OpenCV_INCLUDE_DIRS}
)

# 源文件：主程序 + 蜂鸣器依赖 + zf_device 全部外设库
set(DIR_SRCS
    ${CURRENT_DIR}/main.cpp
    # 用户自定义模块（code 目录）
    ${CURRENT_DIR}/../code/init.cpp
    ${CURRENT_DIR}/../code/display.cpp
    ${CURRENT_DIR}/../code/image.cpp
    ${CURRENT_DIR}/../code/ipm.cpp
    ${CURRENT_DIR}/../code/motor.cpp
    ${CURRENT_DIR}/../code/unless_debug.cpp
    # 芯片外设驱动层
    ${CURRENT_DIR}/../../libraries/zf_driver/zf_driver_gpio.cpp
    ${CURRENT_DIR}/../../libraries/zf_driver/zf_driver_file_buffer.cpp
    ${CURRENT_DIR}/../../libraries/zf_driver/zf_driver_file_string.cpp
    ${CURRENT_DIR}/../../libraries/zf_driver/zf_driver_pit.cpp
    ${CURRENT_DIR}/../../libraries/zf_driver/zf_driver_pwm.cpp
    # 公共层
    ${CURRENT_DIR}/../../libraries/zf_common/zf_common_font.cpp
    ${CURRENT_DIR}/../../libraries/zf_common/zf_common_function.cpp
    # 外接设备驱动层（zf_device 全部：测距 / 惯性 / 两块屏 / 摄像头）
    ${CURRENT_DIR}/../../libraries/zf_device/zf_device_dl1x.cpp
    ${CURRENT_DIR}/../../libraries/zf_device/zf_device_imu.cpp
    ${CURRENT_DIR}/../../libraries/zf_device/zf_device_ips200_fb.cpp
    ${CURRENT_DIR}/../../libraries/zf_device/zf_device_tft180_fb.cpp
    ${CURRENT_DIR}/../../libraries/zf_device/zf_device_uvc.cpp
)

# 生成可执行文件
add_executable(${PROJ_NAME} ${DIR_SRCS})

# 设置运行时搜索路径，让板子能定位到 OpenCV 动态库
set_target_properties(${PROJ_NAME} PROPERTIES
    INSTALL_RPATH "/opt/ls_2k0300_env/opencv_4_10_build/lib"
    BUILD_WITH_INSTALL_RPATH ON
)

# 链接依赖库（含 OpenCV，供 zf_device_uvc 使用）
target_link_libraries(${PROJ_NAME} ${OpenCV_LIBS} pthread m)