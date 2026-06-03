cmake_minimum_required(VERSION 3.16)

# ================================================================
# RemoteID Scanner - 统一项目构建
# ================================================================
#
# 通过 MAIN_DIR 变量选择构建目标：
#   - main_rx: 接收解析程序 (C-RID Scanner)
#   - main_tx: 模拟发送程序 (C-RID Transmitter)
#
# 用法:
#   idf.py set-target esp32 -DMAIN_DIR=main_rx
#   idf.py build
#
#   idf.py set-target esp32 -DMAIN_DIR=main_tx
#   idf.py build
#
# 默认: main_rx
# ================================================================

# 默认构建目标
if(NOT DEFINED MAIN_DIR)
    set(MAIN_DIR main_rx)
endif()

message(STATUS ">>> Building target: ${MAIN_DIR}")

# 强制将 MAIN_DIR 设为主组件目录，并添加 components 目录
set(EXTRA_COMPONENT_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/${MAIN_DIR}
                         ${CMAKE_CURRENT_SOURCE_DIR}/components)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(remoteid_scanner)