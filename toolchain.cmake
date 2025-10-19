# toolchain.template.cmake（提交到Git，作为模板）
# 1. 交叉编译工具链路径（arm-none-eabi-*）
set(TOOLCHAIN_BIN_DIR "D:/Embedded/STM32CubeCLT/STM32CubeCLT_1.19.0/GNU-tools-for-STM32/bin")  # 例如实验室：/opt/gcc-arm-none-eabi/bin；家里：C:/Program Files/arm-none-eabi/bin

# 2. CMake可执行文件路径（如果两台主机的CMake安装路径不同）
set(CMAKE_PROGRAM_PATH "D:/Embedded/STM32CubeCLT/STM32CubeCLT_1.19.0/CMake/bin")  # 可选，若CMake不在系统PATH中

# 3. 项目构建路径（可选，指定本地的build目录，避免与其他环境冲突）
set(LOCAL_BUILD_DIR "${CMAKE_SOURCE_DIR}/build_home")  # 例如实验室：build_lab；家里：build_home

# 设置编译器（基于上面的工具链路径）
set(CMAKE_C_COMPILER "${TOOLCHAIN_BIN_DIR}/arm-none-eabi-gcc")
set(CMAKE_CXX_COMPILER "${TOOLCHAIN_BIN_DIR}/arm-none-eabi-g++")
set(CMAKE_ASM_COMPILER "${TOOLCHAIN_BIN_DIR}/arm-none-eabi-gcc")