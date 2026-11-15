# Cropped-CMSIS-DSP

Cropped version based on v1.14.2
该版本遵照源码仓库 `README` 说明，基于 `V1.14.2` 裁剪

> 源码仓库 https://github.com/ARM-software/CMSIS-DSP
> 官方文档 https://arm-software.github.io/CMSIS-DSP/latest/index.html

# 源码仓库 `README` 说明

The only folders required to build and use CMSIS-DSP Library are:

- Source
- Include
- PrivateInclude
- ComputeLibrary (only when using Neon)

Other folders are part of different projects, tests or examples.

# 简介

CMSIS-DSP 软件库是一套用于基于 Cortex-M 和 Cortex-A 处理器的设备的通用计算处理函数。该库分为多个功能，每个功能涵盖一个特定类别，如：

- 基本数学函数
- 快速数学函数
- 复杂的数学函数
- 滤波器函数
- 矩阵函数
- 变换函数
- 电机控制函数
- 统计功能
- 支持功能
- 插值函数
- 支持向量机函数 (SVM)
- 贝叶斯分类器函数
- 距离函数
- 四元数函数

该库通常具有单独的函数来对 8 位整数、16 位整数、32 位整数和 32 位浮点值进行操作。

将 CMSIS-DSP 库搭配具有 FPU 的处理器，将能够提升运算效率。Cortex-M4 内核便具有单精度浮点单元 (FPU)，支持所有 Arm 单精度数据处理指令和所有数据类型。它还实现了全套 DSP（数字信号处理）指令和增强应用程序安全性的内存保护单元 (MPU)。

> 关于 CMSIS 的更多说明，请参考[官方文档](https://arm-software.github.io/CMSIS_5/latest/General/html/index.html). 选择其中的 [CMSIS-DSP 标签页](https://arm-software.github.io/CMSIS_5/latest/DSP/html/index.html) 以查看 CMSIS-DSP 的相关信息。
> 请注意，CMSIS-DSP V1.10.1 及其之后的版本迁移至了独立的新仓库，[新文档地址](https://arm-software.github.io/CMSIS-DSP/latest/index.html)

# 主要版本差异

建议使用最新版本的 CMSIS-DSP 源码加入工程进行编译.

- V1.10.1 及之后的版本迁移到新的仓库
- V1.10.0 添加了 `atan2` 支持
- V1.9.0 添加了`矩阵向量相乘`的功能，添加`复数矩阵`转置，支持四元数，添加对 `f16` 类型的更多支持
- V1.8.0 添加了`支持向量机`、`贝叶斯函数`等等新函数
- V1.6.0 更改了 `DSP` 文件夹结构
- V1.5.3 删除预编译宏 `__FPU_USED`, `__DSP_PRESENT`

# 使用说明

需要注意，对于这样裁剪得到的软件包，直接编译会有大量 WARNING，具体原因可参考 [这条经验](https://g6ursaxeei.feishu.cn/wiki/wikcnvTNsHomNrfLE0PVHN5VWhc?field=fldrk77lHy&record=recDLg4nf3&table=tbl5nghP4qHQIiZ5&view=vewlyW2exr)。因此，当添加 CMSIS-DSP 目录以及包含有 CMSIS-DSP 文件夹的父目录时，可以加上正则表达式 [^a] 来解决 WARNING.
