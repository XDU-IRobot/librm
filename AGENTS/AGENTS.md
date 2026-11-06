# librm — AI Agent Guide

跨平台 Robomaster 嵌入式软件开发框架，为 STM32、Linux/Raspberry Pi/Jetson 上的机器人开发任务提供统一的 HAL、设备驱动与算法库。

详细说明见 [README.md](README.md) 与 [文档站](https://librm.xduirobot.cc/)，文档站源码位于 [docs/](docs/) 目录。对代码进行修改后应该同步检查文档站相关内容是否需要更新。

## Guidelines

- 自然语言表达方面需要遵守 [humanize-skill.md](humanize-skill.md) 中的条文。

- 代码风格严格以Google C++ Style Guide为基础。在此之上：
  - 80列限制改为120列

  - 对定义在.cc源文件里的类，成员函数直接内联定义在类里；若有必要把类定义放在头文件里，则成员函数的实现分开放在.cc源文件里。

  - 对类、函数、命名空间、成员等的注释采用doxygen规范，对行内的简单解释可直接//后接文本，自行灵活决定

  - 已配置了clang-format规则，在仓库根目录下，见 [.clang-format](.clang-format)

- 由于项目性质关系，工作时应该更看重开发效率、代码可读性而非交付结果的流程上的规范性。严格遵循最小功能原则，仅实现要求实现的功能，不要擅自添加各种过度的防御性判断、正常调试时基本不会动的可调参数，久而久之这些垃圾代码会积重难返。

- 注释保持简洁可读。不要写大段的注释，非必要不要用等号拼分割线、不要用箭头、emoji等等非ASCII字符，在有必要说明的地方简明扼要地注一两行解释，或在行内写一句话，过多的废话严重影响代码的可读性。反面例子：

  ```cmake
  # 第三方依赖：OpenMP + TensorRT + TensorRT-YOLO + 自定义插件。
  #
  # 本文件对外只暴露一个接口目标 `${PROJECT_NAME}_libs`，外层直接链接它即可：
  #
  #     add_subdirectory(libs)
  #     target_link_libraries(<target> ${PROJECT_NAME}_libs)
  #
  # 不使用 libs/TensorRT-YOLO 自带的顶层 CMakeLists，原因：
  #   - 它强制要求 -DTRT_PATH，且拼死 ${TRT_PATH}/include 与 ${TRT_PATH}/lib，
  #     不接受 Debian multiarch / JetPack 的散装布局；
  #   - 带自己的 install 规则，会把 trtyolo.so 装进 ament 前缀；
  #   - 会 FORCE 覆盖 CMAKE_CUDA_ARCHITECTURES。
  # 因此这里直接编译它的源码。

  find_package(CUDAToolkit REQUIRED)

  # ------------------------------------------------------------------------------
  # TensorRT 探测
  #
  # 支持两种安装形态，路径全部来自缓存变量或本文件，不改动系统目录：
  #   1) 标准布局：-DTRT_PATH=/opt/tensorrt，其下有 include/ 与 lib/
  #   2) Debian/Ubuntu multiarch 或 JetPack 布局：头在 /usr/include/<triplet>，
  #      库在 /usr/lib/<triplet>（本机即 /usr/include/x86_64-linux-gnu）
  # ------------------------------------------------------------------------------
  set(TRT_PATH "" CACHE PATH "TensorRT 根目录（含 include/ 与 lib/）；留空则自动探测系统安装")
  ```

- 若需要用到涉及复杂数学原理的算法，首先考虑有没有第三方的成熟实现，非必要不要手搓。

- 若无特殊说明，实现任何功能时禁止进行动态堆分配（new/malloc），以保证在 STM32 等资源受限平台上的行为确定性。如果需要使用STL容器或算法，考虑 ETL（Embedded Template Library）提供的无堆版本。

- librm 整合有 ETL（Embedded Template Library）作为轻量级的 STL 替代方案，提供了常用的数据结构（vector/map/string 等）和算法支持。实现任何功能时先检查 ETL 是否已有对应的实现，避免重复造轮子。

## 项目架构

```
rm::core::    — 类型别名、Trait、异常、时间（平台无关）
rm::hal::     — 外设抽象接口 + 各平台实现
rm::device::  — 设备驱动（电机、传感器、遥控、裁判系统）
rm::modules:: — 算法模块（PID、AHRS、运动学、滤波器…）
```
