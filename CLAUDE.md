# FRDM-MCXA156 项目指南

## 项目概述
基于 NXP FRDM-MCXA156 开发板的 MCUXpresso SDK 项目，当前为 hello_world demo。

## 硬件规格

| 特性 | 详情 |
|------|------|
| MCU | MCXA156VLL (LQFP100) |
| 内核 | Arm Cortex-M33, 最高 96 MHz |
| 闪存 | 1 MB, 双 bank |
| SRAM | 128 KB (含 8 KB ECC) |
| 调试器 | 板载 MCU-Link OB (基于 LPC55S69, CMSIS-DAP) |

### 板载外设引脚

**RGB LED (低电平点亮, GPIO3):**
- RED:   P3_12 (GPIO3, pin 12)
- GREEN: P3_13 (GPIO3, pin 13)
- BLUE:  P3_0  (GPIO3, pin 0)

**用户按钮:**
- SW2: P1_7 (GPIO1, pin 7)
- SW3: P0_6 (GPIO0, pin 6)

**调试串口 (LPUART0):**
- RX: P0_2 (pin 78)
- TX: P0_3 (pin 79)
- 波特率: 115200, 8N1

**扩展接口:** Arduino UNO R3, mikroBUS, Pmod

### 时钟配置
默认使用 FRO96M 配置：Core 96MHz, System 96MHz, MAIN 96MHz, Slow 24MHz。
支持多种时钟模式: FRO12M / FRO24M / FRO48M / FRO64M / FRO96M (96MHz 为默认)。

## 项目结构

```
├── hello_world.c              # 主程序 (main)
├── CMakeLists.txt             # 顶层 CMake
├── CMakePresets.json          # CMake 预设 (debug / release)
├── mcux_include.json          # CMake 额外预设配置
├── prj.conf                   # Kconfig 项目配置(空)
├── Kconfig                    # Kconfig 菜单定义
├── example.yml                # SDK 示例元数据
├── .gitignore                 # 忽略 __repo__/
├── frdmmcxa156/
│   ├── board_files.cmake      # 板级 cmake 配置
│   ├── prj.conf               # 板级 Kconfig (组件使能)
│   ├── hello_world.mex        # MCUXpresso Config Tools 项目文件
│   ├── frdmmcxa156/
│   │   ├── board.h / board.c         # 板级初始化 & LED/按钮宏
│   │   ├── clock_config.h / clock_config.c  # 时钟配置 (多档)
│   │   └── FreeRTOSConfigBoard.h    # FreeRTOS 板级配置
│   └── hello_world/
│       ├── app.h              # BOARD_InitHardware 声明
│       ├── hardware_init.c    # 硬件初始化 (引脚→时钟→串口)
│       └── pin_mux.c / pin_mux.h  # 引脚复用配置
└── __repo__/                  # NXP SDK west 仓库 (gitignored)
```

## 技术栈

- **MCU:** NXP MCXA156 (Arm Cortex-M33, 96MHz)
- **工具链:** `arm-gnu-toolchain-14.2.rel1-mingw-w64-x86_64-arm-none-eabi`
  - 路径: `C:/Users/Jankin/.mcuxpressotools/arm-gnu-toolchain-14.2.rel1-mingw-w64-x86_64-arm-none-eabi`
- **SDK:** MCUXpresso SDK (west 仓库)
  - 路径: `d:/NXP/frdm_mcxa156`
  - SDK cmake: `$env{SdkRootDirPath}/mcuxsdk/cmake/toolchain/armgcc.cmake`
- **构建系统:** CMake + Ninja (通过 CMakePresets.json 管理)
- **Python 虚拟环境:** `C:/Users/Jankin/.mcuxpressotools/.venv_3_12/Scripts`
- **调试器:** 板载 MCU-Link (LinkServer / CMSIS-DAP)

## 构建命令

### 使用 CMakePresets (推荐)

```bash
# 配置 (debug)
cmake --preset debug -B debug

# 构建
cmake --build debug

# 或一步完成
cmake --preset debug -B debug && cmake --build debug
```

Release 构建同理，将 `debug` 替换为 `release`。

### 使用 west

```bash
# 构建
west build -b frdmmcxa156 . -d build

# 清理
west build -t clean -d build

# 配置菜单
west build -t guiconfig -d build
```

**注意:** west 依赖 `__repo__/` 中的 SDK 仓库，以及 CMakePresets.json 中的环境变量。

## 烧录与调试

```bash
# 烧录 (LinkServer)
west flash -r linkserver -d build

# 调试
west debug -r linkserver -d build
```

在 MCUXpresso IDE / VS Code 中也可通过 `.vscode/launch.json` 配置的 `mcuxpresso-debug` 类型直接调试。

## 编码规范

- C 标准库头文件用 `<>`：`#include <stdint.h>`
- SDK 头文件用 `""`：`#include "fsl_common.h"`
- 函数命名: PascalCase (`BOARD_InitHardware`)
- 宏命名: UPPER_SNAKE_CASE (`BOARD_LED_RED_GPIO`)
- 变量命名: camelCase (`coreFreq`, `ldoOption`)
- SDK 状态类型: `status_t`
- 版权头: NXP BSD-3-Clause

## 关键 API

### 硬件初始化
```c
BOARD_InitHardware();  // 依次调用: BOARD_InitPins → BOARD_InitBootClocks → BOARD_InitDebugConsole
```

### LED 控制 (board.h)
```c
LED_RED_INIT(LOGIC_LED_OFF);   // 初始化
LED_RED_ON();                   // 点亮 (低电平)
LED_RED_OFF();                  // 熄灭
LED_RED_TOGGLE();               // 翻转
// GREEN, BLUE 同理
```

### 调试输出
```c
PRINTF("hello world.\r\n");
char ch = GETCHAR();   // 从 LPUART0 读字符
PUTCHAR(ch);           // 向 LPUART0 写字符
```

## 硬件初始化流程

```
main()
  └─ BOARD_InitHardware()       (hardware_init.c)
       ├─ BOARD_InitPins()      (pin_mux.c)    — P0_2,P0_3 配置为 LPUART0
       ├─ BOARD_InitBootClocks() (clock_config.c) — 默认 BOARD_BootClockFRO96M()
       └─ BOARD_InitDebugConsole() (board.c)   — LPUART0, 115200, 12MHz clock
```

## 相关资源

- [NXP FRDM-MCXA156 官方页面](https://www.nxp.com/design/design-center/development-boards-and-designs/FRDM-MCXA156)
- [MCUXpresso SDK 在线文档](https://mcuxpresso.nxp.com/mcuxsdk/25.12.00/html/boards/MCX/frdmmcxa156/)
- [用户手册 UM12121](https://www.nxp.com.cn/docs/en/user-manual/UM12121.pdf)
- [SDK CLI 开发指南](https://mcuxpresso.nxp.com/mcuxsdk/25.12.00/html/gsd/cli.html)
- [NXP 中文社区实验手册](https://www.nxpic.org.cn/document/id-18083)
