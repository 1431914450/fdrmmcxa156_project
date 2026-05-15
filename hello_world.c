/*
 * Copyright 2026 NXP
 * DMA-based I3C-to-UART temperature reading example for FRDM-MCXA156
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*  Standard C Included Files */
#include <string.h>
#include <stdio.h>
/*  SDK Included Files */
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "fsl_clock.h"
#include "fsl_reset.h"
#include "fsl_p3t1755.h"
#include "fsl_i3c_edma.h"
#include "fsl_lpuart_edma.h"
#include "board.h"
#include "app.h"

/*
 * ============================================================================
 * DMA（直接内存访问）原理概述
 * ============================================================================
 *
 * 【什么是 DMA】
 * DMA（Direct Memory Access，直接内存访问）是一种允许外设与内存之间直接传输
 * 数据的硬件机制，传输过程不需要 CPU 参与。这极大地释放了 CPU 带宽，让 CPU
 * 可以专注于其他计算任务。
 *
 * 【DMA 的核心概念】
 *
 * 1. DMA 通道（Channel）
 *    - DMA 控制器有多个独立的通道，每个通道可以配置一个独立的传输任务
 *    - MCXA156 的 DMA3 有 8 个通道（CH0 ~ CH7），可以同时进行 8 个独立传输
 *    - 本例使用：
 *      CH0 = I3C0 发送（TX）
 *      CH1 = I3C0 接收（RX）
 *      CH2 = LPUART0 发送（TX）
 *
 * 2. DMA 请求源（Request Source / Channel Mux）
 *    - 每个 DMA 通道需要通过"通道复用器（Channel Mux）"连接到一个外设请求源
 *    - 当外设（如 I3C 或 UART）需要数据传输时，它会向 DMA 发送"请求信号"
 *    - DMA 收到请求后，自动执行一次数据传输
 *    - MCXA156 的 DMA 请求源编号由芯片手册定义：
 *      kDma0RequestMuxI3c0Tx  = 8   → I3C0 发送请求
 *      kDma0RequestMuxI3c0Rx  = 7   → I3C0 接收请求
 *      kDma0RequestLPUART0Tx  = 22  → LPUART0 发送请求
 *
 * 3. TCD（Transfer Control Descriptor，传输控制描述符）
 *    - TCD 是 DMA 传输的核心数据结构，类似一个"传输任务单"
 *    - 每个 TCD 描述了：
 *      a) 源地址（SADDR）—— 数据从哪里来
 *      b) 目的地址（DADDR）—— 数据到哪里去
 *      c) 每次传输多少字节（SOFF / DOFF）
 *      d) 总共传输多少字节（NBYTES）
 *      e) 传输完成后做什么（CSR）
 *    - SDK 中的 edma_transfer_config_t 结构体封装了 TCD 的配置
 *    - 高级功能如 scatter-gather（分散-聚集）需要多个 TCD 链接使用
 *
 * 4. 回调机制（Callback）
 *    - DMA 传输是异步的 — 启动传输后 CPU 可以继续执行其他代码
 *    - 传输完成后，DMA 硬件触发中断，SDK 调用预先注册的回调函数
 *    - 回调函数中通常设置一个完成标志（如 g_transferComplete = true）
 *    - 主循环轮询这个标志来判断传输是否完成
 *
 * 5. DMA 句柄（Handle）
 *    - SDK 使用句柄来管理 DMA 通道的状态
 *    - edma_handle_t：EDMA 底层通道句柄（每个 DMA 通道一个）
 *    - i3c_master_edma_handle_t：I3C EDMA 上层句柄（封装了 TX + RX 两个通道）
 *    - lpuart_edma_handle_t：LPUART EDMA 上层句柄（封装了 TX 通道）
 *
 * 【DMA 传输流程（以 I3C 读取温度为例）】
 *
 *   第1步：CPU 配置 DMA 通道
 *          - EDMA_CreateHandle() 创建句柄
 *          - EDMA_SetChannelMux() 设置请求源
 *
 *   第2步：CPU 配置 I3C EDMA 句柄
 *          - I3C_MasterTransferCreateHandleEDMA() 将 DMA 通道绑定到 I3C
 *
 *   第3步：CPU 启动 DMA 传输
 *          - 调用 I3C_MasterTransferEDMA()，配置好源/目的/数据量
 *          - SDK 内部将配置写入 TCD，设置 DMA 通道的硬件请求使能
 *
 *   第4步：DMA 自动工作（CPU 可以干其他事）
 *          - I3C 外设发出发送请求 → DMA CH0 自动将命令写入 I3C TX FIFO
 *          - I3C 外设发出接收请求 → DMA CH1 自动从 I3C RX FIFO 读取数据到内存
 *
 *   第5步：传输完成，中断触发
 *          - DMA 通道完成所有传输后，触发 DMA_CHx_IRQn 中断
 *          - 中断处理链：
 *            DMA_CH1_IRQHandler（启动文件弱定义）
 *            → DMA_CH1_DriverIRQHandler（fsl_edma_soc.c）
 *            → EDMA_DriverIRQHandler(0, 1)（EDMA4 驱动）
 *            → EDMA_HandleIRQ()（处理完成标志）
 *            → i3c_master_edma_callback()（用户回调，设置完成标志）
 *
 *   第6步：CPU 检测完成标志，处理数据
 *
 * 【为什么使用 DMA 而不是轮询？】
 *   - 轮询方式：CPU 循环检查外设状态寄存器，等待传输完成（占着 CPU）
 *   - 中断方式：外设完成后发中断，但每个字节都可能中断 CPU
 *   - DMA 方式：一次配置，整个数据块自动传输完毕后才中断一次
 *   - 本例中 I3C 读取 2 字节温度值，数据量很小，DMA 的优势不明显
 *     但当传输数据量大（如几百字节的传感器数据流）时，DMA 优势巨大
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* DMA 传输完成的超时计数 */
#define DMA_TIMEOUT 0xFFFFU

/* UART 发送缓冲区大小 */
#define UART_TX_BUFFER_SIZE 64U

/* P3T1755 温度传感器 I3C 地址（动态地址） */
#define SENSOR_ADDR 0x08U

/* I3C CCC 命令 */
#define CCC_RSTDAA  0x06U /* 重置动态地址 */
#define CCC_SETDASA 0x87U /* 设置动态地址 */

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/* I3C EDMA 传输完成回调函数 */
static void i3c_master_edma_callback(I3C_Type *base, i3c_master_edma_handle_t *handle,
                                      status_t status, void *userData);

/* LPUART EDMA 发送完成回调函数 */
static void lpuart_edma_callback(LPUART_Type *base, lpuart_edma_handle_t *handle,
                                  status_t status, void *userData);

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*
 * DMA 缓冲区必须放在"不可缓存"的内存区域（Non-Cacheable Section）
 *
 * 为什么？Cortex-M33 有数据缓存（Data Cache）：
 *   - 如果 DMA 缓冲区在可缓存区域：
 *     CPU 写入数据 → 数据可能在缓存中，还没写入物理内存
 *     DMA 从物理内存读取 → 读到了旧数据！
 *   - 同理，DMA 写入物理内存后，CPU 可能从缓存读到旧数据
 *   - 解决方案：将 DMA 缓冲区放在不可缓存区域，确保 CPU 和 DMA 看到的
 *     数据是一致的
 */
/*
 * I3C 接收缓冲区 — DMA 从 I3C RX FIFO 搬移温度数据到这里
 * AT_NONCACHEABLE_SECTION 将变量放在"不可缓存"内存区域，
 * 确保 CPU 和 DMA 看到的数据一致。
 */
AT_NONCACHEABLE_SECTION(static uint8_t g_i3cRxBuffer[2]);


/*
 * UART 发送缓冲区 — CPU 写入格式化字符串，DMA 搬移到 UART TX 寄存器
 * 必须放在不可缓存区域，与 g_i3cRxBuffer 同理
 */
AT_NONCACHEABLE_SECTION(static char g_uartTxBuffer[UART_TX_BUFFER_SIZE]);

/*
 * EDMA 底层通道句柄
 * 每个 DMA 通道需要一个独立的句柄，用于底层 EDMA 驱动的状态管理
 */
static edma_handle_t g_i3cTxEdmaHandle;   /* I3C TX 使用 DMA CH0 */
static edma_handle_t g_i3cRxEdmaHandle;   /* I3C RX 使用 DMA CH1 */
static edma_handle_t g_uartTxEdmaHandle;  /* UART TX 使用 DMA CH2 */

/*
 * 上层外设 EDMA 句柄
 * 这些句柄封装了底层 DMA 句柄，提供外设级别的 DMA 传输 API
 */
static i3c_master_edma_handle_t g_i3cMasterEdmaHandle;
static lpuart_edma_handle_t g_uartEdmaHandle;

/*
 * 传输完成标志和状态
 * 回调函数设置这些标志，主循环轮询检查
 */
static volatile bool g_i3cTransferComplete = false;
static volatile status_t g_i3cTransferStatus = kStatus_Success;
static volatile bool g_uartTransferComplete = false;
static volatile status_t g_uartTransferStatus = kStatus_Success;

/* P3T1755 传感器驱动句柄 */
static p3t1755_handle_t g_p3t1755Handle;

/*
 * I3C EDMA 回调函数表
 * SDK 使用回调表（而非单个函数指针）来支持多种事件：
 *   - slave2Master：从机请求主机传输（本例不使用）
 *   - ibiCallback：In-Band Interrupt（带内中断，本例不使用）
 *   - transferComplete：传输完成（本例核心回调）
 */
static const i3c_master_edma_callback_t g_i3cCallback = {
    .slave2Master = NULL,
    .ibiCallback = NULL,
    .transferComplete = i3c_master_edma_callback,
};

/*******************************************************************************
 * Code
 ******************************************************************************/

/*
 * I3C EDMA 传输完成回调函数
 *
 * 当 I3C DMA 传输完成时，SDK 的 IRQ 处理链会调用此函数。
 * 这里只做最小的工作：设置完成标志和保存状态，不执行耗时操作。
 * 把数据处理留给主循环，避免在中断上下文中花费过多时间。
 *
 * @param base     I3C 外设基地址
 * @param handle   I3C EDMA 句柄
 * @param status   传输状态（kStatus_Success 表示成功）
 * @param userData 用户自定义数据（本例为 NULL）
 */
static void i3c_master_edma_callback(I3C_Type *base, i3c_master_edma_handle_t *handle,
                                      status_t status, void *userData)
{
    g_i3cTransferComplete = true;
    g_i3cTransferStatus = status;
}

/*
 * LPUART EDMA 发送完成回调函数
 *
 * 当 UART DMA 发送完成（所有数据已从内存搬移到 UART TX 寄存器并发送出去）
 * 时调用此函数。
 */
static void lpuart_edma_callback(LPUART_Type *base, lpuart_edma_handle_t *handle,
                                  status_t status, void *userData)
{
    g_uartTransferComplete = true;
    g_uartTransferStatus = status;
}

/*
 * 使用 I3C 为 P3T1755 传感器分配动态地址
 *
 * I3C 协议中，每个从设备有一个静态地址（SENSOR_SLAVE_ADDR = 0x48）
 * 和一个由主机分配的动态地址（SENSOR_ADDR = 0x08）。
 * 动态地址分配流程：
 *   1. 广播 RSTDAA（重置所有从设备的动态地址）
 *   2. 通过 SETDASA CCC 命令为指定静态地址的设备分配动态地址
 *
 * 这里使用阻塞式 I3C 传输（非 DMA），因为地址分配只在初始化时执行一次。
 */
static status_t p3t1755_set_dynamic_address(void)
{
    status_t result = kStatus_Success;
    i3c_master_transfer_t masterXfer = {0};
    uint8_t txBuff[1];

    /* 第1步：广播 RSTDAA，重置总线上所有从设备的动态地址 */
    txBuff[0] = CCC_RSTDAA;
    masterXfer.slaveAddress = 0x7E;        /* I3C 广播地址 */
    masterXfer.data = txBuff;
    masterXfer.dataSize = 1;
    masterXfer.direction = kI3C_Write;
    masterXfer.busType = kI3C_TypeI3CSdr;  /* SDR 模式 */
    masterXfer.flags = kI3C_TransferDefaultFlag;
    result = I3C_MasterTransferBlocking(EXAMPLE_MASTER, &masterXfer);
    if (result != kStatus_Success)
    {
        return result;
    }

    /* 第2步：发送 SETDASA CCC（带 NoStop 标志，保持总线占用）*/
    memset(&masterXfer, 0, sizeof(masterXfer));
    txBuff[0] = CCC_SETDASA;
    masterXfer.slaveAddress = 0x7E;
    masterXfer.data = txBuff;
    masterXfer.dataSize = 1;
    masterXfer.direction = kI3C_Write;
    masterXfer.busType = kI3C_TypeI3CSdr;
    masterXfer.flags = kI3C_TransferNoStopFlag;  /* 不发送 STOP */
    result = I3C_MasterTransferBlocking(EXAMPLE_MASTER, &masterXfer);
    if (result != kStatus_Success)
    {
        return result;
    }

    /* 第3步：发送动态地址（0x08 左移 1 位 = 0x10）给静态地址 0x48 的设备 */
    memset(&masterXfer, 0, sizeof(masterXfer));
    txBuff[0] = SENSOR_ADDR << 1;
    masterXfer.slaveAddress = SENSOR_SLAVE_ADDR;  /* 静态地址 0x48 */
    masterXfer.data = txBuff;
    masterXfer.dataSize = 1;
    masterXfer.direction = kI3C_Write;
    masterXfer.busType = kI3C_TypeI3CSdr;
    masterXfer.flags = kI3C_TransferDefaultFlag;
    return I3C_MasterTransferBlocking(EXAMPLE_MASTER, &masterXfer);
}

/*
 * I3C 写传感器寄存器（非 DMA，供 P3T1755 驱动初始化使用）
 *
 * P3T1755 驱动通过函数指针调用此函数来写传感器配置寄存器。
 * 初始化阶段使用阻塞传输更简单可靠。
 */
static status_t I3C_WriteSensor(uint8_t deviceAddress, uint32_t regAddress,
                                 uint8_t *regData, size_t dataSize)
{
    i3c_master_transfer_t masterXfer = {0};

    masterXfer.slaveAddress = deviceAddress;
    masterXfer.direction = kI3C_Write;
    masterXfer.busType = kI3C_TypeI3CSdr;
    masterXfer.subaddress = regAddress;
    masterXfer.subaddressSize = 1;
    masterXfer.data = regData;
    masterXfer.dataSize = dataSize;
    masterXfer.flags = kI3C_TransferDefaultFlag;

    return I3C_MasterTransferBlocking(EXAMPLE_MASTER, &masterXfer);
}

/*
 * I3C 读传感器寄存器（非 DMA，供 P3T1755 驱动初始化使用）
 *
 * 注意：主循环中读取温度使用 DMA 方式（I3C_MasterTransferEDMA），
 * 此函数仅用于传感器初始化阶段的寄存器读取。
 */
static status_t I3C_ReadSensor(uint8_t deviceAddress, uint32_t regAddress,
                                uint8_t *regData, size_t dataSize)
{
    i3c_master_transfer_t masterXfer = {0};

    masterXfer.slaveAddress = deviceAddress;
    masterXfer.direction = kI3C_Read;
    masterXfer.busType = kI3C_TypeI3CSdr;
    masterXfer.subaddress = regAddress;
    masterXfer.subaddressSize = 1;
    masterXfer.data = regData;
    masterXfer.dataSize = dataSize;
    masterXfer.flags = kI3C_TransferDefaultFlag;

    return I3C_MasterTransferBlocking(EXAMPLE_MASTER, &masterXfer);
}

/*
 * 使用 DMA 方式读取温度传感器数据
 *
 * DMA 传输流程：
 *   1. 配置 I3C 传输描述符（读 2 字节从温度寄存器 0x00）
 *   2. 调用 I3C_MasterTransferEDMA() 启动非阻塞 DMA 传输
 *   3. SDK 内部：
 *      a) 将 subaddress（0x00）写入 I3C TX FIFO（通过 DMA CH0）
 *      b) 从 I3C RX FIFO 读取 2 字节到 g_i3cRxBuffer（通过 DMA CH1）
 *      c) 传输完成后触发中断，调用 i3c_master_edma_callback
 *   4. 主循环轮询 g_i3cTransferComplete 等待完成
 */
static status_t I3C_ReadTemperatureDMA(void)
{
    i3c_master_transfer_t masterXfer = {0};

    /*
     * 配置 I3C 传输描述符
     *
     * 对于 I3C SDR 读操作：
     *   主机先发送 1 字节的 subaddress（寄存器地址 0x00）
     *   然后从机返回 2 字节的寄存器数据
     *
     * I3C_MasterTransferEDMA 内部会自动处理：
     *   - CH0（TX）：将 subaddress 发送到 I3C TX FIFO
     *   - CH1（RX）：从 I3C RX FIFO 读取数据到 g_i3cRxBuffer
     */
    masterXfer.slaveAddress = SENSOR_ADDR;     /* 传感器动态地址 0x08 */
    masterXfer.direction = kI3C_Read;          /* 读操作 */
    masterXfer.busType = kI3C_TypeI3CSdr;      /* SDR 模式 */
    masterXfer.subaddress = 0x00U;             /* 温度寄存器地址 */
    masterXfer.subaddressSize = 1U;            /* 子地址长度 1 字节 */
    masterXfer.data = g_i3cRxBuffer;           /* 接收缓冲区 */
    masterXfer.dataSize = 2U;                  /* 温度值为 2 字节 */
    masterXfer.flags = kI3C_TransferDefaultFlag;

    /* 重置完成标志 */
    g_i3cTransferComplete = false;
    g_i3cTransferStatus = kStatus_Success;

    /*
     * 启动非阻塞 DMA 传输
     *
     * 这个函数会立即返回（不会阻塞等待传输完成）。
     * SDK 内部配置好 DMA TCD 后，DMA 引擎开始工作：
     *   1. I3C 硬件发送 START + 从机地址 + 写位
     *   2. I3C 硬件发送 subaddress（0x00）
     *   3. I3C 硬件发送 Repeated START + 从机地址 + 读位
     *   4. I3C 硬件从传感器读取 2 字节
     *   5. DMA 自动将读取的数据搬到 g_i3cRxBuffer
     *   6. 传输完成后产生中断 → 回调设置 g_i3cTransferComplete = true
     */
    return I3C_MasterTransferEDMA(EXAMPLE_MASTER, &g_i3cMasterEdmaHandle, &masterXfer);
}

/*
 * 使用 DMA 方式通过 UART 发送字符串
 *
 * DMA 传输流程：
 *   1. 配置 LPUART 传输描述符（源 = g_uartTxBuffer，大小 = strlen）
 *   2. 调用 LPUART_SendEDMA() 启动非阻塞 DMA 发送
 *   3. DMA CH2 将数据从 g_uartTxBuffer 逐字节搬到 LPUART0 TX 寄存器
 *   4. 传输完成后触发中断 → 回调设置 g_uartTransferComplete = true
 */
static status_t UART_SendStringDMA(const char *str)
{
    lpuart_transfer_t uartXfer;

    /* 重置完成标志 */
    g_uartTransferComplete = false;
    g_uartTransferStatus = kStatus_Success;

    /*
     * 清除 UART 错误标志（OR/FE/PF/NF）
     * RX 引脚悬空可能引入噪声导致帧错误/溢出，这些错误标志会触发中断
     * 并使 LPUART_TransferEdmaHandleIRQ 进入错误处理路径，错误地重置 txState
     */
    LPUART0->STAT |= LPUART_STAT_OR_MASK | LPUART_STAT_FE_MASK
                     | LPUART_STAT_PF_MASK | LPUART_STAT_NF_MASK;

    /* 配置 LPUART DMA 传输 */
    uartXfer.data = (uint8_t *)str;              /* 源地址：要发送的字符串 */
    uartXfer.dataSize = strlen(str);             /* 数据大小 */

    /*
     * 启动非阻塞 DMA 发送
     *
     * LPUART_SendEDMA 内部流程：
     *   1. 检查 UART 发送器是否空闲
     *   2. 配置 DMA CH2 的 TCD：
     *      SADDR = str 的地址（源地址，每次传输后递增）
     *      DADDR = LPUART0->DATA 寄存器地址（目的地址，固定不变）
     *      NBYTES = dataSize
     *   3. 使能 DMA 通道的硬件请求（DMA_EnableChannelRequest）
     *   4. 当 UART TX 寄存器为空时，UART 发出 DMA 请求
     *   5. DMA 响应请求，将 1 字节数据从内存搬到 UART TX 寄存器
     *   6. 重复步骤 4-5 直到所有数据发送完毕
     *   7. DMA 通道中断 → EDMA_HandleIRQ → lpuart_edma_callback
     */
    return LPUART_SendEDMA(LPUART0, &g_uartEdmaHandle, &uartXfer);
}

/*!
 * @brief Main function
 */
int main(void)
{
    status_t result = kStatus_Success;
    i3c_master_config_t masterConfig;
    edma_config_t edmaConfig;
    p3t1755_config_t p3t1755Config;

    /* 初始化硬件（时钟、引脚、调试串口） */
    BOARD_InitHardware();

    PRINTF("MCUX SDK version: %s\r\n", MCUXSDK_VERSION_FULL_STR);
    PRINTF("\r\n=== FRDM-MCXA156 P3T1755 Temperature Sensor (DMA Mode) ===\r\n");
    PRINTF("This demo uses DMA to read I3C sensor and send data via UART.\r\n\r\n");

    /**************************************************************************
     * 步骤 1：初始化 I3C0 主机
     * 配置 I3C 波特率和工作模式，准备与 P3T1755 传感器通信
     **************************************************************************/
    PRINTF("[1] Initializing I3C0 master...\r\n");

    I3C_MasterGetDefaultConfig(&masterConfig);
    masterConfig.baudRate_Hz.i2cBaud = EXAMPLE_I2C_BAUDRATE;
    masterConfig.baudRate_Hz.i3cPushPullBaud = EXAMPLE_I3C_PP_BAUDRATE;
    masterConfig.baudRate_Hz.i3cOpenDrainBaud = EXAMPLE_I3C_OD_BAUDRATE;
    masterConfig.enableOpenDrainStop = false;
    masterConfig.disableTimeout = true;
    I3C_MasterInit(EXAMPLE_MASTER, &masterConfig, I3C_MASTER_CLOCK_FREQUENCY);

    /**************************************************************************
     * 步骤 2：初始化 EDMA（增强型直接内存访问控制器）
     *
     * EDMA_Init 内部会：
     *   - 打开 DMA0 外设时钟（CLOCK_EnableClock）
     *   - 从复位中释放 DMA0（RESET_ReleasePeripheralReset）
     *   - 配置全局 DMA 参数（调试模式、错误处理等）
     *
     * 注意：EDMA_Init 只初始化 DMA 控制器本身，不初始化具体通道。
     * 通道初始化在 EDMA_CreateHandle 中完成。
     **************************************************************************/
    PRINTF("[2] Initializing EDMA...\r\n");

    EDMA_GetDefaultConfig(&edmaConfig);
    EDMA_Init(EXAMPLE_DMA, &edmaConfig);

    /**************************************************************************
     * 步骤 3：创建 DMA 通道句柄并配置通道复用
     *
     * 每个 DMA 通道需要：
     *   a) EDMA_CreateHandle() — 创建句柄，初始化通道状态，注册到驱动
     *   b) EDMA_SetChannelMux() — 配置通道复用器，连接到指定外设
     *
     * 通道复用（Channel Mux）的作用：
     *   将 DMA 通道的请求信号连接到特定外设。
     *   例如：EDMA_SetChannelMux(DMA0, 2, kDma0RequestLPUART0Tx)
     *   表示 DMA CH2 连接到 LPUART0 的 TX（发送）请求信号。
     *   当 LPUART0 的 TX FIFO 有空位时，会自动触发 DMA CH2 发送数据。
     **************************************************************************/
    PRINTF("[3] Creating DMA channel handles...\r\n");
    PRINTF("    CH0 -> I3C0 TX (mux=%d)\r\n", I3C_TX_DMA_CHANNEL_MUX);
    PRINTF("    CH1 -> I3C0 RX (mux=%d)\r\n", I3C_RX_DMA_CHANNEL_MUX);
    PRINTF("    CH2 -> LPUART0 TX (mux=%d)\r\n", UART_TX_DMA_CHANNEL_MUX);

    /* 创建 I3C TX DMA 句柄（CH0） */
    EDMA_CreateHandle(&g_i3cTxEdmaHandle, EXAMPLE_DMA, I3C_TX_DMA_CHANNEL);
    EDMA_SetChannelMux(EXAMPLE_DMA, I3C_TX_DMA_CHANNEL, I3C_TX_DMA_CHANNEL_MUX);

    /* 创建 I3C RX DMA 句柄（CH1） */
    EDMA_CreateHandle(&g_i3cRxEdmaHandle, EXAMPLE_DMA, I3C_RX_DMA_CHANNEL);
    EDMA_SetChannelMux(EXAMPLE_DMA, I3C_RX_DMA_CHANNEL, I3C_RX_DMA_CHANNEL_MUX);

    /* 创建 UART TX DMA 句柄（CH2） */
    EDMA_CreateHandle(&g_uartTxEdmaHandle, EXAMPLE_DMA, UART_TX_DMA_CHANNEL);
    EDMA_SetChannelMux(EXAMPLE_DMA, UART_TX_DMA_CHANNEL, UART_TX_DMA_CHANNEL_MUX);

    /**************************************************************************
     * 步骤 4：创建 I3C EDMA 句柄
     *
     * I3C_MasterTransferCreateHandleEDMA 将底层 DMA 通道句柄绑定到 I3C 外设：
     *   - rxDmaHandle = &g_i3cRxEdmaHandle（CH1 用于接收数据）
     *   - txDmaHandle = &g_i3cTxEdmaHandle（CH0 用于发送命令）
     *
     * SDK 内部会将回调函数挂接到 DMA 通道，形成调用链：
     *   DMA 中断 → EDMA_HandleIRQ → I3C EDMA 内部回调 → 用户回调
     **************************************************************************/
    PRINTF("[4] Creating I3C EDMA transfer handle...\r\n");

    I3C_MasterTransferCreateHandleEDMA(EXAMPLE_MASTER, &g_i3cMasterEdmaHandle,
                                        &g_i3cCallback, NULL,
                                        &g_i3cRxEdmaHandle, &g_i3cTxEdmaHandle);

    /**************************************************************************
     * 步骤 5：创建 LPUART EDMA 句柄
     *
     * LPUART_TransferCreateHandleEDMA 将底层 DMA 通道句柄绑定到 UART 外设：
     *   - txEdmaHandle = &g_uartTxEdmaHandle（CH2 用于发送数据）
     *   - rxEdmaHandle = NULL（本例不需要 UART 接收）
     *
     * 注意：LPUART_SendEDMA 是非阻塞的 — 启动后立即返回。
     * 发送完成后通过 lpuart_edma_callback 通知。
     **************************************************************************/
    PRINTF("[5] Creating LPUART EDMA transfer handle...\r\n");

    LPUART_TransferCreateHandleEDMA(LPUART0, &g_uartEdmaHandle,
                                     lpuart_edma_callback, NULL,
                                     &g_uartTxEdmaHandle, NULL);

    /*
     * 确保 LPUART0 NVIC 中断已使能
     * LPUART_TransferCreateHandleEDMA 内部会调用 EnableIRQ(LPUART0_IRQn)，
     * 但二次确认可防止 SDK 版本差异导致的中断未使能问题。
     * LPUART0 中断用于 DMA 发送完成后的 TC（发送完成）中断处理：
     *   LPUART0_IRQHandler → LPUART0_DriverIRQHandler
     *   → LPUART_TransferEdmaHandleIRQ → 用户回调 → 清除 txState
     */
    NVIC_EnableIRQ(LPUART0_IRQn);

    /*
     * 清除 LPUART0 所有待处理的中断标志
     * LPUART_TransferCreateHandleEDMA 会禁用所有 LPUART 中断使能位，
     * 但硬件状态标志（STAT 寄存器）可能仍有残留（如 RX 引脚悬空导致的
     * 帧错误/溢出标志），这些残留标志会在 DMA 完成后触发错误处理路径。
     */
    LPUART0->STAT |= LPUART_STAT_OR_MASK | LPUART_STAT_FE_MASK
                     | LPUART_STAT_PF_MASK | LPUART_STAT_NF_MASK;

    /**************************************************************************
     * 步骤 6：为 P3T1755 传感器分配 I3C 动态地址
     *
     * I3C 协议要求主机先为从机分配动态地址，之后的数据传输使用动态地址。
     * 这里使用阻塞式传输（非 DMA），因为这是初始化步骤。
     **************************************************************************/
    PRINTF("[6] Assigning dynamic address to P3T1755...\r\n");

    result = p3t1755_set_dynamic_address();
    if (result != kStatus_Success)
    {
        PRINTF("ERROR: P3T1755 set dynamic address failed (code=%d)!\r\n", result);
        while (1) {}
    }

    /**************************************************************************
     * 步骤 7：初始化 P3T1755 传感器驱动
     *
     * P3T1755_Init 内部会：
     *   - 读取传感器配置寄存器
     *   - 根据需要修改配置并写回
     *   - 设置转换模式等参数
     *
     * 这里的 I3C_WriteSensor 和 I3C_ReadSensor 使用阻塞传输。
     * 初始化完成后，主循环中的温度读取将使用 DMA 方式。
     **************************************************************************/
    PRINTF("[7] Initializing P3T1755 sensor driver...\r\n");

    p3t1755Config.writeTransfer = I3C_WriteSensor;
    p3t1755Config.readTransfer = I3C_ReadSensor;
    p3t1755Config.sensorAddress = SENSOR_ADDR;
    p3t1755Config.oneshotMode = false;
    result = P3T1755_Init(&g_p3t1755Handle, &p3t1755Config);
    if (result != kStatus_Success)
    {
        PRINTF("ERROR: P3T1755 init failed (code=%d)!\r\n", result);
        while (1) {}
    }

    PRINTF("Initialization complete. Starting DMA temperature reading...\r\n\r\n");

    /**************************************************************************
     * 主循环：使用 DMA 方式循环读取温度并通过 UART 发送
     *
     * DMA 工作流程（每次循环）：
     *   1. 调用 I3C_ReadTemperatureDMA() 启动 I3C DMA 读温度
     *   2. 轮询等待 g_i3cTransferComplete（DMA 传输完成）
     *   3. 解析 2 字节温度数据为浮点数
     *   4. 格式化为字符串写入 g_uartTxBuffer
     *   5. 调用 UART_SendStringDMA() 启动 UART DMA 发送
     *   6. 轮询等待 g_uartTransferComplete（DMA 发送完成）
     *   7. 延时 1 秒，进入下一轮循环
     *
     * 虽然使用了 DMA，但因为数据量很小（2 字节读取 + ~30 字节发送），
     * 我们仍然需要轮询等待完成标志。真正的优势在于传输过程中 CPU 可以做
     * 其他工作（本例中用不到，但概念上 CPU 是空闲的）。
     **************************************************************************/
    while (1)
    {
        int32_t rawTemp;

        /*
         * DMA 传输阶段 1：通过 I3C DMA 读取温度寄存器
         *
         * I3C 主机的 DMA 读取流程：
         *   a) CPU 调用 I3C_MasterTransferEDMA()
         *   b) SDK 配置 I3C 硬件和 DMA TCD
         *   c) I3C 硬件在总线上发送 START + 从机地址 + 写
         *   d) I3C 硬件发送子地址 0x00（温度寄存器）
         *   e) I3C 硬件发送 Repeated START + 从机地址 + 读
         *   f) 传感器返回 2 字节温度数据
         *   g) DMA CH1 自动将接收到的数据搬到 g_i3cRxBuffer
         *   h) 传输完成 → 中断 → 回调设置标志
         */
        result = I3C_ReadTemperatureDMA();
        if (result != kStatus_Success)
        {
            PRINTF("ERROR: I3C DMA transfer start failed (code=%d)!\r\n", result);
            SDK_DelayAtLeastUs(1000000, CLOCK_GetCoreSysClkFreq());
            continue;
        }

        /* 等待 I3C DMA 传输完成 */
        {
            uint32_t timeout = 0;
            while (!g_i3cTransferComplete)
            {
                timeout++;
                if (timeout > DMA_TIMEOUT)
                {
                    PRINTF("ERROR: I3C DMA transfer timeout!\r\n");
                    break;
                }
            }
        }

        if (g_i3cTransferStatus != kStatus_Success)
        {
            PRINTF("ERROR: I3C DMA transfer failed (code=%d)!\r\n", g_i3cTransferStatus);
            SDK_DelayAtLeastUs(1000000, CLOCK_GetCoreSysClkFreq());
            continue;
        }

        /*
         * 解析温度数据
         *
         * P3T1755 温度寄存器格式（12 位，二进制补码）：
         *   Byte 0: D11 D10 D9 D8 D7 D6 D5 D4
         *   Byte 1: D3  D2  D1  D0  0  0  0  0
         *
         * 温度值 = 原始值 * 0.0625 °C
         * 例如：原始值 = 0x0130 (19.0°C)
         *       原始值 = 0x7D0 (125°C)
         *
         * 注意：大端/小端处理 — P3T1755 先发送高字节
         */
        rawTemp = ((int16_t)((g_i3cRxBuffer[0] << 8) | g_i3cRxBuffer[1])) >> 4;

        /*
         * 温度格式化（手动整数/小数分解）
         *
         * newlib-nano 的 snprintf 不支持 %f 浮点格式化（为节省代码空间）。
         * 即使定义了 PRINTF_FLOAT_ENABLE=1，浮点支持仅对 SDK 的 PRINTF
         * 有效，不影响 C 标准库的 snprintf。因此需要手动将温度分解为
         * 整数部分和小数部分，再用 %d 格式化输出。
         *
         * P3T1755 分辨率 0.0625°C = 1/16 °C：
         *   temperature = rawTemp * 0.0625 = rawTemp / 16.0
         *
         * 使用定点运算避免浮点数：
         *   整数部分 = rawTemp >> 4 (除以 16)
         *   小数部分 = (rawTemp & 0xF) * 625  (余数 * 0.0625 * 10000)
         *   简化：小数 = (rawTemp & 0xF) * 10000 / 16 = (rawTemp & 0xF) * 625
         */
        {
            int32_t raw = rawTemp; /* 12 位有符号温度值 */
            int32_t intPart, fracPart;

            if (raw < 0)
            {
                /* 处理负数：取绝对值分别处理 */
                raw = -raw;
                intPart = raw >> 4;
                fracPart = (int32_t)((raw & 0xF) * 625UL);
                snprintf(g_uartTxBuffer, UART_TX_BUFFER_SIZE,
                         "Temperature: -%ld.%02ld C\r\n", intPart, fracPart / 10);
            }
            else
            {
                intPart = raw >> 4;
                fracPart = (int32_t)((raw & 0xF) * 625UL);
                snprintf(g_uartTxBuffer, UART_TX_BUFFER_SIZE,
                         "Temperature: %ld.%02ld C\r\n", intPart, fracPart / 10);
            }
        }

        /*
         * DMA 传输阶段 2：通过 UART DMA 发送温度字符串
         *
         * LPUART DMA 发送流程：
         *   a) CPU 将格式化字符串写入 g_uartTxBuffer（这是唯一需要 CPU 的步骤）
         *   b) CPU 调用 LPUART_SendEDMA()
         *   c) SDK 配置 DMA CH2 的 TCD
         *   d) 每当 UART TX 寄存器为空，DMA 自动搬移下一个字节
         *   e) 所有数据发送完毕 → 中断 → 回调设置标志
         *
         * 注意：g_uartTxBuffer 在 DMA 传输期间不能被修改！
         *       因为 DMA 是异步的，可能还没读完数据就被新数据覆盖。
         */

        result = UART_SendStringDMA(g_uartTxBuffer);
        if (result != kStatus_Success)
        {
            PRINTF("ERROR: UART DMA send start failed (code=%d)!\r\n", result);
            SDK_DelayAtLeastUs(1000000, CLOCK_GetCoreSysClkFreq());
            continue;
        }

        /* 等待 UART DMA 发送完成 */
        {
            uint32_t timeout = 0;
            while (!g_uartTransferComplete)
            {
                timeout++;
                if (timeout > DMA_TIMEOUT)
                {
                    PRINTF("ERROR: UART DMA transfer timeout!\r\n");
                    break;
                }
            }
        }

        /*
         * 检查 UART DMA 发送状态
         *
         * LPUART_TransferEdmaHandleIRQ 在 TX 完成时通过回调传递的状态是
         * kStatus_LPUART_TxIdle (1302)，而不是 kStatus_Success (0)。
         * 这是因为 LPUART EDMA 驱动使用其自身定义的状态码（status group 13）
         * 来报告"发送器已空闲 = 传输完成"。因此需要同时接受这两个成功标志。
         */
        if (g_uartTransferStatus != kStatus_Success &&
            g_uartTransferStatus != kStatus_LPUART_TxIdle)
        {
            PRINTF("ERROR: UART DMA send failed (code=%d)!\r\n", g_uartTransferStatus);
        }

        /* 延时 1 秒后再进行下一次读取 */
        SDK_DelayAtLeastUs(1000000, CLOCK_GetCoreSysClkFreq());
    }
}
