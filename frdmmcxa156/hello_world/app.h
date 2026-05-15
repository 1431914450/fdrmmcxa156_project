/*
 * Copyright 2024 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _APP_H_
#define _APP_H_

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define EXAMPLE_MASTER             I3C0
#define I3C_MASTER_CLOCK_FREQUENCY CLOCK_GetI3CFClkFreq()
#define SENSOR_SLAVE_ADDR          0x48U

/* DMA */
#define EXAMPLE_DMA                 DMA0
#define I3C_TX_DMA_CHANNEL          (0U)
#define I3C_RX_DMA_CHANNEL          (1U)
#define UART_TX_DMA_CHANNEL         (2U)
#define I3C_TX_DMA_CHANNEL_MUX      kDma0RequestMuxI3c0Tx
#define I3C_RX_DMA_CHANNEL_MUX      kDma0RequestMuxI3c0Rx
#define UART_TX_DMA_CHANNEL_MUX     kDma0RequestLPUART0Tx

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void BOARD_InitHardware(void);

#endif /* _APP_H_ */
