/*
 * Copyright 2024 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_common.h"
#include "fsl_port.h"
#include "fsl_clock.h"
#include "fsl_reset.h"
#include "pin_mux.h"

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitBootPins
 * Description   : Calls initialization functions.
 *
 * END ****************************************************************************************************************/
void BOARD_InitBootPins(void)
{
    BOARD_InitPins();
}

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitPins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
void BOARD_InitPins(void)
{
    /* Enable PORT0 and PORT1 clocks */
    CLOCK_EnableClock(kCLOCK_GatePORT0);
    CLOCK_EnableClock(kCLOCK_GatePORT1);

    /* Release I3C0 from reset */
    RESET_ReleasePeripheralReset(kI3C0_RST_SHIFT_RSTn);
    /* Release PORT0 from reset */
    RESET_ReleasePeripheralReset(kPORT0_RST_SHIFT_RSTn);
    /* Release LPUART0 from reset */
    RESET_ReleasePeripheralReset(kLPUART0_RST_SHIFT_RSTn);
    /* Release PORT1 from reset */
    RESET_ReleasePeripheralReset(kPORT1_RST_SHIFT_RSTn);

    /* PORT0_16 (pin 83): I3C0_SDA */
    const port_pin_config_t port0_16_config = {
        kPORT_PullDisable,
        kPORT_LowPullResistor,
        kPORT_FastSlewRate,
        kPORT_PassiveFilterDisable,
        kPORT_OpenDrainDisable,
        kPORT_LowDriveStrength,
        kPORT_NormalDriveStrength,
        kPORT_MuxAlt10,
        kPORT_InputBufferEnable,
        kPORT_InputNormal,
        kPORT_UnlockRegister
    };
    PORT_SetPinConfig(PORT0, 16U, &port0_16_config);

    /* PORT0_17 (pin 84): I3C0_SCL */
    const port_pin_config_t port0_17_config = {
        kPORT_PullDisable,
        kPORT_LowPullResistor,
        kPORT_FastSlewRate,
        kPORT_PassiveFilterDisable,
        kPORT_OpenDrainDisable,
        kPORT_LowDriveStrength,
        kPORT_NormalDriveStrength,
        kPORT_MuxAlt10,
        kPORT_InputBufferEnable,
        kPORT_InputNormal,
        kPORT_UnlockRegister
    };
    PORT_SetPinConfig(PORT0, 17U, &port0_17_config);

    /* PORT0_2 (pin 78): LPUART0_RXD */
    const port_pin_config_t port0_2_config = {
        kPORT_PullUp,
        kPORT_LowPullResistor,
        kPORT_FastSlewRate,
        kPORT_PassiveFilterDisable,
        kPORT_OpenDrainDisable,
        kPORT_LowDriveStrength,
        kPORT_NormalDriveStrength,
        kPORT_MuxAlt2,
        kPORT_InputBufferEnable,
        kPORT_InputNormal,
        kPORT_UnlockRegister
    };
    PORT_SetPinConfig(PORT0, 2U, &port0_2_config);

    /* PORT0_3 (pin 79): LPUART0_TXD */
    const port_pin_config_t port0_3_config = {
        kPORT_PullUp,
        kPORT_LowPullResistor,
        kPORT_FastSlewRate,
        kPORT_PassiveFilterDisable,
        kPORT_OpenDrainDisable,
        kPORT_LowDriveStrength,
        kPORT_NormalDriveStrength,
        kPORT_MuxAlt2,
        kPORT_InputBufferEnable,
        kPORT_InputNormal,
        kPORT_UnlockRegister
    };
    PORT_SetPinConfig(PORT0, 3U, &port0_3_config);

    /* PORT1_11 (pin 4): I3C0_PUR */
    const port_pin_config_t port1_11_config = {
        kPORT_PullDisable,
        kPORT_LowPullResistor,
        kPORT_FastSlewRate,
        kPORT_PassiveFilterDisable,
        kPORT_OpenDrainDisable,
        kPORT_LowDriveStrength,
        kPORT_NormalDriveStrength,
        kPORT_MuxAlt10,
        kPORT_InputBufferEnable,
        kPORT_InputNormal,
        kPORT_UnlockRegister
    };
    PORT_SetPinConfig(PORT1, 11U, &port1_11_config);
}
