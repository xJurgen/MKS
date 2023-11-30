/*
 * Copyright (c) 2013 - 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "board.h"

#include "pin_mux.h"
#include "clock_config.h"

#include "fsl_ctimer.h"
#include "peripherals.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/


/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void Timer_IRQ(uint32_t flags)
{
	static uint8_t i = 0;
	i++;
	if (i >= 4) i = 0;

	char object[4][10] = {
			"kokos",
			"ananas",
			"mango",
			"penis"
	};

	PRINTF("I like %s.\r\n", object[i]);
}

void Timer2_IRQ(uint32_t flags)
{
	static uint8_t i = 0;
	i++;
	if (i >= 6) i = 0;

	if (i == 0 || i == 1)
		LED_BLUE_TOGGLE();

	else if (i == 2 || i == 3)
		LED_GREEN_TOGGLE();

	else if (i == 4 || i == 5)
		LED_RED_TOGGLE();
}

/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief Main function
 */
int main(void)
{
    char ch;

    /* Init board hardware. */
    /* attach main clock divide to FLEXCOMM0 (debug console) */
    CLOCK_AttachClk(BOARD_DEBUG_UART_CLK_ATTACH);

    BOARD_InitPins();
    BOARD_BootClockFROHF96M();
    BOARD_InitDebugConsole();
    BOARD_InitBootPeripherals();

    CTIMER_StartTimer(CTIMER0_PERIPHERAL);
    CTIMER_StartTimer(CTIMER2_PERIPHERAL);

    PRINTF("hello world.\r\n");

    while (1)
    {
        ch = GETCHAR();
        PUTCHAR(ch);
    }
}
