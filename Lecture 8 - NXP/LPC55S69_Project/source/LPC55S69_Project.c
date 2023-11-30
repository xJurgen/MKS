/*
 * Copyright 2016-2023 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file    LPC55S69_Project.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include <stddef.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "LPC55S69_cm33_core0.h"
#include "fsl_debug_console.h"
/* TODO: insert other include files here. */

/* TODO: insert other definitions and declarations here. */

bool const_strcmp(const void *a, const void *b, size_t len) {
	const char *ca = a, *cb = b;
	bool res = true;

	for (size_t i = 0; i < len; i++) {
		res &= ca[i] == cb[i];
	}

	return res;
}

bool safe_strcmp(const char *str1, const char *str2, size_t num)
{
	uint16_t miss = 0;
	uint16_t hit = 0;

	/*if (str1 == NULL && str2 == NULL) return 0;
	if (str1 == NULL) return -1;
	if (str2 == NULL) return 1;
	if (strlen(str1) != strlen(str2)) return -2;*/

	for (int i = 0; i < num; i++) {
		if (str1[i] != str2[i]) miss++;
		else hit++;
	}

	if (miss) return false;
	else return true;
}

/*
 * @brief   Application entry point.
 */
int main(void) {

    /* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
#ifndef BOARD_INIT_DEBUG_CONSOLE_PERIPHERAL
    /* Init FSL debug console. */
    BOARD_InitDebugConsole();
#endif

    uint32_t DWT1, DWT2;
    uint8_t pass_status = 0;

    // Look into Lennert Wouters
    /* Enter an infinite loop, just incrementing a counter. */
    while(1) {
    	char password_stored[20] = "12345678";
        char input[20];
        PRINTF("\r\nEnter password: ");
        SCANF("%s", input);
    	PRINTF("\r\ninput: %s", input);

        DWT1=DWT->CYCCNT;
        pass_status = safe_strcmp(input, password_stored, strlen(password_stored));
        //pass_status = const_strcmp(input, password_stored, strlen(password_stored));
        DWT2=DWT->CYCCNT;

        PRINTF("\r\nCycles in function: %d\r\nPass status: %s", DWT2-DWT1, (pass_status == true)? "CORRECT" : "INCORRECT" );
    }
    return 0 ;
}
