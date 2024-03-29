/***********************************************************************************************************************
 * This file was generated by the MCUXpresso Config Tools. Any manual edits made to this file
 * will be overwritten if the respective MCUXpresso Config Tools is used to update this file.
 **********************************************************************************************************************/

/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!GlobalInfo
product: Peripherals v11.0
processor: LPC55S69
package_id: LPC55S69JBD100
mcu_data: ksdk2_0
processor_version: 12.0.0
functionalGroups:
- name: BOARD_InitPeripherals
  UUID: f9549bcc-6b71-4905-b031-fa7e5c4eaeac
  called_from_default_init: true
  selectedCore: cm33_core0
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/

/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
component:
- type: 'system'
- type_id: 'system_54b53072540eeeb8f8e9343e71f28176'
- global_system_definitions:
  - user_definitions: ''
  - user_includes: ''
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/

/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
component:
- type: 'uart_cmsis_common'
- type_id: 'uart_cmsis_common_9cb8e302497aa696fdbb5a4fd622c2a8'
- global_USART_CMSIS_common:
  - quick_selection: 'default'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/

/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
component:
- type: 'gpio_adapter_common'
- type_id: 'gpio_adapter_common_57579b9ac814fe26bf95df0a384c36b6'
- global_gpio_adapter_common:
  - quick_selection: 'default'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */

/***********************************************************************************************************************
 * Included files
 **********************************************************************************************************************/
#include "peripherals.h"

/***********************************************************************************************************************
 * BOARD_InitPeripherals functional group
 **********************************************************************************************************************/
/***********************************************************************************************************************
 * NVIC initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'NVIC'
- type: 'nvic'
- mode: 'general'
- custom_name_enabled: 'false'
- type_id: 'nvic_57b5eef3774cc60acaede6f5b8bddc67'
- functional_group: 'BOARD_InitPeripherals'
- peripheral: 'NVIC'
- config_sets:
  - nvic:
    - interrupt_table:
      - 0: []
      - 1: []
    - interrupts: []
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */

/* Empty initialization function (commented out)
static void NVIC_init(void) {
} */

/***********************************************************************************************************************
 * CTIMER0 initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'CTIMER0'
- type: 'ctimer'
- mode: 'Capture_Match'
- custom_name_enabled: 'false'
- type_id: 'ctimer_c8b90232d8b6318ba1dac2cf08fb5f4a'
- functional_group: 'BOARD_InitPeripherals'
- peripheral: 'CTIMER0'
- config_sets:
  - fsl_ctimer:
    - ctimerConfig:
      - mode: 'kCTIMER_TimerMode'
      - clockSource: 'FunctionClock'
      - clockSourceFreq: 'BOARD_BootClockFROHF96M'
      - timerPrescaler: '250Hz'
    - EnableTimerInInit: 'false'
    - matchChannels:
      - 0:
        - matchChannelPrefixId: 'Match_0'
        - matchChannel: 'kCTIMER_Match_0'
        - matchValueStr: '10Hz'
        - enableCounterReset: 'true'
        - enableCounterStop: 'false'
        - outControl: 'kCTIMER_Output_NoAction'
        - outPinInitValue: 'low'
        - enableInterrupt: 'true'
    - interruptCallbackConfig:
      - interrupt:
        - IRQn: 'CTIMER0_IRQn'
        - enable_priority: 'false'
        - priority: '0'
      - callback: 'kCTIMER_SingleCallback'
      - singleCallback: 'Timer_IRQ'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */
const ctimer_config_t CTIMER0_config = {
  .mode = kCTIMER_TimerMode,
  .input = kCTIMER_Capture_0,
  .prescale = 383999
};
const ctimer_match_config_t CTIMER0_Match_0_config = {
  .matchValue = 24,
  .enableCounterReset = true,
  .enableCounterStop = false,
  .outControl = kCTIMER_Output_NoAction,
  .outPinInitState = false,
  .enableInterrupt = true
};
/* Single callback functions definition */
ctimer_callback_t CTIMER0_callback[] = {Timer_IRQ};

static void CTIMER0_init(void) {
  /* CTIMER0 peripheral initialization */
  CTIMER_Init(CTIMER0_PERIPHERAL, &CTIMER0_config);
  /* Match channel 0 of CTIMER0 peripheral initialization */
  CTIMER_SetupMatch(CTIMER0_PERIPHERAL, CTIMER0_MATCH_0_CHANNEL, &CTIMER0_Match_0_config);
  CTIMER_RegisterCallBack(CTIMER0_PERIPHERAL, CTIMER0_callback, kCTIMER_SingleCallback);
}

/***********************************************************************************************************************
 * CTIMER2 initialization code
 **********************************************************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
instance:
- name: 'CTIMER2'
- type: 'ctimer'
- mode: 'Capture_Match'
- custom_name_enabled: 'false'
- type_id: 'ctimer_c8b90232d8b6318ba1dac2cf08fb5f4a'
- functional_group: 'BOARD_InitPeripherals'
- peripheral: 'CTIMER2'
- config_sets:
  - fsl_ctimer:
    - ctimerConfig:
      - mode: 'kCTIMER_TimerMode'
      - clockSource: 'FunctionClock'
      - clockSourceFreq: 'BOARD_BootClockFROHF96M'
      - timerPrescaler: '1000Hz'
    - EnableTimerInInit: 'false'
    - matchChannels:
      - 0:
        - matchChannelPrefixId: 'Match_1'
        - matchChannel: 'kCTIMER_Match_1'
        - matchValueStr: '10Hz'
        - enableCounterReset: 'true'
        - enableCounterStop: 'false'
        - outControl: 'kCTIMER_Output_NoAction'
        - outPinInitValue: 'low'
        - enableInterrupt: 'true'
    - interruptCallbackConfig:
      - interrupt:
        - IRQn: 'CTIMER2_IRQn'
        - enable_priority: 'false'
        - priority: '0'
      - callback: 'kCTIMER_SingleCallback'
      - singleCallback: 'Timer2_IRQ'
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */
const ctimer_config_t CTIMER2_config = {
  .mode = kCTIMER_TimerMode,
  .input = kCTIMER_Capture_0,
  .prescale = 95999
};
const ctimer_match_config_t CTIMER2_Match_1_config = {
  .matchValue = 99,
  .enableCounterReset = true,
  .enableCounterStop = false,
  .outControl = kCTIMER_Output_NoAction,
  .outPinInitState = false,
  .enableInterrupt = true
};
/* Single callback functions definition */
ctimer_callback_t CTIMER2_callback[] = {Timer2_IRQ};

static void CTIMER2_init(void) {
  /* CTIMER2 peripheral initialization */
  CTIMER_Init(CTIMER2_PERIPHERAL, &CTIMER2_config);
  /* Match channel 1 of CTIMER2 peripheral initialization */
  CTIMER_SetupMatch(CTIMER2_PERIPHERAL, CTIMER2_MATCH_1_CHANNEL, &CTIMER2_Match_1_config);
  CTIMER_RegisterCallBack(CTIMER2_PERIPHERAL, CTIMER2_callback, kCTIMER_SingleCallback);
}

/***********************************************************************************************************************
 * Initialization functions
 **********************************************************************************************************************/
void BOARD_InitPeripherals(void)
{
  /* Initialize components */
  CTIMER0_init();
  CTIMER2_init();
}

/***********************************************************************************************************************
 * BOARD_InitBootPeripherals function
 **********************************************************************************************************************/
void BOARD_InitBootPeripherals(void)
{
  BOARD_InitPeripherals();
}
