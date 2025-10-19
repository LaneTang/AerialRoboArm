/*
 * Copyright (c) 2023, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ============ ti_msp_dl_config.c =============
 *  Configured MSPM0 DriverLib module definitions
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G350X
 *  by the SysConfig tool.
 */

#include "ti_msp_dl_config.h"

DL_TimerG_backupConfig gPWM_MotorBackup;
DL_TimerA_backupConfig gTIMER_computePIDBackup;
DL_TimerA_backupConfig gTIMER_DebugBackup;

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform any initialization needed before using any board APIs
 */
SYSCONFIG_WEAK void SYSCFG_DL_init(void)
{
    SYSCFG_DL_initPower();
    SYSCFG_DL_GPIO_init();
    /* Module-Specific Initializations*/
    SYSCFG_DL_SYSCTL_init();
    SYSCFG_DL_PWM_Motor_init();
    SYSCFG_DL_TIMER_computePID_init();
    SYSCFG_DL_TIMER_Debug_init();
    SYSCFG_DL_UART_0_init();
    SYSCFG_DL_UART_JY60_init();
    /* Ensure backup structures have no valid state */
	gPWM_MotorBackup.backupRdy 	= false;
	gTIMER_computePIDBackup.backupRdy 	= false;
	gTIMER_DebugBackup.backupRdy 	= false;


}
/*
 * User should take care to save and restore register configuration in application.
 * See Retention Configuration section for more details.
 */
SYSCONFIG_WEAK bool SYSCFG_DL_saveConfiguration(void)
{
    bool retStatus = true;

	retStatus &= DL_TimerG_saveConfiguration(PWM_Motor_INST, &gPWM_MotorBackup);
	retStatus &= DL_TimerA_saveConfiguration(TIMER_computePID_INST, &gTIMER_computePIDBackup);
	retStatus &= DL_TimerA_saveConfiguration(TIMER_Debug_INST, &gTIMER_DebugBackup);

    return retStatus;
}


SYSCONFIG_WEAK bool SYSCFG_DL_restoreConfiguration(void)
{
    bool retStatus = true;

	retStatus &= DL_TimerG_restoreConfiguration(PWM_Motor_INST, &gPWM_MotorBackup, false);
	retStatus &= DL_TimerA_restoreConfiguration(TIMER_computePID_INST, &gTIMER_computePIDBackup, false);
	retStatus &= DL_TimerA_restoreConfiguration(TIMER_Debug_INST, &gTIMER_DebugBackup, false);

    return retStatus;
}

SYSCONFIG_WEAK void SYSCFG_DL_initPower(void)
{
    DL_GPIO_reset(GPIOA);
    DL_GPIO_reset(GPIOB);
    DL_TimerG_reset(PWM_Motor_INST);
    DL_TimerA_reset(TIMER_computePID_INST);
    DL_TimerA_reset(TIMER_Debug_INST);
    DL_UART_Main_reset(UART_0_INST);
    DL_UART_Main_reset(UART_JY60_INST);

    DL_GPIO_enablePower(GPIOA);
    DL_GPIO_enablePower(GPIOB);
    DL_TimerG_enablePower(PWM_Motor_INST);
    DL_TimerA_enablePower(TIMER_computePID_INST);
    DL_TimerA_enablePower(TIMER_Debug_INST);
    DL_UART_Main_enablePower(UART_0_INST);
    DL_UART_Main_enablePower(UART_JY60_INST);
    delay_cycles(POWER_STARTUP_DELAY);
}

SYSCONFIG_WEAK void SYSCFG_DL_GPIO_init(void)
{
    const uint8_t unusedPinIndexes[] =
    {
        IOMUX_PINCM31, IOMUX_PINCM33, IOMUX_PINCM36, IOMUX_PINCM46,
        IOMUX_PINCM48, IOMUX_PINCM50, IOMUX_PINCM51, IOMUX_PINCM53,
        IOMUX_PINCM54, IOMUX_PINCM56, IOMUX_PINCM57, IOMUX_PINCM58,
        IOMUX_PINCM59, IOMUX_PINCM2, IOMUX_PINCM4, IOMUX_PINCM5,
        IOMUX_PINCM7, IOMUX_PINCM8, IOMUX_PINCM9, IOMUX_PINCM10,
        IOMUX_PINCM11, IOMUX_PINCM14, IOMUX_PINCM15, IOMUX_PINCM16,
        IOMUX_PINCM17, IOMUX_PINCM18, IOMUX_PINCM21, IOMUX_PINCM27,
        IOMUX_PINCM28
    };

    for(int i = 0; i < sizeof(unusedPinIndexes)/sizeof(unusedPinIndexes[0]); i++)
    {
        DL_GPIO_initDigitalOutput(unusedPinIndexes[i]);
    }

    DL_GPIO_clearPins(GPIOA,
        (DL_GPIO_PIN_14 | DL_GPIO_PIN_21 | DL_GPIO_PIN_23 | DL_GPIO_PIN_24 |
        DL_GPIO_PIN_26 | DL_GPIO_PIN_1 | DL_GPIO_PIN_29 | DL_GPIO_PIN_30 |
        DL_GPIO_PIN_2 | DL_GPIO_PIN_3 | DL_GPIO_PIN_4 | DL_GPIO_PIN_5 |
        DL_GPIO_PIN_6 | DL_GPIO_PIN_7 | DL_GPIO_PIN_10));
    DL_GPIO_enableOutput(GPIOA,
        (DL_GPIO_PIN_14 | DL_GPIO_PIN_21 | DL_GPIO_PIN_23 | DL_GPIO_PIN_24 |
        DL_GPIO_PIN_26 | DL_GPIO_PIN_1 | DL_GPIO_PIN_29 | DL_GPIO_PIN_30 |
        DL_GPIO_PIN_2 | DL_GPIO_PIN_3 | DL_GPIO_PIN_4 | DL_GPIO_PIN_5 |
        DL_GPIO_PIN_6 | DL_GPIO_PIN_7 | DL_GPIO_PIN_10));
    DL_GPIO_clearPins(GPIOB,
        (DL_GPIO_PIN_14 | DL_GPIO_PIN_16 | DL_GPIO_PIN_20 | DL_GPIO_PIN_22 |
        DL_GPIO_PIN_23 | DL_GPIO_PIN_25 | DL_GPIO_PIN_26 | DL_GPIO_PIN_27 |
        DL_GPIO_PIN_2 | DL_GPIO_PIN_3 | DL_GPIO_PIN_4 | DL_GPIO_PIN_5 |
        DL_GPIO_PIN_10 | DL_GPIO_PIN_11));
    DL_GPIO_enableOutput(GPIOB,
        (DL_GPIO_PIN_14 | DL_GPIO_PIN_16 | DL_GPIO_PIN_20 | DL_GPIO_PIN_22 |
        DL_GPIO_PIN_23 | DL_GPIO_PIN_25 | DL_GPIO_PIN_26 | DL_GPIO_PIN_27 |
        DL_GPIO_PIN_2 | DL_GPIO_PIN_3 | DL_GPIO_PIN_4 | DL_GPIO_PIN_5 |
        DL_GPIO_PIN_10 | DL_GPIO_PIN_11));

    DL_GPIO_initPeripheralOutputFunction(GPIO_PWM_Motor_C0_IOMUX,GPIO_PWM_Motor_C0_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_PWM_Motor_C0_PORT, GPIO_PWM_Motor_C0_PIN);
    DL_GPIO_initPeripheralOutputFunction(GPIO_PWM_Motor_C1_IOMUX,GPIO_PWM_Motor_C1_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_PWM_Motor_C1_PORT, GPIO_PWM_Motor_C1_PIN);

    DL_GPIO_initPeripheralOutputFunction(
        GPIO_UART_0_IOMUX_TX, GPIO_UART_0_IOMUX_TX_FUNC);
    DL_GPIO_initPeripheralInputFunction(
        GPIO_UART_0_IOMUX_RX, GPIO_UART_0_IOMUX_RX_FUNC);
    DL_GPIO_initPeripheralOutputFunction(
        GPIO_UART_JY60_IOMUX_TX, GPIO_UART_JY60_IOMUX_TX_FUNC);
    DL_GPIO_initPeripheralInputFunction(
        GPIO_UART_JY60_IOMUX_RX, GPIO_UART_JY60_IOMUX_RX_FUNC);

    DL_GPIO_initDigitalOutput(GPIO_Flag_USER_LED_1_IOMUX);

    DL_GPIO_initDigitalOutputFeatures(GPIO_Flag_PROMPT_Buzzer_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_DRIVE_STRENGTH_LOW, DL_GPIO_HIZ_DISABLE);

    DL_GPIO_initDigitalOutputFeatures(GPIO_Flag_PROMPT_LED_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_DOWN,
		 DL_GPIO_DRIVE_STRENGTH_LOW, DL_GPIO_HIZ_DISABLE);

    DL_GPIO_initDigitalInputFeatures(GPIO_Flag_PIN_ResetYaw_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalOutput(GPIO_OLED_PIN_SCL_IOMUX);

    DL_GPIO_initDigitalOutput(GPIO_OLED_PIN_SDA_IOMUX);

    DL_GPIO_initDigitalOutput(GPIO_Motor_PIN_1IN1_IOMUX);

    DL_GPIO_initDigitalOutput(GPIO_Motor_PIN_1IN2_IOMUX);

    DL_GPIO_initDigitalOutput(GPIO_Motor_PIN_2IN1_IOMUX);

    DL_GPIO_initDigitalOutput(GPIO_Motor_PIN_2IN2_IOMUX);

    DL_GPIO_initDigitalInput(GPIO_Encoder_PIN_1A_IOMUX);

    DL_GPIO_initDigitalInput(GPIO_Encoder_PIN_1B_IOMUX);

    DL_GPIO_initDigitalInput(GPIO_SensorTracking_PIN_1_IOMUX);

    DL_GPIO_initDigitalInput(GPIO_SensorTracking_PIN_2_IOMUX);

    DL_GPIO_initDigitalInput(GPIO_SensorTracking_PIN_3_IOMUX);

    DL_GPIO_initDigitalInput(GPIO_SensorTracking_PIN_4_IOMUX);

    DL_GPIO_initDigitalInput(GPIO_SensorTracking_PIN_5_IOMUX);

    DL_GPIO_initDigitalInput(GPIO_SensorTracking_PIN_6_IOMUX);

    DL_GPIO_initDigitalInput(GPIO_SensorTracking_PIN_7_IOMUX);

    DL_GPIO_initDigitalInputFeatures(GPIO_Questions_PIN_Q1_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(GPIO_Questions_PIN_Q2_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(GPIO_Questions_PIN_Q3_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(GPIO_Questions_PIN_Q4_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_clearPins(GPIOA, GPIO_Flag_PROMPT_LED_PIN |
		GPIO_OLED_PIN_SCL_PIN |
		GPIO_OLED_PIN_SDA_PIN |
		GPIO_Motor_PIN_2IN2_PIN);
    DL_GPIO_setPins(GPIOA, GPIO_Flag_USER_LED_1_PIN |
		GPIO_Flag_PROMPT_Buzzer_PIN);
    DL_GPIO_enableOutput(GPIOA, GPIO_Flag_USER_LED_1_PIN |
		GPIO_Flag_PROMPT_Buzzer_PIN |
		GPIO_Flag_PROMPT_LED_PIN |
		GPIO_OLED_PIN_SCL_PIN |
		GPIO_OLED_PIN_SDA_PIN |
		GPIO_Motor_PIN_2IN2_PIN);
    DL_GPIO_setUpperPinsPolarity(GPIOA, DL_GPIO_PIN_17_EDGE_RISE |
		DL_GPIO_PIN_16_EDGE_RISE);
    DL_GPIO_clearInterruptStatus(GPIOA, GPIO_Encoder_PIN_1A_PIN |
		GPIO_Encoder_PIN_1B_PIN);
    DL_GPIO_enableInterrupt(GPIOA, GPIO_Encoder_PIN_1A_PIN |
		GPIO_Encoder_PIN_1B_PIN);
    DL_GPIO_clearPins(GPIOB, GPIO_Motor_PIN_1IN1_PIN |
		GPIO_Motor_PIN_1IN2_PIN |
		GPIO_Motor_PIN_2IN1_PIN);
    DL_GPIO_enableOutput(GPIOB, GPIO_Motor_PIN_1IN1_PIN |
		GPIO_Motor_PIN_1IN2_PIN |
		GPIO_Motor_PIN_2IN1_PIN);
    DL_GPIO_setLowerPinsPolarity(GPIOB, DL_GPIO_PIN_1_EDGE_RISE |
		DL_GPIO_PIN_12_EDGE_RISE |
		DL_GPIO_PIN_15_EDGE_RISE);
    DL_GPIO_setUpperPinsPolarity(GPIOB, DL_GPIO_PIN_17_EDGE_RISE);
    DL_GPIO_setLowerPinsInputFilter(GPIOB, DL_GPIO_PIN_1_INPUT_FILTER_8_CYCLES |
		DL_GPIO_PIN_12_INPUT_FILTER_8_CYCLES |
		DL_GPIO_PIN_15_INPUT_FILTER_8_CYCLES);
    DL_GPIO_setUpperPinsInputFilter(GPIOB, DL_GPIO_PIN_17_INPUT_FILTER_8_CYCLES);
    DL_GPIO_clearInterruptStatus(GPIOB, GPIO_Questions_PIN_Q1_PIN |
		GPIO_Questions_PIN_Q2_PIN |
		GPIO_Questions_PIN_Q3_PIN |
		GPIO_Questions_PIN_Q4_PIN);
    DL_GPIO_enableInterrupt(GPIOB, GPIO_Questions_PIN_Q1_PIN |
		GPIO_Questions_PIN_Q2_PIN |
		GPIO_Questions_PIN_Q3_PIN |
		GPIO_Questions_PIN_Q4_PIN);

}



SYSCONFIG_WEAK void SYSCFG_DL_SYSCTL_init(void)
{

	//Low Power Mode is configured to be STANDBY0
    DL_SYSCTL_setPowerPolicySTANDBY0();
    DL_SYSCTL_setBORThreshold(DL_SYSCTL_BOR_THRESHOLD_LEVEL_0);

    
	DL_SYSCTL_setSYSOSCFreq(DL_SYSCTL_SYSOSC_FREQ_BASE);
	/* Set default configuration */
	DL_SYSCTL_disableHFXT();
	DL_SYSCTL_disableSYSPLL();

}


/*
 * Timer clock configuration to be sourced by  / 2 (16000000 Hz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   1000000 Hz = 16000000 Hz / (2 * (15 + 1))
 */
static const DL_TimerG_ClockConfig gPWM_MotorClockConfig = {
    .clockSel = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_2,
    .prescale = 15U
};

static const DL_TimerG_PWMConfig gPWM_MotorConfig = {
    .pwmMode = DL_TIMER_PWM_MODE_EDGE_ALIGN_UP,
    .period = 1000,
    .startTimer = DL_TIMER_STOP,
};

SYSCONFIG_WEAK void SYSCFG_DL_PWM_Motor_init(void) {

    DL_TimerG_setClockConfig(
        PWM_Motor_INST, (DL_TimerG_ClockConfig *) &gPWM_MotorClockConfig);

    DL_TimerG_initPWMMode(
        PWM_Motor_INST, (DL_TimerG_PWMConfig *) &gPWM_MotorConfig);

    DL_TimerG_setCaptureCompareOutCtl(PWM_Motor_INST, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
		DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
		DL_TIMERG_CAPTURE_COMPARE_0_INDEX);

    DL_TimerG_setCaptCompUpdateMethod(PWM_Motor_INST, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERG_CAPTURE_COMPARE_0_INDEX);
    DL_TimerG_setCaptureCompareValue(PWM_Motor_INST, 750, DL_TIMER_CC_0_INDEX);

    DL_TimerG_setCaptureCompareOutCtl(PWM_Motor_INST, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
		DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
		DL_TIMERG_CAPTURE_COMPARE_1_INDEX);

    DL_TimerG_setCaptCompUpdateMethod(PWM_Motor_INST, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERG_CAPTURE_COMPARE_1_INDEX);
    DL_TimerG_setCaptureCompareValue(PWM_Motor_INST, 750, DL_TIMER_CC_1_INDEX);

    DL_TimerG_enableClock(PWM_Motor_INST);


    
    DL_TimerG_setCCPDirection(PWM_Motor_INST , DL_TIMER_CC0_OUTPUT | DL_TIMER_CC1_OUTPUT );


}



/*
 * Timer clock configuration to be sourced by BUSCLK /  (8000000 Hz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   62500 Hz = 8000000 Hz / (4 * (127 + 1))
 */
static const DL_TimerA_ClockConfig gTIMER_computePIDClockConfig = {
    .clockSel    = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_4,
    .prescale    = 127U,
};

/*
 * Timer load value (where the counter starts from) is calculated as (timerPeriod * timerClockFreq) - 1
 * TIMER_computePID_INST_LOAD_VALUE = (200 ms * 62500 Hz) - 1
 */
static const DL_TimerA_TimerConfig gTIMER_computePIDTimerConfig = {
    .period     = TIMER_computePID_INST_LOAD_VALUE,
    .timerMode  = DL_TIMER_TIMER_MODE_PERIODIC,
    .startTimer = DL_TIMER_STOP,
};

SYSCONFIG_WEAK void SYSCFG_DL_TIMER_computePID_init(void) {

    DL_TimerA_setClockConfig(TIMER_computePID_INST,
        (DL_TimerA_ClockConfig *) &gTIMER_computePIDClockConfig);

    DL_TimerA_initTimerMode(TIMER_computePID_INST,
        (DL_TimerA_TimerConfig *) &gTIMER_computePIDTimerConfig);
    DL_TimerA_enableInterrupt(TIMER_computePID_INST , DL_TIMERA_INTERRUPT_ZERO_EVENT);
    DL_TimerA_enableClock(TIMER_computePID_INST);





}

/*
 * Timer clock configuration to be sourced by BUSCLK /  (8000000 Hz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   62500 Hz = 8000000 Hz / (4 * (127 + 1))
 */
static const DL_TimerA_ClockConfig gTIMER_DebugClockConfig = {
    .clockSel    = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_4,
    .prescale    = 127U,
};

/*
 * Timer load value (where the counter starts from) is calculated as (timerPeriod * timerClockFreq) - 1
 * TIMER_Debug_INST_LOAD_VALUE = (600 ms * 62500 Hz) - 1
 */
static const DL_TimerA_TimerConfig gTIMER_DebugTimerConfig = {
    .period     = TIMER_Debug_INST_LOAD_VALUE,
    .timerMode  = DL_TIMER_TIMER_MODE_PERIODIC,
    .startTimer = DL_TIMER_STOP,
};

SYSCONFIG_WEAK void SYSCFG_DL_TIMER_Debug_init(void) {

    DL_TimerA_setClockConfig(TIMER_Debug_INST,
        (DL_TimerA_ClockConfig *) &gTIMER_DebugClockConfig);

    DL_TimerA_initTimerMode(TIMER_Debug_INST,
        (DL_TimerA_TimerConfig *) &gTIMER_DebugTimerConfig);
    DL_TimerA_enableInterrupt(TIMER_Debug_INST , DL_TIMERA_INTERRUPT_ZERO_EVENT);
    DL_TimerA_enableClock(TIMER_Debug_INST);





}



static const DL_UART_Main_ClockConfig gUART_0ClockConfig = {
    .clockSel    = DL_UART_MAIN_CLOCK_BUSCLK,
    .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1
};

static const DL_UART_Main_Config gUART_0Config = {
    .mode        = DL_UART_MAIN_MODE_NORMAL,
    .direction   = DL_UART_MAIN_DIRECTION_TX_RX,
    .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
    .parity      = DL_UART_MAIN_PARITY_NONE,
    .wordLength  = DL_UART_MAIN_WORD_LENGTH_8_BITS,
    .stopBits    = DL_UART_MAIN_STOP_BITS_ONE
};

SYSCONFIG_WEAK void SYSCFG_DL_UART_0_init(void)
{
    DL_UART_Main_setClockConfig(UART_0_INST, (DL_UART_Main_ClockConfig *) &gUART_0ClockConfig);

    DL_UART_Main_init(UART_0_INST, (DL_UART_Main_Config *) &gUART_0Config);
    /*
     * Configure baud rate by setting oversampling and baud rate divisors.
     *  Target baud rate: 115200
     *  Actual baud rate: 115211.52
     */
    DL_UART_Main_setOversampling(UART_0_INST, DL_UART_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(UART_0_INST, UART_0_IBRD_32_MHZ_115200_BAUD, UART_0_FBRD_32_MHZ_115200_BAUD);


    /* Configure Interrupts */
    DL_UART_Main_enableInterrupt(UART_0_INST,
                                 DL_UART_MAIN_INTERRUPT_RX);


    DL_UART_Main_enable(UART_0_INST);
}

static const DL_UART_Main_ClockConfig gUART_JY60ClockConfig = {
    .clockSel    = DL_UART_MAIN_CLOCK_BUSCLK,
    .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1
};

static const DL_UART_Main_Config gUART_JY60Config = {
    .mode        = DL_UART_MAIN_MODE_NORMAL,
    .direction   = DL_UART_MAIN_DIRECTION_TX_RX,
    .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
    .parity      = DL_UART_MAIN_PARITY_NONE,
    .wordLength  = DL_UART_MAIN_WORD_LENGTH_8_BITS,
    .stopBits    = DL_UART_MAIN_STOP_BITS_ONE
};

SYSCONFIG_WEAK void SYSCFG_DL_UART_JY60_init(void)
{
    DL_UART_Main_setClockConfig(UART_JY60_INST, (DL_UART_Main_ClockConfig *) &gUART_JY60ClockConfig);

    DL_UART_Main_init(UART_JY60_INST, (DL_UART_Main_Config *) &gUART_JY60Config);
    /*
     * Configure baud rate by setting oversampling and baud rate divisors.
     *  Target baud rate: 9600
     *  Actual baud rate: 9600.24
     */
    DL_UART_Main_setOversampling(UART_JY60_INST, DL_UART_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(UART_JY60_INST, UART_JY60_IBRD_32_MHZ_9600_BAUD, UART_JY60_FBRD_32_MHZ_9600_BAUD);


    /* Configure Interrupts */
    DL_UART_Main_enableInterrupt(UART_JY60_INST,
                                 DL_UART_MAIN_INTERRUPT_RX);


    DL_UART_Main_enable(UART_JY60_INST);
}

