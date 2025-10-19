/*
 * Copyright (c) 2023, Texas Instruments Incorporated - http://www.ti.com
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
 *  ============ ti_msp_dl_config.h =============
 *  Configured MSPM0 DriverLib module declarations
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G350X
 *  by the SysConfig tool.
 */
#ifndef ti_msp_dl_config_h
#define ti_msp_dl_config_h

#define CONFIG_MSPM0G350X

#if defined(__ti_version__) || defined(__TI_COMPILER_VERSION__)
#define SYSCONFIG_WEAK __attribute__((weak))
#elif defined(__IAR_SYSTEMS_ICC__)
#define SYSCONFIG_WEAK __weak
#elif defined(__GNUC__)
#define SYSCONFIG_WEAK __attribute__((weak))
#endif

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform all required MSP DL initialization
 *
 *  This function should be called once at a point before any use of
 *  MSP DL.
 */


/* clang-format off */

#define POWER_STARTUP_DELAY                                                (16)



#define CPUCLK_FREQ                                                     32000000



/* Defines for PWM_Motor */
#define PWM_Motor_INST                                                     TIMG6
#define PWM_Motor_INST_IRQHandler                               TIMG6_IRQHandler
#define PWM_Motor_INST_INT_IRQN                                 (TIMG6_INT_IRQn)
#define PWM_Motor_INST_CLK_FREQ                                          1000000
/* GPIO defines for channel 0 */
#define GPIO_PWM_Motor_C0_PORT                                             GPIOB
#define GPIO_PWM_Motor_C0_PIN                                      DL_GPIO_PIN_6
#define GPIO_PWM_Motor_C0_IOMUX                                  (IOMUX_PINCM23)
#define GPIO_PWM_Motor_C0_IOMUX_FUNC                 IOMUX_PINCM23_PF_TIMG6_CCP0
#define GPIO_PWM_Motor_C0_IDX                                DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_PWM_Motor_C1_PORT                                             GPIOB
#define GPIO_PWM_Motor_C1_PIN                                      DL_GPIO_PIN_7
#define GPIO_PWM_Motor_C1_IOMUX                                  (IOMUX_PINCM24)
#define GPIO_PWM_Motor_C1_IOMUX_FUNC                 IOMUX_PINCM24_PF_TIMG6_CCP1
#define GPIO_PWM_Motor_C1_IDX                                DL_TIMER_CC_1_INDEX



/* Defines for TIMER_computePID */
#define TIMER_computePID_INST                                            (TIMA0)
#define TIMER_computePID_INST_IRQHandler                        TIMA0_IRQHandler
#define TIMER_computePID_INST_INT_IRQN                          (TIMA0_INT_IRQn)
#define TIMER_computePID_INST_LOAD_VALUE                                (12499U)
/* Defines for TIMER_Debug */
#define TIMER_Debug_INST                                                 (TIMA1)
#define TIMER_Debug_INST_IRQHandler                             TIMA1_IRQHandler
#define TIMER_Debug_INST_INT_IRQN                               (TIMA1_INT_IRQn)
#define TIMER_Debug_INST_LOAD_VALUE                                     (37499U)



/* Defines for UART_0 */
#define UART_0_INST                                                        UART0
#define UART_0_INST_IRQHandler                                  UART0_IRQHandler
#define UART_0_INST_INT_IRQN                                      UART0_INT_IRQn
#define GPIO_UART_0_RX_PORT                                                GPIOA
#define GPIO_UART_0_TX_PORT                                                GPIOA
#define GPIO_UART_0_RX_PIN                                        DL_GPIO_PIN_31
#define GPIO_UART_0_TX_PIN                                        DL_GPIO_PIN_28
#define GPIO_UART_0_IOMUX_RX                                      (IOMUX_PINCM6)
#define GPIO_UART_0_IOMUX_TX                                      (IOMUX_PINCM3)
#define GPIO_UART_0_IOMUX_RX_FUNC                       IOMUX_PINCM6_PF_UART0_RX
#define GPIO_UART_0_IOMUX_TX_FUNC                       IOMUX_PINCM3_PF_UART0_TX
#define UART_0_BAUD_RATE                                                (115200)
#define UART_0_IBRD_32_MHZ_115200_BAUD                                      (17)
#define UART_0_FBRD_32_MHZ_115200_BAUD                                      (23)
/* Defines for UART_JY60 */
#define UART_JY60_INST                                                     UART1
#define UART_JY60_INST_IRQHandler                               UART1_IRQHandler
#define UART_JY60_INST_INT_IRQN                                   UART1_INT_IRQn
#define GPIO_UART_JY60_RX_PORT                                             GPIOA
#define GPIO_UART_JY60_TX_PORT                                             GPIOA
#define GPIO_UART_JY60_RX_PIN                                      DL_GPIO_PIN_9
#define GPIO_UART_JY60_TX_PIN                                      DL_GPIO_PIN_8
#define GPIO_UART_JY60_IOMUX_RX                                  (IOMUX_PINCM20)
#define GPIO_UART_JY60_IOMUX_TX                                  (IOMUX_PINCM19)
#define GPIO_UART_JY60_IOMUX_RX_FUNC                   IOMUX_PINCM20_PF_UART1_RX
#define GPIO_UART_JY60_IOMUX_TX_FUNC                   IOMUX_PINCM19_PF_UART1_TX
#define UART_JY60_BAUD_RATE                                               (9600)
#define UART_JY60_IBRD_32_MHZ_9600_BAUD                                    (208)
#define UART_JY60_FBRD_32_MHZ_9600_BAUD                                     (21)





/* Defines for USER_LED_1: GPIOA.0 with pinCMx 1 on package pin 33 */
#define GPIO_Flag_USER_LED_1_PORT                                        (GPIOA)
#define GPIO_Flag_USER_LED_1_PIN                                 (DL_GPIO_PIN_0)
#define GPIO_Flag_USER_LED_1_IOMUX                                (IOMUX_PINCM1)
/* Defines for PROMPT_Buzzer: GPIOA.15 with pinCMx 37 on package pin 8 */
#define GPIO_Flag_PROMPT_Buzzer_PORT                                     (GPIOA)
#define GPIO_Flag_PROMPT_Buzzer_PIN                             (DL_GPIO_PIN_15)
#define GPIO_Flag_PROMPT_Buzzer_IOMUX                            (IOMUX_PINCM37)
/* Defines for PROMPT_LED: GPIOA.27 with pinCMx 60 on package pin 31 */
#define GPIO_Flag_PROMPT_LED_PORT                                        (GPIOA)
#define GPIO_Flag_PROMPT_LED_PIN                                (DL_GPIO_PIN_27)
#define GPIO_Flag_PROMPT_LED_IOMUX                               (IOMUX_PINCM60)
/* Defines for PIN_ResetYaw: GPIOB.21 with pinCMx 49 on package pin 20 */
#define GPIO_Flag_PIN_ResetYaw_PORT                                      (GPIOB)
#define GPIO_Flag_PIN_ResetYaw_PIN                              (DL_GPIO_PIN_21)
#define GPIO_Flag_PIN_ResetYaw_IOMUX                             (IOMUX_PINCM49)
/* Port definition for Pin Group GPIO_OLED */
#define GPIO_OLED_PORT                                                   (GPIOA)

/* Defines for PIN_SCL: GPIOA.13 with pinCMx 35 on package pin 6 */
#define GPIO_OLED_PIN_SCL_PIN                                   (DL_GPIO_PIN_13)
#define GPIO_OLED_PIN_SCL_IOMUX                                  (IOMUX_PINCM35)
/* Defines for PIN_SDA: GPIOA.12 with pinCMx 34 on package pin 5 */
#define GPIO_OLED_PIN_SDA_PIN                                   (DL_GPIO_PIN_12)
#define GPIO_OLED_PIN_SDA_IOMUX                                  (IOMUX_PINCM34)
/* Defines for PIN_1IN1: GPIOB.8 with pinCMx 25 on package pin 60 */
#define GPIO_Motor_PIN_1IN1_PORT                                         (GPIOB)
#define GPIO_Motor_PIN_1IN1_PIN                                  (DL_GPIO_PIN_8)
#define GPIO_Motor_PIN_1IN1_IOMUX                                (IOMUX_PINCM25)
/* Defines for PIN_1IN2: GPIOB.13 with pinCMx 30 on package pin 1 */
#define GPIO_Motor_PIN_1IN2_PORT                                         (GPIOB)
#define GPIO_Motor_PIN_1IN2_PIN                                 (DL_GPIO_PIN_13)
#define GPIO_Motor_PIN_1IN2_IOMUX                                (IOMUX_PINCM30)
/* Defines for PIN_2IN1: GPIOB.0 with pinCMx 12 on package pin 47 */
#define GPIO_Motor_PIN_2IN1_PORT                                         (GPIOB)
#define GPIO_Motor_PIN_2IN1_PIN                                  (DL_GPIO_PIN_0)
#define GPIO_Motor_PIN_2IN1_IOMUX                                (IOMUX_PINCM12)
/* Defines for PIN_2IN2: GPIOA.11 with pinCMx 22 on package pin 57 */
#define GPIO_Motor_PIN_2IN2_PORT                                         (GPIOA)
#define GPIO_Motor_PIN_2IN2_PIN                                 (DL_GPIO_PIN_11)
#define GPIO_Motor_PIN_2IN2_IOMUX                                (IOMUX_PINCM22)
/* Port definition for Pin Group GPIO_Encoder */
#define GPIO_Encoder_PORT                                                (GPIOA)

/* Defines for PIN_1A: GPIOA.17 with pinCMx 39 on package pin 10 */
// pins affected by this interrupt request:["PIN_1A","PIN_1B"]
#define GPIO_Encoder_INT_IRQN                                   (GPIOA_INT_IRQn)
#define GPIO_Encoder_INT_IIDX                   (DL_INTERRUPT_GROUP1_IIDX_GPIOA)
#define GPIO_Encoder_PIN_1A_IIDX                            (DL_GPIO_IIDX_DIO17)
#define GPIO_Encoder_PIN_1A_PIN                                 (DL_GPIO_PIN_17)
#define GPIO_Encoder_PIN_1A_IOMUX                                (IOMUX_PINCM39)
/* Defines for PIN_1B: GPIOA.16 with pinCMx 38 on package pin 9 */
#define GPIO_Encoder_PIN_1B_IIDX                            (DL_GPIO_IIDX_DIO16)
#define GPIO_Encoder_PIN_1B_PIN                                 (DL_GPIO_PIN_16)
#define GPIO_Encoder_PIN_1B_IOMUX                                (IOMUX_PINCM38)
/* Defines for PIN_1: GPIOA.25 with pinCMx 55 on package pin 26 */
#define GPIO_SensorTracking_PIN_1_PORT                                   (GPIOA)
#define GPIO_SensorTracking_PIN_1_PIN                           (DL_GPIO_PIN_25)
#define GPIO_SensorTracking_PIN_1_IOMUX                          (IOMUX_PINCM55)
/* Defines for PIN_2: GPIOB.24 with pinCMx 52 on package pin 23 */
#define GPIO_SensorTracking_PIN_2_PORT                                   (GPIOB)
#define GPIO_SensorTracking_PIN_2_PIN                           (DL_GPIO_PIN_24)
#define GPIO_SensorTracking_PIN_2_IOMUX                          (IOMUX_PINCM52)
/* Defines for PIN_3: GPIOB.9 with pinCMx 26 on package pin 61 */
#define GPIO_SensorTracking_PIN_3_PORT                                   (GPIOB)
#define GPIO_SensorTracking_PIN_3_PIN                            (DL_GPIO_PIN_9)
#define GPIO_SensorTracking_PIN_3_IOMUX                          (IOMUX_PINCM26)
/* Defines for PIN_4: GPIOB.19 with pinCMx 45 on package pin 16 */
#define GPIO_SensorTracking_PIN_4_PORT                                   (GPIOB)
#define GPIO_SensorTracking_PIN_4_PIN                           (DL_GPIO_PIN_19)
#define GPIO_SensorTracking_PIN_4_IOMUX                          (IOMUX_PINCM45)
/* Defines for PIN_5: GPIOA.22 with pinCMx 47 on package pin 18 */
#define GPIO_SensorTracking_PIN_5_PORT                                   (GPIOA)
#define GPIO_SensorTracking_PIN_5_PIN                           (DL_GPIO_PIN_22)
#define GPIO_SensorTracking_PIN_5_IOMUX                          (IOMUX_PINCM47)
/* Defines for PIN_6: GPIOB.18 with pinCMx 44 on package pin 15 */
#define GPIO_SensorTracking_PIN_6_PORT                                   (GPIOB)
#define GPIO_SensorTracking_PIN_6_PIN                           (DL_GPIO_PIN_18)
#define GPIO_SensorTracking_PIN_6_IOMUX                          (IOMUX_PINCM44)
/* Defines for PIN_7: GPIOA.18 with pinCMx 40 on package pin 11 */
#define GPIO_SensorTracking_PIN_7_PORT                                   (GPIOA)
#define GPIO_SensorTracking_PIN_7_PIN                           (DL_GPIO_PIN_18)
#define GPIO_SensorTracking_PIN_7_IOMUX                          (IOMUX_PINCM40)
/* Port definition for Pin Group GPIO_Questions */
#define GPIO_Questions_PORT                                              (GPIOB)

/* Defines for PIN_Q1: GPIOB.1 with pinCMx 13 on package pin 48 */
// pins affected by this interrupt request:["PIN_Q1","PIN_Q2","PIN_Q3","PIN_Q4"]
#define GPIO_Questions_INT_IRQN                                 (GPIOB_INT_IRQn)
#define GPIO_Questions_INT_IIDX                 (DL_INTERRUPT_GROUP1_IIDX_GPIOB)
#define GPIO_Questions_PIN_Q1_IIDX                           (DL_GPIO_IIDX_DIO1)
#define GPIO_Questions_PIN_Q1_PIN                                (DL_GPIO_PIN_1)
#define GPIO_Questions_PIN_Q1_IOMUX                              (IOMUX_PINCM13)
/* Defines for PIN_Q2: GPIOB.12 with pinCMx 29 on package pin 64 */
#define GPIO_Questions_PIN_Q2_IIDX                          (DL_GPIO_IIDX_DIO12)
#define GPIO_Questions_PIN_Q2_PIN                               (DL_GPIO_PIN_12)
#define GPIO_Questions_PIN_Q2_IOMUX                              (IOMUX_PINCM29)
/* Defines for PIN_Q3: GPIOB.17 with pinCMx 43 on package pin 14 */
#define GPIO_Questions_PIN_Q3_IIDX                          (DL_GPIO_IIDX_DIO17)
#define GPIO_Questions_PIN_Q3_PIN                               (DL_GPIO_PIN_17)
#define GPIO_Questions_PIN_Q3_IOMUX                              (IOMUX_PINCM43)
/* Defines for PIN_Q4: GPIOB.15 with pinCMx 32 on package pin 3 */
#define GPIO_Questions_PIN_Q4_IIDX                          (DL_GPIO_IIDX_DIO15)
#define GPIO_Questions_PIN_Q4_PIN                               (DL_GPIO_PIN_15)
#define GPIO_Questions_PIN_Q4_IOMUX                              (IOMUX_PINCM32)

/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);
void SYSCFG_DL_PWM_Motor_init(void);
void SYSCFG_DL_TIMER_computePID_init(void);
void SYSCFG_DL_TIMER_Debug_init(void);
void SYSCFG_DL_UART_0_init(void);
void SYSCFG_DL_UART_JY60_init(void);


bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
