/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "bsp_uart.h"
#include "app_scheduler.h" // [添加]
#include "task_motion.h"   // [添加]
#include "test_rc_console.h"
#include <math.h> // 用于测试浮点打印
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for vLogTask */
osThreadId_t vLogTaskHandle;
const osThreadAttr_t vLogTask_attributes = {
  .name = "vLogTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for vMotorTask */
osThreadId_t vMotorTaskHandle;
const osThreadAttr_t vMotorTask_attributes = {
  .name = "vMotorTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void LogTask(void *argument);
void MotorTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of vLogTask */
  vLogTaskHandle = osThreadNew(LogTask, NULL, &vLogTask_attributes);

  /* creation of vMotorTask */
  vMotorTaskHandle = osThreadNew(MotorTask, NULL, &vMotorTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
    // [添加] 直接调用 ARA 的调度器入口
    // App_Scheduler_Entry 内部有 for(;;) 循环，所以不会返回
//    App_Scheduler_Entry(argument);

    // 防御性代码，理论上永远不会运行到这里
  /* Infinite loop */
  for(;;)
  {
      osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_LogTask */
/**
* @brief Function implementing the vLogTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LogTask */
void LogTask(void *argument)
{
  /* USER CODE BEGIN LogTask */
//    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
//    TestConsole_Init();
    TestRcConsole_Init();


    /* Infinite loop */
    for(;;)
    {
        // 示例：打印当前系统状态
//        TestConsole_TaskLoop();
        TestRcConsole_TaskLoop();


        osDelay(20);
    }
  /* USER CODE END LogTask */
}

/* USER CODE BEGIN Header_MotorTask */
/**
* @brief Function implementing the vMotorTask thread.
* @param argument: Not used
* @retval None
*/


/* USER CODE END Header_MotorTask */
void MotorTask(void *argument)
{
  /* USER CODE BEGIN MotorTask */


  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END MotorTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

