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
#include "ALL.h"
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
/* Definitions for SendMyControl_D */
osThreadId_t SendMyControl_DHandle;
const osThreadAttr_t SendMyControl_D_attributes = {
  .name = "SendMyControl_D",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for IMU */
osThreadId_t IMUHandle;
const osThreadAttr_t IMU_attributes = {
  .name = "IMU",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for SwitchControl */
osThreadId_t SwitchControlHandle;
const osThreadAttr_t SwitchControl_attributes = {
  .name = "SwitchControl",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for Get_arm_data */
osThreadId_t Get_arm_dataHandle;
const osThreadAttr_t Get_arm_data_attributes = {
  .name = "Get_arm_data",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for back_control */
osThreadId_t back_controlHandle;
const osThreadAttr_t back_control_attributes = {
  .name = "back_control",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Init_task */
osThreadId_t Init_taskHandle;
const osThreadAttr_t Init_task_attributes = {
  .name = "Init_task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for taskbegain */
osMessageQueueId_t taskbegainHandle;
const osMessageQueueAttr_t taskbegain_attributes = {
  .name = "taskbegain"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void SendMyControl_Data_Task(void *argument);
void IMU_Task(void *argument);
void SwitchControl_task(void *argument);
void get_arm_task(void *argument);
void back_control_task(void *argument);
void Init_motor_Task(void *argument);

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

  /* Create the queue(s) */
  /* creation of taskbegain */
  taskbegainHandle = osMessageQueueNew (6, sizeof(uint8_t), &taskbegain_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of SendMyControl_D */
  SendMyControl_DHandle = osThreadNew(SendMyControl_Data_Task, NULL, &SendMyControl_D_attributes);

  /* creation of IMU */
  IMUHandle = osThreadNew(IMU_Task, NULL, &IMU_attributes);

  /* creation of SwitchControl */
  SwitchControlHandle = osThreadNew(SwitchControl_task, NULL, &SwitchControl_attributes);

  /* creation of Get_arm_data */
  Get_arm_dataHandle = osThreadNew(get_arm_task, NULL, &Get_arm_data_attributes);

  /* creation of back_control */
  back_controlHandle = osThreadNew(back_control_task, NULL, &back_control_attributes);

  /* creation of Init_task */
  Init_taskHandle = osThreadNew(Init_motor_Task, NULL, &Init_task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_SendMyControl_Data_Task */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_SendMyControl_Data_Task */
__weak void SendMyControl_Data_Task(void *argument)
{
  /* USER CODE BEGIN SendMyControl_Data_Task */
  /* Infinite loop */
  for(;;)
  {
		osDelay(1);
  }
  /* USER CODE END SendMyControl_Data_Task */
}

/* USER CODE BEGIN Header_IMU_Task */
/**
* @brief Function implementing the myTask02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_IMU_Task */
__weak void IMU_Task(void *argument)
{
  /* USER CODE BEGIN IMU_Task */
  /* Infinite loop */
  for(;;)
  {
		osDelay(1);
  }
  /* USER CODE END IMU_Task */
}

/* USER CODE BEGIN Header_SwitchControl_task */
/**
* @brief Function implementing the myTask03 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SwitchControl_task */
__weak void SwitchControl_task(void *argument)
{
  /* USER CODE BEGIN SwitchControl_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END SwitchControl_task */
}

/* USER CODE BEGIN Header_get_arm_task */
/**
* @brief Function implementing the Get_arm_data thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_get_arm_task */
__weak void get_arm_task(void *argument)
{
  /* USER CODE BEGIN get_arm_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END get_arm_task */
}

/* USER CODE BEGIN Header_back_control_task */
/**
* @brief Function implementing the back_control thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_back_control_task */
__weak void back_control_task(void *argument)
{
  /* USER CODE BEGIN back_control_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END back_control_task */
}

/* USER CODE BEGIN Header_Init_motor_Task */
/**
* @brief Function implementing the Init_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Init_motor_Task */
__weak void Init_motor_Task(void *argument)
{
  /* USER CODE BEGIN Init_motor_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Init_motor_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

