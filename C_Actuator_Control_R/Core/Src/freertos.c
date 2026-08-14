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
/* Definitions for mainTask */
osThreadId_t mainTaskHandle;
const osThreadAttr_t mainTask_attributes = {
  .name = "mainTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for deal_IMU_data */
osThreadId_t deal_IMU_dataHandle;
const osThreadAttr_t deal_IMU_data_attributes = {
  .name = "deal_IMU_data",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for Data_Handle */
osThreadId_t Data_HandleHandle;
const osThreadAttr_t Data_Handle_attributes = {
  .name = "Data_Handle",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for To_Send_Angle */
osThreadId_t To_Send_AngleHandle;
const osThreadAttr_t To_Send_Angle_attributes = {
  .name = "To_Send_Angle",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Debug */
osThreadId_t DebugHandle;
const osThreadAttr_t Debug_attributes = {
  .name = "Debug",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for MYor_remote */
osMessageQueueId_t MYor_remoteHandle;
const osMessageQueueAttr_t MYor_remote_attributes = {
  .name = "MYor_remote"
};
/* Definitions for Claw_flag */
osMessageQueueId_t Claw_flagHandle;
const osMessageQueueAttr_t Claw_flag_attributes = {
  .name = "Claw_flag"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void DM_Arm_Task(void *argument);
void IMU_Task(void *argument);
void Data_Solve_task(void *argument);
void Send_angle_task(void *argument);
void debug_task(void *argument);

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
  /* creation of MYor_remote */
  MYor_remoteHandle = osMessageQueueNew (4, sizeof(uint8_t), &MYor_remote_attributes);

  /* creation of Claw_flag */
  Claw_flagHandle = osMessageQueueNew (32, sizeof(float), &Claw_flag_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of mainTask */
  mainTaskHandle = osThreadNew(DM_Arm_Task, NULL, &mainTask_attributes);

  /* creation of deal_IMU_data */
  deal_IMU_dataHandle = osThreadNew(IMU_Task, NULL, &deal_IMU_data_attributes);

  /* creation of Data_Handle */
  Data_HandleHandle = osThreadNew(Data_Solve_task, NULL, &Data_Handle_attributes);

  /* creation of To_Send_Angle */
  To_Send_AngleHandle = osThreadNew(Send_angle_task, NULL, &To_Send_Angle_attributes);

  /* creation of Debug */
  DebugHandle = osThreadNew(debug_task, NULL, &Debug_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_DM_Arm_Task */
/**
  * @brief  Function implementing the mainTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_DM_Arm_Task */
__weak void DM_Arm_Task(void *argument)
{
  /* USER CODE BEGIN DM_Arm_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END DM_Arm_Task */
}

/* USER CODE BEGIN Header_IMU_Task */
/**
* @brief Function implementing the myTask03 thread.
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

/* USER CODE BEGIN Header_Data_Solve_task */
/**
* @brief Function implementing the Data_Handle thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Data_Solve_task */
__weak void Data_Solve_task(void *argument)
{
  /* USER CODE BEGIN Data_Solve_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Data_Solve_task */
}

/* USER CODE BEGIN Header_Send_angle_task */
/**
* @brief Function implementing the To_Send_Angle thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Send_angle_task */
__weak void Send_angle_task(void *argument)
{
  /* USER CODE BEGIN Send_angle_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Send_angle_task */
}

/* USER CODE BEGIN Header_debug_task */
/**
* @brief Function implementing the Debug thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_debug_task */
__weak void debug_task(void *argument)
{
  /* USER CODE BEGIN debug_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END debug_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

