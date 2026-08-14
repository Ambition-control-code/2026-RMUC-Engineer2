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
osThreadId mainTaskHandle;
osThreadId deal_IMU_dataHandle;
osThreadId Lifting_mechaniHandle;
osThreadId debug_workHandle;
osThreadId UIHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void SendMyControl_Data_Task(void const * argument);
void IMU_Task(void const * argument);
void Lifting_task(void const * argument);
void Debug_task(void const * argument);
void UI_task(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

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
  /* definition and creation of mainTask */
  osThreadDef(mainTask, SendMyControl_Data_Task, osPriorityNormal, 0, 256);
  mainTaskHandle = osThreadCreate(osThread(mainTask), NULL);

  /* definition and creation of deal_IMU_data */
  osThreadDef(deal_IMU_data, IMU_Task, osPriorityRealtime, 0, 256);
  deal_IMU_dataHandle = osThreadCreate(osThread(deal_IMU_data), NULL);

  /* definition and creation of Lifting_mechani */
  osThreadDef(Lifting_mechani, Lifting_task, osPriorityIdle, 0, 256);
  Lifting_mechaniHandle = osThreadCreate(osThread(Lifting_mechani), NULL);

  /* definition and creation of debug_work */
  osThreadDef(debug_work, Debug_task, osPriorityLow, 0, 256);
  debug_workHandle = osThreadCreate(osThread(debug_work), NULL);

  /* definition and creation of UI */
  osThreadDef(UI, UI_task, osPriorityIdle, 0, 256);
  UIHandle = osThreadCreate(osThread(UI), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_SendMyControl_Data_Task */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_SendMyControl_Data_Task */
__weak void SendMyControl_Data_Task(void const * argument)
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
* @brief Function implementing the myTask03 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_IMU_Task */
__weak void IMU_Task(void const * argument)
{
  /* USER CODE BEGIN IMU_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END IMU_Task */
}

/* USER CODE BEGIN Header_Lifting_task */
/**
* @brief Function implementing the Lifting_mechani thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Lifting_task */
__weak void Lifting_task(void const * argument)
{
  /* USER CODE BEGIN Lifting_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Lifting_task */
}

/* USER CODE BEGIN Header_Debug_task */
/**
* @brief Function implementing the debug_work thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Debug_task */
__weak void Debug_task(void const * argument)
{
  /* USER CODE BEGIN Debug_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Debug_task */
}

/* USER CODE BEGIN Header_UI_task */
/**
* @brief Function implementing the UI thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_UI_task */
__weak void UI_task(void const * argument)
{
  /* USER CODE BEGIN UI_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END UI_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
