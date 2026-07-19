/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "usart.h"
#include "string.h"
#include "robot_def.h"
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
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Task_Control */
osThreadId_t Task_ControlHandle;
const osThreadAttr_t Task_Control_attributes = {
  .name = "Task_Control",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for Task_Depth */
osThreadId_t Task_DepthHandle;
const osThreadAttr_t Task_Depth_attributes = {
  .name = "Task_Depth",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Task_Comms */
osThreadId_t Task_CommsHandle;
const osThreadAttr_t Task_Comms_attributes = {
  .name = "Task_Comms",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for Queues_Cmd */
osMessageQueueId_t Queues_CmdHandle;
const osMessageQueueAttr_t Queues_Cmd_attributes = {
  .name = "Queues_Cmd"
};
/* Definitions for Mutex_State */
osMutexId_t Mutex_StateHandle;
const osMutexAttr_t Mutex_State_attributes = {
  .name = "Mutex_State"
};
/* Definitions for Sem_IMU_Ready */
osSemaphoreId_t Sem_IMU_ReadyHandle;
const osSemaphoreAttr_t Sem_IMU_Ready_attributes = {
  .name = "Sem_IMU_Ready"
};
/* Definitions for Sem_LoRa_Rx */
osSemaphoreId_t Sem_LoRa_RxHandle;
const osSemaphoreAttr_t Sem_LoRa_Rx_attributes = {
  .name = "Sem_LoRa_Rx"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
extern char msg[];
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
extern void StartControlTask(void *argument);
extern void StartDepthTask(void *argument);
extern void StartCommTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of Mutex_State */
  Mutex_StateHandle = osMutexNew(&Mutex_State_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of Sem_IMU_Ready */
  Sem_IMU_ReadyHandle = osSemaphoreNew(1, 1, &Sem_IMU_Ready_attributes);

  /* creation of Sem_LoRa_Rx */
  Sem_LoRa_RxHandle = osSemaphoreNew(1, 1, &Sem_LoRa_Rx_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of Queues_Cmd */
  Queues_CmdHandle = osMessageQueueNew (16, sizeof(Command_t), &Queues_Cmd_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of Task_Control */
  Task_ControlHandle = osThreadNew(StartControlTask, NULL, &Task_Control_attributes);

  /* creation of Task_Depth */
  Task_DepthHandle = osThreadNew(StartDepthTask, NULL, &Task_Depth_attributes);

  /* creation of Task_Comms */
  Task_CommsHandle = osThreadNew(StartCommTask, NULL, &Task_Comms_attributes);

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
  // 心跳任务: 绿灯 500ms 闪烁 = FreeRTOS 调度正常
  // LED 共阳极接法: RESET=亮, SET=灭, Toggle 翻转
  for(;;)
  {
    HAL_GPIO_TogglePin(LED1_GREEN_GPIO_Port, LED1_GREEN_Pin);
    osDelay(500);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

