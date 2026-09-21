/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "app_board_config.h"

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
osThreadId Init_TaskHandleHandle;
osThreadId Comm_TaskHandleHandle;
osThreadId Ins_TaskHandlerHandle;
osThreadId WatchDog_TaskHaHandle;
#if BOARD_HAS_SHOOTER
osThreadId Shoot_TaskHandlHandle;
#endif
osThreadId Control_TaskHandle;
#if BOARD_HAS_GIMBAL
osThreadId Gimbal_TaskHandHandle;
#endif
#define FREERTOS_ENABLE_REFEREE_TASK 0U
#if FREERTOS_ENABLE_REFEREE_TASK
osThreadId Referee_TaskHaHandle;
#endif
#if BOARD_HAS_CHASSIS
osThreadId CHASSISHandle;
#endif
osTimerId SoftTimerHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void Init_Task(void const * argument);
void Comm_Task(void const * argument);
#if BOARD_HAS_INS
void Ins_Task(void const * argument);
#endif
void WatchDog_Task(void const * argument);
#if BOARD_HAS_SHOOTER
void Shoot_Task(void const * argument);
#endif
void Control_Task(void const * argument);
#if BOARD_HAS_GIMBAL
void Gimbal_Task(void const * argument);
#endif
#if FREERTOS_ENABLE_REFEREE_TASK
void Referee_Task(void const * argument);
#endif
#if BOARD_HAS_CHASSIS
void Chassis_Task(void const * argument);
#endif
void SoftTimerCallback(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* GetTimerTaskMemory prototype (linked to static allocation support) */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize );

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

/* USER CODE BEGIN GET_TIMER_TASK_MEMORY */
static StaticTask_t xTimerTaskTCBBuffer;
static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];

void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize )
{
  *ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer;
  *ppxTimerTaskStackBuffer = &xTimerStack[0];
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
  /* place for user code */
}
/* USER CODE END GET_TIMER_TASK_MEMORY */

/* USER CODE BEGIN ApplicationHooks */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
  (void)xTask;
  (void)pcTaskName;
  taskDISABLE_INTERRUPTS();
  for (;;) {
  }
}

void vApplicationMallocFailedHook(void)
{
  taskDISABLE_INTERRUPTS();
  for (;;) {
  }
}
/* USER CODE END ApplicationHooks */

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

  /* Create the timer(s) */
  /* definition and creation of SoftTimer */
  osTimerDef(SoftTimer, SoftTimerCallback);
  SoftTimerHandle = osTimerCreate(osTimer(SoftTimer), osTimerPeriodic, NULL);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of Init_TaskHandle */
  osThreadDef(Init_TaskHandle, Init_Task, osPriorityRealtime, 0, 512);
  Init_TaskHandleHandle = osThreadCreate(osThread(Init_TaskHandle), NULL);

  /* definition and creation of Comm_TaskHandle */
  osThreadDef(Comm_TaskHandle, Comm_Task, osPriorityRealtime, 0, 512);
  Comm_TaskHandleHandle = osThreadCreate(osThread(Comm_TaskHandle), NULL);

#if BOARD_HAS_INS
  /* definition and creation of Ins_TaskHandler */
  osThreadDef(Ins_TaskHandler, Ins_Task, osPriorityRealtime, 0, 512);
  Ins_TaskHandlerHandle = osThreadCreate(osThread(Ins_TaskHandler), NULL);
#endif

  /* definition and creation of WatchDog_TaskHa */
  osThreadDef(WatchDog_TaskHa, WatchDog_Task, osPriorityHigh, 0, 512);
  WatchDog_TaskHaHandle = osThreadCreate(osThread(WatchDog_TaskHa), NULL);

#if BOARD_HAS_SHOOTER
  /* definition and creation of Shoot_TaskHandl */
  osThreadDef(Shoot_TaskHandl, Shoot_Task, osPriorityRealtime, 0, 512);
  Shoot_TaskHandlHandle = osThreadCreate(osThread(Shoot_TaskHandl), NULL);
#endif

  /* definition and creation of board-level control router */
  osThreadDef(Control_TaskThread, Control_Task, osPriorityRealtime, 0, 512);
  Control_TaskHandle = osThreadCreate(osThread(Control_TaskThread), NULL);

#if BOARD_HAS_GIMBAL
  /* definition and creation of Gimbal_TaskHand */
  osThreadDef(Gimbal_TaskHand, Gimbal_Task, osPriorityRealtime, 0, 512);
  Gimbal_TaskHandHandle = osThreadCreate(osThread(Gimbal_TaskHand), NULL);
#endif

#if FREERTOS_ENABLE_REFEREE_TASK
  /* definition and creation of Referee_TaskHa */
  osThreadDef(Referee_TaskHa, Referee_Task, osPriorityRealtime, 0, 512);
  Referee_TaskHaHandle = osThreadCreate(osThread(Referee_TaskHa), NULL);
#endif

#if BOARD_HAS_CHASSIS
  /* definition and creation of CHASSIS */
  osThreadDef(CHASSIS, Chassis_Task, osPriorityRealtime, 0, 512);
  CHASSISHandle = osThreadCreate(osThread(CHASSIS), NULL);
#endif

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_Init_Task */
/**
  * @brief  Function implementing the Init_TaskHandle thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Init_Task */
__weak void Init_Task(void const * argument)
{
  /* USER CODE BEGIN Init_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Init_Task */
}

/* USER CODE BEGIN Header_Comm_Task */
/**
* @brief Function implementing the Comm_TaskHandle thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Comm_Task */
__weak void Comm_Task(void const * argument)
{
  /* USER CODE BEGIN Comm_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Comm_Task */
}

/* USER CODE BEGIN Header_Ins_Task */
/**
* @brief Function implementing the Ins_TaskHandler thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Ins_Task */
#if BOARD_HAS_INS
__weak void Ins_Task(void const * argument)
{
  /* USER CODE BEGIN Ins_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Ins_Task */
}
#endif

/* USER CODE BEGIN Header_WatchDog_Task */
/**
* @brief Function implementing the WatchDog_TaskHa thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_WatchDog_Task */
__weak void WatchDog_Task(void const * argument)
{
  /* USER CODE BEGIN WatchDog_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END WatchDog_Task */
}

/* USER CODE BEGIN Header_Shoot_Task */
/**
* @brief Function implementing the Shoot_TaskHandl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Shoot_Task */
#if BOARD_HAS_SHOOTER
__weak void Shoot_Task(void const * argument)
{
  /* USER CODE BEGIN Shoot_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Shoot_Task */
}
#endif

/* USER CODE BEGIN Header_Control_Task */
/**
* @brief Function implementing the board-level control thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Control_Task */
__weak void Control_Task(void const * argument)
{
  /* USER CODE BEGIN Control_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Control_Task */
}

/* USER CODE BEGIN Header_Gimbal_Task */
/**
* @brief Function implementing the Gimbal_TaskHand thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Gimbal_Task */
#if BOARD_HAS_GIMBAL
__weak void Gimbal_Task(void const * argument)
{
  /* USER CODE BEGIN Gimbal_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Gimbal_Task */
}
#endif

/* USER CODE BEGIN Header_Referee_Task */
/**
* @brief Function implementing the Referee_TaskHa thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Referee_Task */
#if FREERTOS_ENABLE_REFEREE_TASK
__weak void Referee_Task(void const * argument)
{
  /* USER CODE BEGIN Referee_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Referee_Task */
}
#endif

/* USER CODE BEGIN Header_Chassis_Task */
/**
* @brief Function implementing the CHASSIS thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Chassis_Task */
#if BOARD_HAS_CHASSIS
__weak void Chassis_Task(void const * argument)
{
  /* USER CODE BEGIN Chassis_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Chassis_Task */
}
#endif

/* SoftTimerCallback function */
__weak void SoftTimerCallback(void const * argument)
{
  /* USER CODE BEGIN SoftTimerCallback */

  /* USER CODE END SoftTimerCallback */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
