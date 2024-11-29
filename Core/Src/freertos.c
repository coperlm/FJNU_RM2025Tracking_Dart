/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "tim.h"
#include "IIC.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "usart.h"
#include <math.h>
#include "dmp_self.h"

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

// 定义加速度计和陀螺仪的量程范围
#define ACCEL_RANGE 16.0f    // 加速度计量程 ±16g
#define GYRO_RANGE  2000.0f  // 陀螺仪量程 ±2000°/s
#define G_TO_MS2    9.80665f // g 转换为 m/s2

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif


static uint8_t surve_left , surve_right;//舵机数据
static uint8_t ducked;
uint8_t receiveData[7];

static uint32_t pwmVal = 500;//输出于涵道电机

static uint8_t state = 0;//现状态的状态

static uint8_t x[100] , y[100] , fps[100]; 

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId ducted_motorHandle;
osThreadId gyroscopeHandle;
osThreadId servo_motorHandle;
osThreadId connecterHandle;
osThreadId counterHandle;
osThreadId To_openMVHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void delay(unsigned int k)  
{  
  static int i,j;        
  for(i=0;i<k;i++)
  {      
    for(j=0;j<121;j++);
  }            
}

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void ducted_motor_control(void const * argument);
void gyroscope_read(void const * argument);
void servo_motor_control(void const * argument);
void connect_read(void const * argument);
void counter_func(void const * argument);
void To_openMV_func(void const * argument);

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
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 64);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of ducted_motor */
  osThreadDef(ducted_motor, ducted_motor_control, osPriorityIdle, 0, 64);
  ducted_motorHandle = osThreadCreate(osThread(ducted_motor), NULL);

  /* definition and creation of gyroscope */
  osThreadDef(gyroscope, gyroscope_read, osPriorityIdle, 0, 64);
  gyroscopeHandle = osThreadCreate(osThread(gyroscope), NULL);

  /* definition and creation of servo_motor */
  osThreadDef(servo_motor, servo_motor_control, osPriorityIdle, 0, 64);
  servo_motorHandle = osThreadCreate(osThread(servo_motor), NULL);

  /* definition and creation of connecter */
  osThreadDef(connecter, connect_read, osPriorityIdle, 0, 64);
  connecterHandle = osThreadCreate(osThread(connecter), NULL);

  /* definition and creation of counter */
  osThreadDef(counter, counter_func, osPriorityIdle, 0, 128);
  counterHandle = osThreadCreate(osThread(counter), NULL);

  /* definition and creation of To_openMV */
  osThreadDef(To_openMV, To_openMV_func, osPriorityIdle, 0, 64);
  To_openMVHandle = osThreadCreate(osThread(To_openMV), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_ducted_motor_control */
/**
* @brief Function implementing the ducted_motor thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ducted_motor_control */
void ducted_motor_control(void const * argument)
{
  /* USER CODE BEGIN ducted_motor_control */
  
  pwmVal=500;
  /* Infinite loop */
  for(;;)
  {
    if(pwmVal<700)
    {
      static int i=0;
      if(i<10)
      {i++;}
      else if(i>=10)
      {i=0;pwmVal+=1;}   
    //0.05;
    }
    else{
      pwmVal=700;
    }
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, pwmVal );
    HAL_Delay(1);
    //osDelay(1);
  }
  /* USER CODE END ducted_motor_control */
}

/* USER CODE BEGIN Header_gyroscope_read */
/**
* @brief Function implementing the gyroscope thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_gyroscope_read */
void gyroscope_read(void const * argument)
{
  /* USER CODE BEGIN gyroscope_read */
  /* Infinite loop */
  //如果debug中不显示，需要开一下断点
  //现在dmp不能稳定接收
  surve_left = surve_right = 200;
  surve_left = surve_right = 100;
  
  for(;;)
  {
    static uint8_t a = 2 , b = 2;
    a = MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);
    b = MPU_Get_Accelerometer(&aacx,&aacy,&aacz);
    if( a || b ){
      MPU_Init();
    }
    calculate_angles();
    delay( 2000 );//自定义函数，相当于40ms
    osDelay(1);
  }
  /* USER CODE END gyroscope_read */
}

/* USER CODE BEGIN Header_servo_motor_control */
/**
* @brief Function implementing the servo_motor thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_servo_motor_control */
void servo_motor_control(void const * argument)
{
  /* USER CODE BEGIN servo_motor_control */
  /* Infinite loop */
  for(;;)
  {
    __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,surve_left);
    __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,surve_right);
    osDelay(1);
  }
  /* USER CODE END servo_motor_control */
}

/* USER CODE BEGIN Header_connect_read */
/**
* @brief Function implementing the connecter thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_connect_read */
void connect_read(void const * argument)
{
  /* USER CODE BEGIN connect_read */
  /* Infinite loop */
  for(;;)
  {
    HAL_UART_Receive(&huart1,receiveData,5,100);
    osDelay(1);
    if( receiveData[0] == 'r' ){
      state = 1;
    }else if( receiveData[0] == 'R' ){
      state = 2;
    }else if( receiveData[0] == 'm' ){
      state = 3;
    }else{
      state = 0;
    }
  }
  /* USER CODE END connect_read */
}

/* USER CODE BEGIN Header_counter_func */
/**
* @brief Function implementing the counter thread.
* @param argument: Not used
* @retval None
*/
void checker_forge( int num ){
  surve_left = num + 150;
  surve_right = num + 150;
  HAL_Delay(500);
  return ;
}
void checker(){
  checker_forge( 90 );
  checker_forge(-90 );
  
  checker_forge( 60 );
  checker_forge(-60 );
  
  checker_forge( 30 );
  checker_forge(-30 );
  
  receiveData[0] = 'R';
  pwmVal = 500;
  return ;
}
/* USER CODE END Header_counter_func */
void counter_func(void const * argument)
{
  /* USER CODE BEGIN counter_func */
  /* Infinite loop */
  while( pitch < 0.00001 && pitch > -0.00001 );
  //gyro_init();
  for(;;)
  {
    static float derta_pitch , derta_yaw , derta_row;
    derta_pitch = pitch;//初始化目标坐标
    derta_yaw = yaw;
    derta_row = roll;
    
    static uint8_t left , right;
    static uint8_t mid;
    static uint8_t tempp;
    
    //别忘了每次初始值
    left = right = 0;
    mid = 0;
    
    if( state == 1 ){
      checker();
      left = right = 0;
    }
    else if( state == 2 ){
      left = right = 0;
    }
    else{
    //初版划分权重3:2:1
    tempp = 30;
    if( derta_row > 0 ){//roll轴最重要，我觉得
      left += tempp , right -= tempp;//初版直接拉满，然后随便分配一下~
    }else{
      left -= tempp , right += tempp;
    }
    tempp = 20;
    if( derta_pitch > 0 ){//pitch其次重要，我觉得
      left += tempp , right -= tempp;
    }else{
      left -= tempp , right += tempp;
    }
    tempp = 10;
    if( derta_yaw > 0 ){//yaw也挺重要的，我觉得
      left += tempp , right -= tempp;
    }else{
      left -= tempp , right += tempp;
    }
    mid = 100;//不知道给多少，先给一点吧~
    }
    
    //将-90°-+90°映射到60-240之间（舵机）
    
    surve_left = left + 150;
    surve_right = right + 150;
    
    //将0-300映射到500-800之间（涵道）
    
    ducked = mid + 500;
    
    
  
    osDelay(1);
  }
  /* USER CODE END counter_func */
}

/* USER CODE BEGIN Header_To_openMV_func */
/**
* @brief Function implementing the To_openMV thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_To_openMV_func */
void To_openMV_func(void const * argument)
{
  /* USER CODE BEGIN To_openMV_func */
  /* Infinite loop */
  for(;;)
  {
    if( state == 2 ){
      HAL_UART_Receive(&huart1,fps,5,HAL_MAX_DELAY);
      HAL_UART_Receive(&huart1,x,5,HAL_MAX_DELAY);
      HAL_UART_Receive(&huart1,y,5,HAL_MAX_DELAY);//不知道数据几位哇
    }
    osDelay(1);
  }
  /* USER CODE END To_openMV_func */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

