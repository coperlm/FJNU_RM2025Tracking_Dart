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
#include "pid.h"

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

static uint8_t surve_left , surve_right;//舵机数据的pwm，范围50-250
static uint32_t ducked;//涵道电机的pwm，范围500-1000
static uint8_t receiveData[20];//串口一收到的数据（来源于蓝牙串口）
static uint8_t mesg[32];//串口二收到的数据（来源于视觉）
static uint8_t x , y;//解包出来的视觉坐标（x最大160，y最大120）
static uint32_t pwmVal = 500;//输出于涵道电机（现阶段正在使用的，仅在测试阶段）
static uint8_t state;//模式（见README）
static uint8_t slb , srb;

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
void delay(unsigned int k)  
{  
  static int i,j;        
  for(i=0;i<k;i++)
  {      
    for(j=0;j<121;j++);
  }            
}
/* USER CODE END Header_ducted_motor_control */
void ducted_motor_control(void const * argument)
{
  /* USER CODE BEGIN ducted_motor_control */
  //HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
  pwmVal=600;
  ducked = pwmVal;
  /*
  __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, 600 );
  HAL_Delay( 10000 );
  __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, 500 );
  HAL_Delay( 2000 );*/
  /* Infinite loop */
  while( 1 )
  {
    /*
    if(pwmVal<700)
    {
      pwmVal ++;
    }
    else{
      pwmVal=500;
    }*/
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, pwmVal );
//    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, ducked );
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
    if( a || b ){//如果没读到信息，那就初始化一下下
      MPU_Init();
    }
    calculate_angles();
    HAL_Delay(20);
    //delay( 2000 );//自定义函数，相当于40ms
    //osDelay(1);
  }
  /* USER CODE END gyroscope_read */
}

/* USER CODE BEGIN Header_servo_motor_control */
/**
* @brief Function implementing the servo_motor thread.
* @param argument: Not used
* @retval None
*/
PID survePID;
/* USER CODE END Header_servo_motor_control */
void servo_motor_control(void const * argument)
{
  /* USER CODE BEGIN servo_motor_control */
  /*
  PID survePID_left;
  PID survePID_right;
  float left_output, right_output;
  PID_Init(&survePID_left, 10, 0.01, 0.01, 100, 100);
  PID_Init(&survePID_right, 10, 0.01, 0.01, 100, 100);*/
  /* Infinite loop */
  for(;;)
  {
    // 假设您有左、右的反馈值feedback_left和feedback_right
    /*
    PID_Calc(&survePID_left, surve_left, slb);
    PID_Calc(&survePID_right, surve_right, srb);
    
    left_output = survePID_left.output;
    right_output = survePID_right.output;

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (uint8_t)(left_output));
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, (uint8_t)(right_output));
    slb = left_output , srb = right_output;*/
    
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, surve_left );
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, surve_right);
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
    if( receiveData[0] == 'r' ){
      state = 1;
    }else if( receiveData[0] == 'R' ){
      state = 2;
    }else if( receiveData[0] == 'm' ){
      state = 3;
    }else if( receiveData[0] == 'g' ){
      state = 4;
    }else{
      state = 0;
    }
    osDelay(1);
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
/*
  for(int i = -90;i <= 90;i ++){
    checker_forge( i );
  }*/
  receiveData[0] = 'R';
  state = 2;
  ducked = 500;
  return ;
}

float Kp_pitch = 0.5 , Ki_pitch = 0.1 , Kd_pitch = 0.2;
float Kp_yaw = 0.5 , Ki_yaw = 0.1 , Kd_yaw = 0.2;
float k_alpha = 0.1; //机翼角度系数
float K_T = 50.0; //基础推力
float K_y = 0.05; //垂直补偿增益

float f = 1000.0; //摄像头焦距（假设值）


// 计算反正切 atan(x) 的近似值（基于有理函数逼近）
float my_atan(float x) {
    if (x < 0) {
        return -my_atan(-x);  // 利用奇函数特性处理负数
    }
    if (x > 1.0) {
        return 1.570795 - my_atan(1.0 / x);  // 利用 atan(x) = π/2 - atan(1/x) 处理 x > 1
    }

    // 在 [0, 1] 区间内使用多项式近似
    // 系数来源: 《Numerical Recipes in C》优化公式
    const float a1 = 0.99997726;
    const float a3 = -0.33262347;
    const float a5 = 0.19354346;
    const float a7 = -0.11643287;
    const float a9 = 0.05265332;
    const float a11 = -0.01172120;

    float x_sq = x * x;
    float numerator = x * (a1 + x_sq * (a3 + x_sq * (a5 + x_sq * (a7 + x_sq * (a9 + x_sq * a11)))));
    return numerator;
}

/* USER CODE END Header_counter_func */
void counter_func(void const * argument)
{
  /* USER CODE BEGIN counter_func */
  /* Infinite loop */
  static float derta_pitch , derta_yaw , derta_roll;
  static uint8_t left , right;//范围±90，单位为度，控制舵机角度
  static uint8_t mid;//范围0-500，
  static uint8_t tempp;
  state = 3;//初始化状态值，默认为静默模式
  
  while( pitch < 0.00001 && pitch > -0.00001 );//读不到数据就不开始
  for(;;)
  {
    //初始值
    left = right = 0;
    mid = 0;
    //初始化目标坐标
    derta_pitch = pitch;
    derta_yaw = yaw;
    derta_roll = roll;
    
    if( state == 1 ){//校准模式
      checker();
      left = right = 0;
    }else if( state == 2 ){//静默模式，完成后自动转化到静默模式
      left = right = 0;
    }else if( state == 3 ){//纯视觉模式
      derta_roll = (x-40)*2;
      derta_pitch = (y-30)*2;
      left = derta_pitch*2+derta_roll*0.8;
      right = derta_pitch*2+derta_roll*0.8;
    }
    else if( state == 4 ){//纯陀螺仪模式
      //初版划分权重3:2:1
      tempp = 30;
      if( derta_roll > 0 ){//roll轴最重要，我觉得
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
    }else{//默认模式（混合模式）
    //计算目标角度偏差
      float delta_psi = my_atan(x / f);//这个还要细调
      float delta_theta = my_atan(y / f);
      
    //PID计算力矩（简化版，无积分项）
      float tau_theta = Kp_pitch * (delta_theta - pitch) - Kd_pitch * gyroy;
      float tau_psi = Kp_yaw * (delta_psi - yaw) - Kd_yaw * gyroz;
        
    //机翼角度计算
      float alpha_L = k_alpha * (tau_theta + tau_psi);
      float alpha_R = k_alpha * (tau_theta - tau_psi);

    //推力计算（假设目标在y轴正方向需增加推力）
      float thrust = K_T * (1 + K_y * y);
      left = (int)(alpha_L) , right = (int)(alpha_R);
      mid = (int)(thrust);
    }
    //将-90°-+90°映射到60-240之间（舵机）
    surve_left = left + 150;
    surve_right = right + 150;
    
    //将0-100映射到500-800之间（涵道）
    ducked = mid*3 + 500;
    
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
uint8_t x_temp , y_temp , check_temp;
/* USER CODE END Header_To_openMV_func */
void To_openMV_func(void const * argument)
{
  /* USER CODE BEGIN To_openMV_func */
  receiveData[0] = 'm';
  /* Infinite loop */
  for(;;)
  {
    if( state == 3 ){
      
      HAL_UART_Receive(&huart2,mesg,12,HAL_MAX_DELAY);
      
      for(int i = 0;i < 10;i ++){
        x_temp = (int)( mesg[i] ) , y_temp = (int)( mesg[i+1] ) , check_temp = (int)( mesg[i+2] );
        if( (x_temp+y_temp)%121==check_temp ){
          x = (int)( mesg[i+0] );
          y = (int)( mesg[i+1] );
          break;
        }
      }
        /*
        xx = (mesg[starti+0]-'0')*16 + (mesg[starti+1]-'0');
        yy = (mesg[starti+2]-'0')*16 + (mesg[starti+3]-'0');
        x = xx , y = yy;*/
        
      //mesg[0] = mesg[1] = mesg[2] = mesg[3] = mesg[4] = mesg[5] = mesg[6] = mesg[7] = mesg[8] = mesg[9] = mesg[10] = mesg[11] = mesg[12] = mesg[13] = 0;
      
    }
    osDelay(1);
  }
  /* USER CODE END To_openMV_func */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

