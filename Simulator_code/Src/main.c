/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

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
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;

/* USER CODE BEGIN PV */
#define FALSE                          0u
#define TRUE                           1u
#define Sat_Motor_Speed                9u
#define motor_period                   5u
#define TMR2_16bits                65536u
#define RPM_const               78545455u

//Global variable
enum Output_Level {OFF, ON} level;
enum Event_status {EMPTY, PROGRAMMED, DONE} status;
enum Transmission_Status {BUFFER_TX_EMPTY, TRANSMITING, TRANSMISSION_DONE} transmstatus = BUFFER_TX_EMPTY;
enum Reception_Status {BUFFER_RX_EMPTY, DATA_AVAILABLE_RX_BUFFER} receptstatus = BUFFER_RX_EMPTY;

int8_t sim_rpm_index;                                                           
  
typedef struct emulation
{
	uint8_t  Activation;
	uint16_t Ton;
	uint16_t Toff;
	uint16_t Engine_Speed;
}eng_speed_emu;	

eng_speed_emu Engine_Speed_Simulated[12] = {{ 1, 61473,  3135,   650},  //   650 rpm
                                            { 1, 62235, 14835,   800},  //   800 rpm
                                            { 1, 62895, 24975,  1000},  //  1000 rpm
                                            { 1, 64215, 45255,  2000},  //  2000 rpm
                                            { 1, 64655, 52015,  3000},  //  3000 rpm
                                            { 1, 64875, 55395,  4000},  //  4000 rpm
                                            { 1, 65007, 57423,  5000},  //  5000 rpm
                                            { 1, 65095, 58775,  6000},  //  6000 rpm
                                            { 1, 65158, 59741,  7000},  //  7000 rpm
                                            { 1, 65205, 60465,  8000},  //  8000 rpm
                                            { 1, 65242, 61028,  9000},  //  9000 rpm
                                            { 1, 65271, 61479, 10000}}; // 10000 rpm
typedef struct system_info
{
	uint8_t  Low_speed_detected;
	uint8_t  Cutoff_IGN;
	uint8_t  Update_calc;
  uint32_t nOverflow;	
  uint32_t Rising_Edge_Counter;
	uint32_t Measured_Period;
	uint8_t  nOverflow_RE;
	uint8_t  nOverflow_FE;
	uint16_t Engine_Speed_old;
	uint16_t Engine_Speed;
	uint32_t Predicted_Period;
	uint32_t TDuty_Input_Signal;
  uint32_t TStep;
	uint8_t  nAdv;		
}system_vars;

volatile system_vars scenario = {0,0,0,0,0,0,0,0,0,0,0,0,0,0};

typedef struct list
{
	uint8_t length;
	uint8_t array_sorted[12];
}sort_alg;

sort_alg sortvar = {0,{0,0,0,0,0,0,0,0,0,0,0,0}};

typedef struct Scheduler
{
	uint8_t program;
	uint32_t target_time;
}sched_var;

sched_var array_sched_var[3];

volatile uint32_t timer2 = 0;

typedef struct Motor_Control
{
	uint16_t motor_speed[10];
	uint16_t timer[10];
}motor_control_type;

/*
motor_control_type motor_status = {{    0,  1000,  2000,  1000,  2000,
	                                   1000,  2000,  1000,  2000,  1000},
                                    {  30,   400,   400,   400,   400,
									                    400,   400,   400,   400,   400}};  
*/
motor_control_type motor_status = {{    0,  1000,  1500,  2000,  2500,
	                                   3000,  3500,  4000,  4500,  5000},
                                    { 200,   400,   400,   400,   400,
									                    400,   400,   400,   400,   400}};  
                        	

uint8_t time_elapsed;
																			
//PID control
uint16_t MotorSpeed_Setpoint;		
uint16_t MotorSpeed;	
int32_t CumError;		
uint16_t kpnum=600;
uint16_t kpdenum=1000;
uint16_t kinum=20;
uint16_t kidenum=1000;				
int32_t Pwm_PI=0;		
uint16_t Pwm_OUT=0;		
int32_t error_visual;						

//Communication Variables
uint8_t UART3_txBuffer[6]={'A','1','0','0','0',0x0A};
uint8_t UART3_rxBuffer[5];																			
uint8_t Curve_Generation=0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

void Set_Ouput_LED(void)
{
	HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);	
}
	
void Set_Ouput_EngSpeedSignalEmulation(uint8_t Value)
{
	if (Value == TRUE)
	{
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_SET);	
	}
	else
	{
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);	
	}	
}

void Set_Ouput_InterruptionTest(void)
{
	HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_6);
}

void sort_alg_exec(void)
{	
	uint8_t y=0;
	
	for(int j=0;j<12;j++)
	{
		if(Engine_Speed_Simulated[j].Activation==1)
		{	
			sortvar.array_sorted[y] = j;
			sortvar.length++;
			y++;
		}	
	}	
}	
																						
void Change_Engine_Speed_Simulation(void)
{
	static uint8_t counter;
		
	if(counter<sortvar.length)
	{	
		sim_rpm_index = sortvar.array_sorted[counter];
		counter++;
	}	
	else
	{	
		counter = 0;
	}	
}

void Timeout(uint32_t period, void (*func)(void), sched_var var[], uint8_t pos, uint8_t *resp_var)
{
  uint32_t counter;
    
  counter = HAL_GetTick();
		
  if(var[pos].program == 0)
  {
    var[pos].target_time = counter+period;
    var[pos].program = 1;
  } 

  if(counter>=var[pos].target_time)
  {
    var[pos].program = 0;
    *resp_var = TRUE;   
  }
  else
  {
    *resp_var = FALSE; 
  }  
}

void Engine_STOP_test(void)
{
  static uint32_t actual, old;

	actual=scenario.Rising_Edge_Counter;
	
	if(actual==old)
	{	
		scenario.Rising_Edge_Counter=0;
    MotorSpeed=0;
		CumError=0;
  }
	
  old=actual;  
}

uint8_t ConvertNum4DigToStr(uint16_t num, uint8_t resp[], uint8_t i)
{	
	uint8_t Mil, Cent, Dez, Unid;
	uint16_t Man;
		
	if((num>=0)&&(num<=9999))
	{
		Mil = (num/1000u)+0x30;
		Man = num%1000u;
		Cent = (Man/100u)+0x30;
		Man = Man%100u;
		Dez = (Man/10u)+0x30;
		Unid = (Man%10u)+0x30;
	
		resp[i]=Mil;
		resp[i+1]=Cent;
		resp[i+2]=Dez;
		resp[i+3]=Unid;	
		
		return(TRUE);
	}	
	else
	{	
		return(FALSE);
	}	
}	

void Checksum(uint8_t strg[], uint8_t strg_length)
{	
	uint8_t i,result;
		
	for(i=0;i<(strg_length)-1;i++)
	{
		result+=strg[i];
	}	
	
	strg[(strg_length)-1]=result;
}	

void Data_Transmission(void)
{	
	UART3_txBuffer[0] = 'R';
	UART3_txBuffer[5] = 0x0A;
	ConvertNum4DigToStr(MotorSpeed, UART3_txBuffer, 1);
	transmstatus = TRANSMITING;
	HAL_UART_Transmit_DMA(&huart3, UART3_txBuffer, sizeof(UART3_txBuffer));
}	

uint8_t Data_Reception()
{	
	uint8_t i, j, service;
	uint8_t buffer[5];
	
	if(receptstatus == DATA_AVAILABLE_RX_BUFFER)
	{	
	
		for(i=0;i<5;i++)
		{
			buffer[i] = UART3_rxBuffer[i];
			UART3_rxBuffer[i] = 0x00;
		}	
	
		service=buffer[0];
		j=1;
	
		switch(service)
		{	
			case 'A': MotorSpeed_Setpoint=(((buffer[j]-48u)*1000u)+((buffer[j+1]-48u)*100u)+((buffer[j+2]-48u)*10u)+((buffer[j+3]-48u*1u)));
								break;
		
			case 'B': MotorSpeed_Setpoint=0;
								break;
		
			case 'C': Curve_Generation=1;
								break;
		
			case 'D': Curve_Generation=0;
								time_elapsed=0;
								MotorSpeed_Setpoint=0;		
								break;
			
			default:  break;
		}	
		
		receptstatus = BUFFER_RX_EMPTY;
	}
}

void PI_Control(void)
{		
	int32_t Error=0;
	
	Error=MotorSpeed_Setpoint-MotorSpeed;	
	error_visual=Error;
	
	if((Pwm_PI>=0)&&(Pwm_PI<=2800u))
  {	
		CumError+=Error;
	}
		
	//The wheel speed is not reliable in low speed, can interfer drasticly during the control...
	if(MotorSpeed_Setpoint==0)
	{	
		Error=0;
    CumError=0;		
	}	
	
  Pwm_PI=(((kpnum*kidenum*Error)+(kinum*kpdenum*CumError))/(kpdenum*kidenum));				
  
  if((Pwm_PI>=0)&&(Pwm_PI<=2800u))
  {
		Pwm_OUT=Pwm_PI;
  }  
	else
	{	
		/*
		if(Pwm_PI<0u)
		{	
			Pwm_OUT=0u;
		}	
		else	
	  {	
			Pwm_OUT=2800u;
		}	
		*/
		Pwm_OUT=0u;
  }	  	
	
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,Pwm_OUT);
}	

void Task_Fast(void)
{
	
}	

void Task_Medium(void)
{
	static uint8_t i;
	
	Set_Ouput_LED();	
	
	Data_Reception();
	Data_Transmission();
	
	//if(Curve_Generation)
	if(1)
	{	
		if(time_elapsed==0)
		{	
			MotorSpeed_Setpoint=motor_status.motor_speed[i];		
			time_elapsed=motor_status.timer[i];	
			
			if(i<Sat_Motor_Speed)
			{	
				i++;
			}	
			else
			{	
				i=0;
			}		
		}		
		else
		{	
			time_elapsed--;
		}	
	}
	
	//MotorSpeed_Setpoint=1000;
  PI_Control();  
}	

void Task_Slow(void)
{
	Change_Engine_Speed_Simulation();		
	Engine_STOP_test();
}	

void Periodic_task(uint32_t period, void (*func)(void), sched_var var[], uint8_t pos)
{ 
  volatile uint32_t counter;
  
	counter = HAL_GetTick();
	
  if(var[pos].program == 0)
  {
    var[pos].target_time = counter+period;
    var[pos].program = 1;
  } 

  if(counter>=var[pos].target_time)
  {
    var[pos].program = 0;
    (*func)();   
  }
}

void Set_Pulse_Program(void)
{	
	scenario.Measured_Period += scenario.nOverflow_RE*TMR2_16bits;
  scenario.Engine_Speed = RPM_const/scenario.Measured_Period;	
	MotorSpeed = scenario.Engine_Speed;
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  transmstatus = TRANSMISSION_DONE;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{		
	receptstatus = DATA_AVAILABLE_RX_BUFFER;
	HAL_UART_Receive_DMA(&huart3, (uint8_t*)UART3_rxBuffer, sizeof(UART3_rxBuffer));	  	
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	static int8_t k;
	
	k++;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_UART_Receive_DMA(&huart3, (uint8_t*)UART3_rxBuffer, sizeof(UART3_rxBuffer));
	
	__HAL_TIM_ENABLE_IT(&htim2, TIM_IT_UPDATE);          
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
	
	HAL_TIM_Base_Start_IT(&htim3);
		
	sort_alg_exec();
	Change_Engine_Speed_Simulation();	
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		if (scenario.Update_calc == 1)
	  {
			Set_Pulse_Program();
			scenario.Update_calc = 0;
		}	
		
    //Scheduler
		Periodic_task(20,&Task_Fast, array_sched_var, 0);		
		Periodic_task(100,&Task_Medium, array_sched_var, 1);		
		Periodic_task(1000,&Task_Slow, array_sched_var, 2);		
    		
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 55;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 8;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 100;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1440;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 300;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7 
                          |GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11 
                          |GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_12 
                          |GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_3 
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_7|GPIO_PIN_8 
                          |GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA3 PA4 
                           PA5 PA7 PA8 PA9 
                           PA10 PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_4 
                          |GPIO_PIN_5|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9 
                          |GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA2 PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 PB12 PB13 PB14 
                           PB15 PB3 PB4 PB5 
                           PB7 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14 
                          |GPIO_PIN_15|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5 
                          |GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

uint32_t pulse_phase = 1;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM2)
	{	
		scenario.nOverflow++;
	}	
	
  if(htim->Instance == TIM3)
	{	
		if(Engine_Speed_Simulated[sim_rpm_index].Activation)
		{	
			switch(pulse_phase)
			{			
				case 1:  Set_Ouput_EngSpeedSignalEmulation(ON);
								 __HAL_TIM_SET_COUNTER(&htim3,Engine_Speed_Simulated[sim_rpm_index].Ton);					          
			           pulse_phase = 2;
			           break;
			
		  	case 2:  Set_Ouput_EngSpeedSignalEmulation(OFF);
			           __HAL_TIM_SET_COUNTER(&htim3,Engine_Speed_Simulated[sim_rpm_index].Toff);	  
                 pulse_phase = 1;
			           break;		
			
			  default: break;
		  }		
	  }
	}	
}

void Rising_Edge_Event(void)
{
	scenario.Measured_Period = HAL_TIM_ReadCapturedValue(&htim2,TIM_CHANNEL_1);	 
	scenario.nOverflow_RE = scenario.nOverflow;
	__HAL_TIM_SET_COUNTER(&htim2,0u);	
	scenario.nOverflow = 0;	
	scenario.Rising_Edge_Counter++;	
	
	if (scenario.Rising_Edge_Counter>=2)
	{
		scenario.Update_calc = 1;        //set zero after Engine Stop was detected
	}
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{			
	if((htim->Instance == TIM2)&& 
	   (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1))
	{		
		Rising_Edge_Event();			
	}  		
}	

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
