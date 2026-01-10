/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
  **********************************************************************************
  * @file           : main.c
  * @brief          : Coil Winder Control Program
  * @description    : 2 stepper motors synchronization for coil winding
  *                   - Motor 1 (TIM1_CH4/PA11): Bobbin rotation
  *                   - Motor 2 (TIM2_CH3/PA2): Lead screw for wire guide
  *                   chức năng yêu cầu từ bàn phím:
  *                   - A: start
  *                   - B: Pause
  *                   - C: resume
  *                   - D: menu set
  *                   - #: Home Leadscrew/Reset Total Turns / New Coil
  *                   - *: Slow Mode / Test Mode
  *                   - các số từ 0 đến 9 chỉ dùng để nhập giá trị số nguyên
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "lcd_i2c.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEBOUNCE_TIME 200
#define SLAVE_ADDRESS_LCD 0x27 << 1
#define TIM2_CLK    8000000UL
#define TIM2_PSC    99
#define FLASH_PAGE_ADDR 0x0801FC00
//4 thông số có thể thay đổi

#define LEADSCREW_PITCH     8.0f     // mm (bước ren vitme) //giữ nguyên theo phần cứng

#define WIRE_DIAMETER       0.8f     // mm độ rộng dây
#define PULSES_PER_REV      800      // steps/revolution (cả 2 motor)
#define OVERWIND_TURNS  	10

#define BOBBIN_WIDTH        10.0f   // mm độ rộng bobbin
#define WIRE_SPACING        0.8f     // mm (= WIRE_DIAMETER, quấn sát nhau có thể thêm sai số 0.1 hoặc 0.05, 0.01)
#define TURNS_PER_LAYER     ((uint32_t)(BOBBIN_WIDTH / WIRE_SPACING))  // 200/0.8 = 250 vòng
#define SETTINGS_MENU_SIZE 4
#define SPACING_MENU_SIZE 4
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

uint32_t lastPressTime = 0;
uint32_t lastTime = 0;
uint32_t pause_start_time = 0;
uint32_t total_paused_time = 0;

uint32_t turns_in_current_layer;
uint32_t estimated_turns;
uint32_t elapsed_ms;
uint32_t completed_layers;
uint32_t elapsed_sec;
float new_spacing;

static uint32_t button_press_count = 0 ;
static uint8_t rx_data;
uint8_t values = 1;
char keypad[4][4] = {
    {'1', '2', '3', 'A'},
    {'4', '5', '6', 'B'},
    {'7', '8', '9', 'C'},
    {'*', '0', '#', 'D'}
};
uint16_t rowPins[4] = {GPIO_PIN_12, GPIO_PIN_13, GPIO_PIN_14, GPIO_PIN_15};
uint16_t colPins[4] = {GPIO_PIN_5, GPIO_PIN_6, GPIO_PIN_7, GPIO_PIN_8};
char buffer[10];
uint8_t idx = 0;
uint32_t current_steps = 0;
uint8_t slow_mode = 0;          // 0: bình thường, 1: chế độ chậm (test)
uint8_t in_menu = 0;            // Đang ở trong menu thiết lập
uint8_t menu_item = 0;          // 1: Set Layers, 2: Set RPM, ...
uint32_t temp_value = 0;
char key ;

uint8_t menu_cursor = 0;
uint8_t display_offset = 0;
char input_buffer[50];
uint8_t buffer_idx = 50;
uint8_t test_mode_active = 0;
uint32_t current_freq = 400;
uint8_t enteringRPM = 0;
uint8_t enteringTime = 0;
uint32_t target_runtime = 0;
uint32_t motor_start_time = 0;
uint32_t accumulated_running_ms = 0;
uint8_t motor_running = 0;
uint8_t enableState = 0;
typedef enum {
    STATE_IDLE = 0,
    STATE_RUNNING,
    STATE_PAUSED,
    STATE_COMPLETED,
    STATE_ERROR
} SystemState_t;

typedef enum {
    MENU_MAIN = 0,          // Màn hình chính - hiển thị trạng thái
    MENU_SETTINGS,          // Menu cài đặt
    MENU_INPUT_RPM,         // Đang nhập RPM
    MENU_INPUT_LAYERS,      // Đang nhập số lớp
    MENU_INPUT_SPACING,     // Chọn spacing method
    MENU_TEST_MODE          // Test mode
} MenuState_t;
MenuState_t current_menu = MENU_MAIN;
const char* settings_menu[] = {
    "1. Set RPM",
    "2. Set Layers",
    "3. Wire Spacing",
    "4. Back"
};
const char* spacing_menu[] = {
    "0. Wire 0.8mm",
    "1. Overlap",
    "2. Progressive D/2",
    "3. Back"
};
typedef struct {
    float    wire_diameter;     // mm - đường kính dây
    float    leadscrew_pitch;   // mm - bước ren vít me
    uint32_t pulses_per_rev;    // pulses/vòng quay motor (cả 2 motor)
    uint32_t overwind_turns;    // số vòng quấn thừa (overwind)
    float    bobbin_width;      // mm - chiều rộng phần quấn của bobbin
    float    wire_spacing;      // mm - khoảng cách tâm-của-dây đến tâm-dây kế tiếp
    uint32_t turns_per_layer;   // số vòng trên 1 lớp - sẽ được tính tự động
} CoilConfig_t;
CoilConfig_t coil_config = { //coil_config.wire_diameter
    .wire_diameter    = 0.8f,  //chiều rộng dây
    .leadscrew_pitch  = 8.0f,  //bước ren vitme
    .pulses_per_rev   = 800,   //tần số động cơ
    .overwind_turns   = 10,
    .bobbin_width     = 100.0f,  //chiều rộng bobbin
    .wire_spacing     = 0.8f,   //chiều rộng dây thêm 0.01mm sai số để tránh chồng dây
    .turns_per_layer  = 250      // số vòng cần quấn để đày bobbibn sẽ được tính sau, tạm để 0
};

typedef struct {
    uint32_t bobbin_freq_hz;        // Tần số motor bobbin (Hz)
    uint32_t leadscrew_freq_hz;     // Tần số motor vitme (Hz)
    uint32_t bobbin_rpm;            // Tốc độ bobbin (RPM)
    uint32_t target_layers;         // Số lớp cần quấn
    uint32_t current_layer;         // Lớp hiện tại
    uint32_t steps_in_layer;        // Steps đã quấn trong lớp hiện tại
    uint32_t total_turns;           // Tổng số vòng đã quấn
    uint8_t direction;              // 0: trái->phải, 1: phải->trái
    SystemState_t state;
} CoilWinderParams_t;

CoilWinderParams_t winder = {  //winder.bobbin_freq_hz winder.target_layers
    .bobbin_freq_hz = 0,         // 600 RPM ??
    .leadscrew_freq_hz = 0,			//??
    .bobbin_rpm = 600, 				//???
    .target_layers = 2,   			//thay thế trong thực tế
    .current_layer = 0,
    .steps_in_layer = 0,
    .total_turns = 0,
    .direction = 0,
    .state = STATE_IDLE
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */
char readKeypad(void);
void processKey(char key);
void Choose_Control(char key);
void update_freq(uint32_t target_freq);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void send_uart_message(char *message);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
float parse_input_to_float(char *buf);
void ReadFlagFromFlash(void);
void SaveFlagToFlash(void);

void CoilConfig_Update(void);

void CoilWinder_Init(void);
void CoilWinder_Start(void);
void CoilWinder_Stop(void);
void CoilWinder_Pause(void);
void CoilWinder_Pause_TIM2(void);
void CoilWinder_Resume(void);
void CoilWinder_Resume_TIM2(void);
void CoilWinder_Process(void);
void CoilWinder_SetSpeed(uint32_t rpm);
void CoilWinder_CalculateFrequencies(void);
void TIM1_SetFrequency(uint32_t freq_hz);
void TIM2_SetFrequency(uint32_t freq_hz);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//---------------------------------------------------------//

// Source code is private but can be shared for interview purposes.

//---------------------------------------------------------//

void CoilWinder_Process(void) //hàm thực thi liên tục trog while
{
    if(winder.state != STATE_RUNNING) return;
    // Ước tính số vòng đã quấn dựa trên thời gian
    elapsed_ms = (HAL_GetTick() - motor_start_time) - total_paused_time;
    estimated_turns = (uint32_t)floor((elapsed_ms / 1000.0f) * (winder.bobbin_rpm / 60.0f));
    // Tính số vòng trong lớp hiện tại
    turns_in_current_layer = estimated_turns % coil_config.turns_per_layer;
    completed_layers = estimated_turns / coil_config.turns_per_layer;
    winder.total_turns = estimated_turns;
    winder.steps_in_layer = turns_in_current_layer * coil_config.pulses_per_rev;
    // Check nếu hoàn thành 1 lớp
    if(completed_layers > winder.current_layer) {
		winder.current_layer++;
		winder.steps_in_layer = 0;

		winder.direction = !winder.direction;
		if(winder.current_layer >= winder.target_layers) {
			winder.state = STATE_COMPLETED;
			CoilWinder_Stop();

			lcd_clear_display();
			lcd_goto_XY(0, 0);
			lcd_send_string("COMPLETED!");

			current_menu = MENU_MAIN;
			LCD_ShowMainScreen();
		}
    }

}
/* USER CODE END 0 */

//thực hiện hàm nhập số thực/ kiểm tra hàm bàn phím //thực hiện phần cứng

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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
//  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
//  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);

  HAL_UART_Receive_IT(&huart1, &rx_data, sizeof(rx_data));

  if (HAL_I2C_IsDeviceReady(&hi2c2, SLAVE_ADDRESS_LCD, 2, HAL_MAX_DELAY) != HAL_OK) {
      Error_Handler();
   }

  ReadFlagFromFlash();

  lcd_init();
  LCD_ShowMainScreen();
  CoilConfig_Update();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  key = readKeypad();
	  if(key) {
		  ProcessKeypadInput(key);
	  }
//	  CoilWinder_Process();
	  CoilWinder_MainLoop();

//	  	  char temp_str[20];
//	  	  sprintf(temp_str, "Temperature: %lu\r\n", tempC);
//	  	  HAL_UART_Transmit(&huart1, (uint8_t*)temp_str, strlen(temp_str), HAL_MAX_DELAY);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV4;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 35;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 799;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 400;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 35;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 799;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 400;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_12|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_12
                          |GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_3
                          |GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA6 PA7 PA12 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_12|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB12
                           PB13 PB14 PB15 PB3
                           PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_12
                          |GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_3
                          |GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB5 PB6 PB7 PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
//    if (htim->Instance == TIM1)
//    {
//        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
//    }
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
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

