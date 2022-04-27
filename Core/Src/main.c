/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "main.h"
#include "cmsis_os.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "usbd_hid.h"
#include "usb_hid_keys.h"

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
 UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

extern USBD_HandleTypeDef hUsbDeviceFS;

typedef struct __attribute__((packed)) {
	uint8_t MODIFIER;
	uint8_t RESERVED;
	uint8_t KEYCODES[6];
} subKeyBoard;

subKeyBoard keyBoardHIDsub = {0};

osTimerId_t keyTimers[6];
osSemaphoreId_t keysSemaphore;
osMutexId_t reportMutex;

uint8_t uartBuf[128];
uint16_t uartBufIdx;

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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  HAL_Delay(500);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

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

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

void keyDown(uint8_t keyCode, uint8_t modifiers) {
	osMutexAcquire(reportMutex, HAL_MAX_DELAY);
	keyBoardHIDsub.MODIFIER = modifiers;
	USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&keyBoardHIDsub, sizeof(keyBoardHIDsub));
	osMutexRelease(reportMutex);

	osDelay(10);

	osSemaphoreAcquire(keysSemaphore, HAL_MAX_DELAY);
	osMutexAcquire(reportMutex, HAL_MAX_DELAY);
	for (uint8_t i = 6; i-- > 1;) {
		keyBoardHIDsub.KEYCODES[i] = keyBoardHIDsub.KEYCODES[i - 1];
	}
	keyBoardHIDsub.KEYCODES[0] = keyCode;
	USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&keyBoardHIDsub, sizeof(keyBoardHIDsub));
	osMutexRelease(reportMutex);

	for (uint8_t i = 0; i < 6; i++) {
		if (!osTimerIsRunning(keyTimers[i])) {
			osTimerStart(keyTimers[i], 100);
			return;
		}
	}

	Error_Handler(); //all timers taken???
}

void keyUP(void *argument) {
	osMutexAcquire(reportMutex, HAL_MAX_DELAY);
	keyBoardHIDsub.KEYCODES[6 - osSemaphoreGetCount(keysSemaphore) - 1] = 0;
	USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&keyBoardHIDsub, sizeof(keyBoardHIDsub));
	osMutexRelease(reportMutex);
	osSemaphoreRelease(keysSemaphore);
}

void keyIdle() {
	osMutexAcquire(reportMutex, HAL_MAX_DELAY);
	if (keyBoardHIDsub.MODIFIER != 0) {
		keyBoardHIDsub.MODIFIER = 0;
		USBD_HID_SendReport(&hUsbDeviceFS, (uint8_t*)&keyBoardHIDsub, sizeof(keyBoardHIDsub));
	}
	osMutexRelease(reportMutex);
}

uint8_t getChar() {
	uint32_t startTime = osKernelGetTickCount();
	while (uartBufIdx == sizeof(uartBuf) - __HAL_DMA_GET_COUNTER(huart1.hdmarx)) {
		osDelay(1);
		if (osKernelGetTickCount() > startTime + 50) {
			return 0;
		}
	}

	uint8_t val = uartBuf[uartBufIdx];
	uartBufIdx = (uartBufIdx + 1) % sizeof(uartBuf);

	char debugStr[10];
	sprintf(debugStr, "%02x\r\n", val);
	HAL_UART_Transmit(&huart1, (uint8_t*)debugStr, strlen(debugStr), HAL_MAX_DELAY);

	return val;
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 5 */

  	for (uint8_t i = 0; i < 6; i++) {
  		keyTimers[i] = osTimerNew(keyUP, osTimerOnce, NULL, NULL);
  	}
  	keysSemaphore = osSemaphoreNew(6, 6, NULL);
  	reportMutex = osMutexNew(NULL);

	HAL_UART_Receive_DMA(&huart1, uartBuf, sizeof(uartBuf));

	for (;;) {
		uint8_t c = getChar();

		if (c >= 'A' && c <= 'Z') {
			keyDown(c - 'A' + KEY_A, KEY_MOD_LSHIFT);
		}
		else if (c >= 'a' && c <= 'z') {
			keyDown(c - 'a' + KEY_A, 0);
		}
		else if (c >= '1' && c <= '9') {
			keyDown(c - '1' + KEY_1, 0);
		}
		else switch(c) {
		case 0: keyIdle(); break;
		case '`': keyDown(KEY_GRAVE, 0); break;
		case '~': keyDown(KEY_GRAVE, KEY_MOD_LSHIFT); break;
		case ' ': keyDown(KEY_SPACE, 0); break;
		case '!': keyDown(KEY_1, KEY_MOD_LSHIFT); break;
		case '@': keyDown(KEY_2, KEY_MOD_LSHIFT); break;
		case '#': keyDown(KEY_3, KEY_MOD_LSHIFT); break;
		case '$': keyDown(KEY_4, KEY_MOD_LSHIFT); break;
		case '%': keyDown(KEY_5, KEY_MOD_LSHIFT); break;
		case '^': keyDown(KEY_6, KEY_MOD_LSHIFT); break;
		case '&': keyDown(KEY_7, KEY_MOD_LSHIFT); break;
		case '*': keyDown(KEY_8, KEY_MOD_LSHIFT); break;
		case '(': keyDown(KEY_9, KEY_MOD_LSHIFT); break;
		case ')': keyDown(KEY_0, KEY_MOD_LSHIFT); break;
		case '0': keyDown(KEY_0, 0); break;
		case '-': keyDown(KEY_MINUS, 0); break;
		case '_': keyDown(KEY_MINUS, KEY_MOD_LSHIFT); break;
		case '=': keyDown(KEY_EQUAL, 0); break;
		case '+': keyDown(KEY_EQUAL, KEY_MOD_LSHIFT); break;
		case '[': keyDown(KEY_LEFTBRACE, 0); break;
		case '{': keyDown(KEY_LEFTBRACE, KEY_MOD_LSHIFT); break;
		case ']': keyDown(KEY_RIGHTBRACE, 0); break;
		case '}': keyDown(KEY_RIGHTBRACE, KEY_MOD_LSHIFT); break;
		case '\\': keyDown(KEY_BACKSLASH, 0); break;
		case '|': keyDown(KEY_BACKSLASH, KEY_MOD_LSHIFT); break;
		case ';': keyDown(KEY_SEMICOLON, 0); break;
		case ':': keyDown(KEY_SEMICOLON, KEY_MOD_LSHIFT); break;
		case '\'': keyDown(KEY_APOSTROPHE, 0); break;
		case '"': keyDown(KEY_APOSTROPHE, KEY_MOD_LSHIFT); break;
		case ',': keyDown(KEY_COMMA, 0); break;
		case '<': keyDown(KEY_COMMA, KEY_MOD_LSHIFT); break;
		case '.': keyDown(KEY_DOT, 0); break;
		case '>': keyDown(KEY_DOT, KEY_MOD_LSHIFT); break;
		case '/': keyDown(KEY_SLASH, 0); break;
		case '?': keyDown(KEY_SLASH, KEY_MOD_LSHIFT); break;
		case 0x09: keyDown(KEY_TAB, 0); break;
		case 0x0d: keyDown(KEY_ENTER, 0); break;
		case 0x1b: { //ESC
			switch(c = getChar()) {
			case '[': {
				switch(c = getChar()) {
				case 0x31: {
					switch (getChar()) {
					case '~': keyDown(KEY_HOME, 0); break;
					case 0x31: if (getChar() == '~') keyDown(KEY_F1, 0); break;
					case 0x32: if (getChar() == '~') keyDown(KEY_F2, 0); break;
					case 0x33: if (getChar() == '~') keyDown(KEY_F3, 0); break;
					case 0x34: if (getChar() == '~') keyDown(KEY_F4, 0); break;
					case 0x35: if (getChar() == '~') keyDown(KEY_F5, 0); break;
					case 0x37: if (getChar() == '~') keyDown(KEY_F6, 0); break;
					case 0x38: if (getChar() == '~') keyDown(KEY_F7, 0); break;
					case 0x39: if (getChar() == '~') keyDown(KEY_F8, 0); break;
					}
				} break;
				case 0x32: {
					switch (getChar()) {
					case '~': keyDown(KEY_INSERT, 0); break;
					case 0x30: if (getChar() == '~') keyDown(KEY_F9, 0); break;
					case 0x31: if (getChar() == '~') keyDown(KEY_F10, 0); break;
					case 0x33: if (getChar() == '~') keyDown(KEY_F11, 0); break;
					case 0x34: if (getChar() == '~') keyDown(KEY_F12, 0); break;
					}
				} break;
				case 0x33: if (getChar() == '~') keyDown(KEY_DELETE, 0); break;
				case 0x34: if (getChar() == '~') keyDown(KEY_END, 0); break;
				case 0x35: if (getChar() == '~') keyDown(KEY_PAGEUP, 0); break;
				case 0x36: if (getChar() == '~') keyDown(KEY_PAGEDOWN, 0); break;
				case 0x41: keyDown(KEY_UP, 0); break;
				case 0x42: keyDown(KEY_DOWN, 0); break;
				case 0x43: keyDown(KEY_RIGHT, 0); break;
				case 0x44: keyDown(KEY_LEFT, 0); break;
				} break;
			} break;
			case 0: keyDown(KEY_ESC, 0);
			}
		} break;
		case 0x7f: keyDown(KEY_BACKSPACE, 0); break;
		default:
			if (c >= 1 && c <= 0x1A) {
				keyDown(c - 1 + KEY_A, KEY_MOD_LCTRL);
			}
		}
	}
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM10 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM10) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
