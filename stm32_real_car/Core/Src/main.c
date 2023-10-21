/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
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
CAN_HandleTypeDef hcan;

UART_HandleTypeDef huart1;

osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */
CAN_FilterTypeDef FilterConfig;
CAN_RxHeaderTypeDef RxHeader; //structure for message reception

/* Create semaphore for UART transmit task */
SemaphoreHandle_t uartTransmitRequestSemaphore;

/* Create uart transmit queue */
QueueHandle_t uartTransmitQueue;

static st_last_data_state_t st_gs_last_data_state;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_USART1_UART_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
void task_process_can_data(void * pvParameters);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	st_uart_queue_item_t st_uart_queue_item;

	/* check message */
	/* Read Message into local data converter rx buffer */
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, st_uart_queue_item.un_data_converter.RxData);

	/* store can id for further processing */
	st_uart_queue_item.item_id = RxHeader.StdId;

	/* give transmit request semaphore */
	xSemaphoreGive(uartTransmitRequestSemaphore);

}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	volatile boolean sent = TRUE;
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
  MX_CAN_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

//  uint8_t * namePtr = NULL;
//  namePtr = myname;

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */

  /* Create semaphore for UART transmit task */
  uartTransmitRequestSemaphore = xSemaphoreCreateBinary();

  /* Take semaphore to block at startup */
  xSemaphoreTake(uartTransmitRequestSemaphore , portMAX_DELAY);

  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  uartTransmitQueue = xQueueCreate(APP_UART_TX_QUEUE_LENGTH, sizeof(st_uart_queue_item_t));

  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
//  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
//  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */

  xTaskCreate(task_process_can_data, "can2rc", APP_RTOS_TASK_STACK_SIZE, (void *) NULL, 0, (void *) NULL);

  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  /* do nothing */
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
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
  /** Initializes the CPU, AHB and APB buses clocks
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
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 16;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_1TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = ENABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = ENABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* Configure CAN Receiving Filter */
    /* set FIFO assignment */
    FilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    /* 0x245<<5; the ID that the filter looks for: Zero to pass all IDs */
    FilterConfig.FilterIdHigh = 0;
    FilterConfig.FilterIdLow = 0;
    FilterConfig.FilterMaskIdHigh = 0;
    FilterConfig.FilterMaskIdLow = 0;

    /* Set Filter Scale */
    FilterConfig.FilterScale = CAN_FILTERSCALE_16BIT; //set filter scale

    /* Enable Filter */
    FilterConfig.FilterActivation = ENABLE;

    /* Configure CAN Filter */
    HAL_CAN_ConfigFilter(&hcan, &FilterConfig); //configure CAN filter

    /* Start CAN */
    HAL_CAN_Start(&hcan);

    /* Enable Rx FIFO0 Interrupt */
    HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);


  /* USER CODE END CAN_Init 2 */

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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */


void task_process_can_data(void * pvParameters)
{
	for(;;)
	{
		st_uart_queue_item_t st_l_uart_queue_item;
		/* Dequeue / block until more data is available to dequeue */
		xQueueReceive(uartTransmitQueue, &st_l_uart_queue_item, portMAX_DELAY);

		/* variables */
		BOOLEAN bool_has_data = FALSE;
		uint8_t u8_l_uart_data = ZERO;
		union
		{
			en_steering_state_t en_current_steering;
			uint8_t 			u8_current_transmission_val;
			uint32_t			u32_lights_val;
			st_lighting_bits_t	st_lights_bits;
		}un_local_temp_data;

		/* Inline helping function(s) */
		inline void app_calc_throttle_power_percentage(uint16_t u16_a_throttle_reading)
		{
			/* calculate throttle percentage */
			u8_l_uart_data = (u16_a_throttle_reading * MAX_PERCENTAGE) / (THROTTLE_READING_REDUCTION_FACTOR);

			/* raise UART send request flag */
			bool_has_data = TRUE;
		}

		/* Map transmission state */
		inline void app_map_transmission(void)
		{
			/* store current transmission value temporarily for readability */
			un_local_temp_data.u8_current_transmission_val = st_l_uart_queue_item.un_data_converter.u8_rxNumber;

			switch(un_local_temp_data.u8_current_transmission_val)
			{
				case APP_CAR_TRANSMISSION_PARK:
				{
					if(TRANSMISSION_STATE_PARKING != st_gs_last_data_state.en_transmission_state)
					{
						/* update global state */
						st_gs_last_data_state.en_transmission_state = TRANSMISSION_STATE_PARKING;

						/* generate frame */
						u8_l_uart_data = GENERATE_ESP_FRAME(APP_ESP_HEADER_TRANSMISSION, SHIFT_HIGH_NIBBLE_TO_LOW(un_local_temp_data.u8_current_transmission_val));

						/* queue queue data and raise UART send request flag */
						bool_has_data = TRUE;
					}
					else
					{
						/* duplicated data - ignore */
					}

					break;
				}
				case APP_CAR_TRANSMISSION_DRIVE:
				{
					if(TRANSMISSION_STATE_DRIVE != st_gs_last_data_state.en_transmission_state)
					{
						/* update global state */
						st_gs_last_data_state.en_transmission_state = TRANSMISSION_STATE_DRIVE;

						/* generate frame */
						u8_l_uart_data = GENERATE_ESP_FRAME(APP_ESP_HEADER_TRANSMISSION, SHIFT_HIGH_NIBBLE_TO_LOW(un_local_temp_data.u8_current_transmission_val));

						/* queue queue data and raise UART send request flag */
						bool_has_data = TRUE;
					}
					else
					{
						/* duplicated data - ignore */
					}

					break;
				}
				case APP_CAR_TRANSMISSION_NEUTRAL:
				{
					if(TRANSMISSION_STATE_NEUTRAL != st_gs_last_data_state.en_transmission_state)
					{
						/* update global state */
						st_gs_last_data_state.en_transmission_state = TRANSMISSION_STATE_NEUTRAL;

						/* generate frame */
						u8_l_uart_data = GENERATE_ESP_FRAME(APP_ESP_HEADER_TRANSMISSION, SHIFT_HIGH_NIBBLE_TO_LOW(un_local_temp_data.u8_current_transmission_val));

						/* queue data and raise UART send request flag */
						bool_has_data = TRUE;
					}
					else
					{
						/* duplicated data - ignore */
					}

					break;
				}
				case APP_CAR_TRANSMISSION_REVERSE:
				{
					if(TRANSMISSION_STATE_REVERSE != st_gs_last_data_state.en_transmission_state)
					{
						/* update global state */
						st_gs_last_data_state.en_transmission_state = TRANSMISSION_STATE_REVERSE;

						/* generate frame */
						u8_l_uart_data = GENERATE_ESP_FRAME(APP_ESP_HEADER_TRANSMISSION, SHIFT_HIGH_NIBBLE_TO_LOW(un_local_temp_data.u8_current_transmission_val));

						/* queue data and raise UART send request flag */
						bool_has_data = TRUE;
					}
					else
					{
						/* duplicated data - ignore */
					}

					break;
				}

				default:
				{
					/* Do Nothing */
					break;
				}
			}
		}

		/* Map Steering Value */
		inline void app_map_steering(uint8_t u8_a_steering_val)
		{
			un_local_temp_data.en_current_steering = STEERING_STATE_NONE;

			/* check steering range */
			/* invalid boundary check */
			if(
					(u8_a_steering_val > APP_CAR_STEERING_THRESHOLD_SHARP_LEFT_MAX) ||
					(u8_a_steering_val < APP_CAR_STEERING_THRESHOLD_SHARP_RIGHT_MIN)
			)
			{
				/* Invalid data - ignore */
			}
			/* decremental steering check */
			else if(u8_a_steering_val >= APP_CAR_STEERING_THRESHOLD_SHARP_LEFT_MIN)
			{
				/* sharp left */
				un_local_temp_data.en_current_steering = STEERING_STATE_SHARP_LEFT;
			}
			else if(u8_a_steering_val >= APP_CAR_STEERING_THRESHOLD_LEFT_MIN)
			{
				/* slight left */
				un_local_temp_data.en_current_steering = STEERING_STATE_LEFT;
			}
			else if(u8_a_steering_val >= APP_CAR_STEERING_THRESHOLD_STRAIGHT_MIN)
			{
				/* straight - no steering */
				un_local_temp_data.en_current_steering = STEERING_STATE_STRAIGHT;
			}
			else if(u8_a_steering_val >= APP_CAR_STEERING_THRESHOLD_RIGHT_MIN)
			{
				/* slight right */
				un_local_temp_data.en_current_steering = STEERING_STATE_RIGHT;
			}
			else
			{
				/* sharp right */
				un_local_temp_data.en_current_steering = STEERING_STATE_SHARP_RIGHT;
			}

			/* queue sending signal to ESP */
			if(
					(un_local_temp_data.en_current_steering != STEERING_STATE_NONE) &&
					(un_local_temp_data.en_current_steering != st_gs_last_data_state.en_steering_state)
				)
			{
				/* update last steering state */
				st_gs_last_data_state.en_steering_state = un_local_temp_data.en_current_steering;

				/* send data */
				u8_l_uart_data = GENERATE_ESP_FRAME(APP_ESP_HEADER_STEERING, un_local_temp_data.en_current_steering);

				/* raise send flag */
				bool_has_data = TRUE;
			}
		}


		/* Map Data */
		switch (RxHeader.StdId)
		{
			case APP_CAN_ID_THROTTLE:
			{
				/* Map throttle values to 0 -> 100% */
				app_calc_throttle_power_percentage(st_l_uart_queue_item.un_data_converter.u16_rxNumber);
				break;
			}
			case APP_CAN_ID_STEERING:
			{
				/* Map steering values */
				app_map_steering(st_l_uart_queue_item.un_data_converter.u8_rxNumber);
				break;
			}

			case APP_CAN_ID_LIGHTS:
			{
				/* check if there's a change */
				if(st_l_uart_queue_item.un_data_converter.u32_rxNumber != st_gs_last_data_state.u32_lighting_val)
				{
					/* changed - calculate changes */
					un_local_temp_data.u32_lights_val = GET_CHANGED_BITS(st_gs_last_data_state.u32_lighting_val,
							st_l_uart_queue_item.un_data_converter.u32_rxNumber);

					/* update last state global variable */
					st_gs_last_data_state.u32_lighting_val = st_l_uart_queue_item.un_data_converter.u32_rxNumber;

					/* check changed lights */
					if(un_local_temp_data.st_lights_bits.u32_bit21_brake_lights)
					{
						/* brake lights state changed - toggle last state */
						TOGGLE(st_gs_last_data_state.st_lighting_state.bool_break_lights);

						/* queue to ESP */
						u8_l_uart_data = GENERATE_ESP_FRAME(APP_ESP_HEADER_LIGHT_BRAKES, st_gs_last_data_state.st_lighting_state.bool_break_lights);

						/* raise flag */
						bool_has_data = TRUE;
					}
					if(un_local_temp_data.st_lights_bits.u32_bit8_left_indicator)
					{
						/* brake lights state changed - toggle last state */
						TOGGLE(st_gs_last_data_state.st_lighting_state.bool_left_indicator);

						if(
								(st_gs_last_data_state.st_lighting_state.bool_left_indicator &&
									st_gs_last_data_state.st_lighting_state.bool_right_indicator) &&
									(!st_gs_last_data_state.st_lighting_state.bool_hazard_indicator)
								)
						{
							/* left & right indicators are on (== hazard) - therefore signal a hazard turn on instead */
							/* update hazard global state */
							st_gs_last_data_state.st_lighting_state.bool_hazard_indicator = TRUE;


							/* queue to ESP */
							u8_l_uart_data = GENERATE_ESP_FRAME(APP_ESP_HEADER_LIGHT_BRAKES, st_gs_last_data_state.st_lighting_state.bool_hazard_indicator);

						}else
						{
							/* turn off hazard state */
							st_gs_last_data_state.st_lighting_state.bool_hazard_indicator = FALSE;

							/* queue to ESP */
							u8_l_uart_data = GENERATE_ESP_FRAME(APP_ESP_HEADER_LIGHT_BRAKES, st_gs_last_data_state.st_lighting_state.bool_left_indicator);
						}


						/* raise flag */
						bool_has_data = TRUE;
					}
					if(un_local_temp_data.st_lights_bits.u32_bit18_right_indicator)
					{
						/* brake lights state changed - toggle last state */
						TOGGLE(st_gs_last_data_state.st_lighting_state.bool_right_indicator);

						if(
								(st_gs_last_data_state.st_lighting_state.bool_left_indicator &&
									st_gs_last_data_state.st_lighting_state.bool_right_indicator) &&
									(!st_gs_last_data_state.st_lighting_state.bool_hazard_indicator)
								)
						{
							/* left & right indicators are on (== hazard) - therefore signal a hazard turn on instead */
							/* update hazard global state */
							st_gs_last_data_state.st_lighting_state.bool_hazard_indicator = TRUE;


							/* queue to ESP */
							u8_l_uart_data = GENERATE_ESP_FRAME(APP_ESP_HEADER_LIGHT_BRAKES, st_gs_last_data_state.st_lighting_state.bool_hazard_indicator);

						}else
						{
							/* turn off hazard state */
							st_gs_last_data_state.st_lighting_state.bool_hazard_indicator = FALSE;

							/* queue to ESP */
							u8_l_uart_data = GENERATE_ESP_FRAME(APP_ESP_HEADER_LIGHT_BRAKES, st_gs_last_data_state.st_lighting_state.bool_right_indicator);
						}


						/* raise flag */
						bool_has_data = TRUE;
					}

				}
				else
				{
					/* no change - do nothing */
				}



				break;
			}
			case APP_CAN_ID_TRANSMISSION:
			{
				/* check transmission state */
				app_map_transmission();
				break;
			}

			default:
			{
				/* Do Nothing */
				break;
			}
		}

		/* Dispatch to UART */
		if(TRUE == bool_has_data)
		{
			/* queue data to be sent over UART */
			HAL_UART_Transmit(&huart1, &u8_l_uart_data, 1, 500);
		}
		else
		{
			/* Do Nothing */
		}
	}
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
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
