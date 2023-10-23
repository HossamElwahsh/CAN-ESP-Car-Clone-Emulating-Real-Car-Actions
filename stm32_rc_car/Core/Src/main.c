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

/* NADA BEGIN */
/* NADA END */

/* SAKR BEGIN */
/* SAKR END */

/* NORHAN BEGIN */
/* NORHAN END */

/* AHMED BEGIN */
#include "fonts.h"
#include "ssd1306.h"
#include "test.h"
#include "bitmap.h"
#include "horse_anim.h"
/* AHMED END */

/* HOSSAM BEGIN */
/* HOSSAM END */

/* SALMA BEGIN */
/* SALMA END */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* NADA BEGIN */
/* NADA END */
/*NORHAN BEGIN*/
QueueHandle_t Lights_Queue;

TaskHandle_t Lights_Handle = NULL;
TaskHandle_t LED_Blink_Handle = NULL;

SemaphoreHandle_t Semaphore_Lights;


/*NOURHAN END*/
/*AHMED BEGIN*/
/*AHMED END*/

/* SAKR BEGIN */

typedef enum {
    Neutral = 0X70,
    Reverse = 0X77,
    Drive = 0x71,
    Parking = 0x7F
} transmission_en;

typedef enum {
    Straight = 0xE0,
    right = 0xE2,
    sharp_right = 0xE3,
    left = 0xE8,
    sharp_left = 0xEC
} Steering_en;

typedef enum {
    brake_lights_off = 0x80,
    brake_lights_on = 0x81,
    right_indicators_off = 0x90,
    right_indicators_on = 0x91,
    left_indicators_off = 0xA0,
    left_indicators_on = 0xA1,
    hazard_indicators_off = 0xB0,
    hazard_indicators_on = 0xB1,
    front_lights_off = 0xC0,
    front_lights_on = 0xC1,
    reverse_lights_off = 0xD0,
    reverse_lights_on = 0xD1
} lights_en;


typedef enum {
    throttle_0_percent = 0x10,
    throttle_70_percent = 0x11,
    throttle_80_percent = 0x12,
    throttle_90_percent = 0x13,
    throttle_100_percent = 0x14
} throttle_en;

/* SAKR END */

/* NORHAN BEGIN */
/* NORHAN END */

/* AHMED BEGIN */

/* AHMED END */

/* HOSSAM BEGIN */
/* HOSSAM END */

/* SALMA BEGIN */
/* SALMA END */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* NADA BEGIN */
/* NADA END */

/* SAKR BEGIN */

uint8_t throttle_counter = 0;
uint8_t Rx_data[1]; //  creating a buffer of 1 byte
BaseType_t QueueResult = pdFALSE;

/*creating enum of the 4 transmissions that the car would have..*/

lights_en gl_lights_en = front_lights_on;
transmission_en gl_transmission_en = Parking;
Steering_en gl_steering_en = Straight;
throttle_en gl_throttle_en = throttle_0_percent;

uint8_t Throttle_readings_gl; /*readings of the throttle to be simulated*/

/*creating a semaphore handle*/
UART_HandleTypeDef huart1;

osThreadId defaultTaskHandle;
SemaphoreHandle_t semaphore_transmissionHandle;
SemaphoreHandle_t semaphore_OLEDHandle;
SemaphoreHandle_t semaphore_steeringHandle;

/* SAKR END */

/* NORHAN BEGIN */
/* NORHAN END */

/* AHMED BEGIN */
/* AHMED END */

/* HOSSAM BEGIN */
/* HOSSAM END */

/* SALMA BEGIN */
/* SALMA END */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* NADA BEGIN */
/* NADA END */

/* SAKR BEGIN */
/* SAKR END */

/* NORHAN BEGIN */
/* NORHAN END */

/* AHMED BEGIN */
/* AHMED END */

/* HOSSAM BEGIN */
/* HOSSAM END */

/* SALMA BEGIN */
/* SALMA END */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */

/* NADA BEGIN */
/* NADA END */

/* SAKR BEGIN */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

    HAL_UART_Receive_IT(&huart1, Rx_data, 1);

    switch (Rx_data[0]) {
        case Parking:
            gl_transmission_en = Parking;/*updating car transmission state*/
            /*throttle equals zero.*/
            Throttle_readings_gl = 0;
            xSemaphoreGive(semaphore_transmissionHandle);
            xSemaphoreGive(semaphore_OLEDHandle);


            break;
        case Neutral:
            gl_transmission_en = Neutral;/*updating car transmission state*/
            /*car is in neutral mode..the gears are all up*/
            /*throttle equals zero.*/
            Throttle_readings_gl = 0;
            xSemaphoreGive(semaphore_transmissionHandle);
            xSemaphoreGive(semaphore_OLEDHandle);

            break;
        case Drive:
            //if(Pass_Signal == Green_Flag){
            gl_transmission_en = Drive; /*updating car transmission state for later check*/
            xSemaphoreGive(semaphore_transmissionHandle);
            xSemaphoreGive(semaphore_OLEDHandle);
            //xSemaphoreGive(Semaphore_Ultrasonic); /*for the ultrasonic sensor...*/
            //}
            break;

        case Reverse:
            //if(Pass_Signal == Green_Flag){
            gl_transmission_en = Reverse;/*updating car transmission state  for later check */
            xSemaphoreGive(semaphore_transmissionHandle);
            xSemaphoreGive(semaphore_OLEDHandle);
            //xSemaphoreGive(Semaphore_Ultrasonic); /*for the ultrasonic sensor...*/
            //}
            break;
        default:
            break;
    }

    if (gl_transmission_en == Reverse || gl_transmission_en == Drive) {
        switch (Rx_data[0]) {
            case throttle_0_percent:
                gl_throttle_en = 0;
                break;
            case throttle_70_percent:
                gl_throttle_en = 70;
                break;
            case throttle_80_percent:
                gl_throttle_en = 80;
                break;
            case throttle_90_percent:
                gl_throttle_en = 90;
                break;
            case throttle_100_percent:
                gl_throttle_en = 100;
                break;
            default:
                break;
        }
    }


    switch (Rx_data[0]) {
        case Straight:
            gl_steering_en = Straight;
            xSemaphoreGive(semaphore_steeringHandle);
            break;
        case right :
            gl_steering_en = right;
            xSemaphoreGive(semaphore_steeringHandle);
            break;
        case sharp_right :
            gl_steering_en = sharp_right;
            xSemaphoreGive(semaphore_steeringHandle);
            break;
        case left :
            gl_steering_en = left;
            xSemaphoreGive(semaphore_steeringHandle);
            break;
        case sharp_left:
            gl_steering_en = sharp_left;
            xSemaphoreGive(semaphore_steeringHandle);
            break;
        default:
            break;
    }

    switch (Rx_data[0]) {
        case brake_lights_off:
            gl_lights_en = brake_lights_off;
            xQueueSendFromISR(Lights_Queue, &gl_lights_en, &QueueResult);
            xSemaphoreGive(Semaphore_Lights);
            break;
        case brake_lights_on:
            gl_lights_en = brake_lights_on;
            xQueueSendFromISR(Lights_Queue, &gl_lights_en, &QueueResult);
            xSemaphoreGive(Semaphore_Lights);
            break;
        case right_indicators_off:
            gl_lights_en = right_indicators_off;

            xQueueSendFromISR(Lights_Queue, &gl_lights_en, &QueueResult);
            xSemaphoreGive(Semaphore_Lights);
            break;
        case right_indicators_on:
            gl_lights_en = right_indicators_on;

            xQueueSendFromISR(Lights_Queue, &gl_lights_en, &QueueResult);
            xSemaphoreGive(Semaphore_Lights);
            break;
        case left_indicators_off :
            gl_lights_en = left_indicators_off;

            xQueueSendFromISR(Lights_Queue, &gl_lights_en, &QueueResult);
            xSemaphoreGive(Semaphore_Lights);
            break;
        case left_indicators_on:
            gl_lights_en = left_indicators_on;

            xQueueSendFromISR(Lights_Queue, &gl_lights_en, &QueueResult);
            xSemaphoreGive(Semaphore_Lights);
            break;
        case hazard_indicators_off :
            gl_lights_en = hazard_indicators_off;

            xQueueSendFromISR(Lights_Queue, &gl_lights_en, &QueueResult);
            xSemaphoreGive(Semaphore_Lights);
            break;
        case hazard_indicators_on:
            gl_lights_en = hazard_indicators_on;

            xQueueSendFromISR(Lights_Queue, &gl_lights_en, &QueueResult);
            xSemaphoreGive(Semaphore_Lights);
            break;
        case front_lights_off :
            gl_lights_en = front_lights_off;

            xQueueSendFromISR(Lights_Queue, &gl_lights_en, &QueueResult);
            xSemaphoreGive(Semaphore_Lights);
            break;
        case front_lights_on :
            gl_lights_en = front_lights_on;

            xQueueSendFromISR(Lights_Queue, &gl_lights_en, &QueueResult);
            xSemaphoreGive(Semaphore_Lights);
            break;
        case reverse_lights_off:
            gl_lights_en = reverse_lights_off;

            xQueueSendFromISR(Lights_Queue, &gl_lights_en, &QueueResult);
            xSemaphoreGive(Semaphore_Lights);
            break;
        case reverse_lights_on:
            gl_lights_en = reverse_lights_on;

            xQueueSendFromISR(Lights_Queue, &gl_lights_en, &QueueResult);
            xSemaphoreGive(Semaphore_Lights);
            break;

        default:
            break;
    }


}






/* SAKR END */

/* NORHAN BEGIN */

typedef enum {
    ON,
    OFF
} LED_STATUS;


LED_STATUS Left_led_flag = OFF;
LED_STATUS Right_led_flag = OFF;
/* NORHAN END */

/* AHMED BEGIN */

/* AHMED END */

/* HOSSAM BEGIN */
/* HOSSAM END */

/* SALMA BEGIN */
/* SALMA END */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

static void MX_GPIO_Init(void);

static void MX_TIM3_Init(void);

static void MX_I2C1_Init(void);

static void MX_USART1_UART_Init(void);

static void MX_TIM1_Init(void);

void StartDefaultTask(void const *argument);

/* USER CODE BEGIN PFP */

/* NADA BEGIN */
/* NADA END */

/* SAKR BEGIN */
/* SAKR END */

/* NORHAN BEGIN */
void LightingSystem(void * pvParameter);
void Blinking(void * pvParameter);

/* NORHAN END */

/* AHMED BEGIN */

void OLED_Function(void * pvParameters);
/* AHMED END */

/* HOSSAM BEGIN */
/* HOSSAM END */

/* SALMA BEGIN */
/* SALMA END */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* NADA BEGIN */
/* NADA END */

/* SAKR BEGIN */
/* SAKR END */

/* NORHAN BEGIN */
/* NORHAN END */

/* AHMED BEGIN */
/* AHMED END */

/* HOSSAM BEGIN */
/* HOSSAM END */

/* SALMA BEGIN */
/* SALMA END */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {
    /* USER CODE BEGIN 1 */

    /* NADA BEGIN */
    /* NADA END */

    /* SAKR BEGIN */
    /* SAKR END */

    /* NORHAN BEGIN */
    Lights_Queue = xQueueCreate(15, sizeof(uint8_t));
    /* NORHAN END */

    /* AHMED BEGIN */
    /* AHMED END */

    /* HOSSAM BEGIN */
    /* HOSSAM END */

    /* SALMA BEGIN */
    /* SALMA END */

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* NADA BEGIN */
    /* NADA END */

    /* SAKR BEGIN */

    semaphore_transmissionHandle = xSemaphoreCreateBinary();
    semaphore_OLEDHandle = xSemaphoreCreateBinary();
    semaphore_steeringHandle = xSemaphoreCreateBinary();


    HAL_UART_Receive_IT(&huart1, Rx_data, 1);
    /* SAKR END */

    /* NORHAN BEGIN */
    /* NORHAN END */

    /* AHMED BEGIN */
    /* AHMED END */

    /* HOSSAM BEGIN */
    /* HOSSAM END */

    /* SALMA BEGIN */
    /* SALMA END */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* NADA BEGIN */
    /* NADA END */

    /* SAKR BEGIN */
    /* SAKR END */

    /* NORHAN BEGIN */
    /* NORHAN END */

    /* AHMED BEGIN */
    /* AHMED END */

    /* HOSSAM BEGIN */
    /* HOSSAM END */

    /* SALMA BEGIN */
    /* SALMA END */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_TIM3_Init();
    MX_I2C1_Init();
    MX_USART1_UART_Init();
    MX_TIM1_Init();
    /* USER CODE BEGIN 2 */

    /* NADA BEGIN */
    /* NADA END */

    /* SAKR BEGIN */
    /* SAKR END */

    /* NORHAN BEGIN */
    xTaskCreate(
            LightingSystem,       /* Function that implements the task. */
            "Lightings",          /* Text name for the task. */
            128,      /* Stack size in words, not bytes. */
            (void *) NULL,    /* Parameter passed into the task. */
            1,/* Priority at which the task is created. */
            &Lights_Handle);      /* Used to pass out the created task's handle. */

    xTaskCreate(
            Blinking,       /* Function that implements the task. */
            "Blinking",          /* Text name for the task. */
            128,      /* Stack size in words, not bytes. */
            (void *) NULL,    /* Parameter passed into the task. */
            1,/* Priority at which the task is created. */
            &LED_Blink_Handle);      /* Used to pass out the created task's handle. */


    /* NORHAN END */

    /* AHMED BEGIN */

    /* AHMED END */

    /* HOSSAM BEGIN */
    /* HOSSAM END */

    /* SALMA BEGIN */
    /* SALMA END */

    /* USER CODE END 2 */

    /* USER CODE BEGIN RTOS_MUTEX */
    /* add mutexes, ... */

    /* NADA BEGIN */
    /* NADA END */

    /* SAKR BEGIN */
    /* SAKR END */

    /* NORHAN BEGIN */
    /* NORHAN END */

    /* AHMED BEGIN */
    SSD1306_Init(); // initialize the display
    xTaskCreate(OLED_Function, "oled", 200, (void *) NULL, 0, NULL);
    /* AHMED END */

    /* HOSSAM BEGIN */
    /* HOSSAM END */

    /* SALMA BEGIN */
    /* SALMA END */

    /* USER CODE END RTOS_MUTEX */

    /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* add semaphores, ... */


    /* NADA BEGIN */
    /* NADA END */

    /* SAKR BEGIN */
    /* SAKR END */

    /* NORHAN BEGIN */
    Semaphore_Lights = xSemaphoreCreateBinary();
    /* NORHAN END */

    /* AHMED BEGIN */
    /* AHMED END */

    /* HOSSAM BEGIN */
    /* HOSSAM END */

    /* SALMA BEGIN */
    /* SALMA END */

    /* USER CODE END RTOS_SEMAPHORES */

    /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */

    /* NADA BEGIN */
    /* NADA END */

    /* SAKR BEGIN */
    /* SAKR END */

    /* NORHAN BEGIN */
    /* NORHAN END */

    /* AHMED BEGIN */
    /* AHMED END */

    /* HOSSAM BEGIN */
    /* HOSSAM END */

    /* SALMA BEGIN */
    /* SALMA END */

    /* USER CODE END RTOS_TIMERS */

    /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */

    /* NADA BEGIN */
    /* NADA END */

    /* SAKR BEGIN */
    /* SAKR END */

    /* NORHAN BEGIN */

    /* NORHAN END */

    /* AHMED BEGIN */
    /* AHMED END */

    /* HOSSAM BEGIN */
    /* HOSSAM END */

    /* SALMA BEGIN */
    /* SALMA END */

    /* USER CODE END RTOS_QUEUES */

    /* Create the thread(s) */
    /* definition and creation of defaultTask */
    osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
    defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

    /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */

    /* NADA BEGIN */
    /* NADA END */

    /* SAKR BEGIN */
    /* SAKR END */

    /* NORHAN BEGIN */
    /* NORHAN END */

    /* AHMED BEGIN */
    /* AHMED END */

    /* HOSSAM BEGIN */
    /* HOSSAM END */

    /* SALMA BEGIN */
    /* SALMA END */

    /* USER CODE END RTOS_THREADS */

    /* Start scheduler */
    osKernelStart();

    /* We should never get here as control is now taken by the scheduler */
    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {
        /* NADA BEGIN */
        /* NADA END */

        /* SAKR BEGIN */
        /* SAKR END */

        /* NORHAN BEGIN */
        /* NORHAN END */

        /* AHMED BEGIN */
        /* AHMED END */

        /* HOSSAM BEGIN */
        /* HOSSAM END */

        /* SALMA BEGIN */
        /* SALMA END */

        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }

}

/* NADA BEGIN */
/* NADA END */

/* SAKR BEGIN */
/* SAKR END */

/* NORHAN BEGIN */
void LightingSystem(void * pvParameter) {
    lights_en LedToPowerOn;
    uint8_t messagesWaiting = 0;
    xSemaphoreTake(Semaphore_Lights, portMAX_DELAY);
    for (;;) {
        xSemaphoreTake(Semaphore_Lights, portMAX_DELAY);
        messagesWaiting = uxQueueMessagesWaiting(Lights_Queue);
        while (messagesWaiting != 0) {
            xQueueReceive(Lights_Queue, &LedToPowerOn, portMAX_DELAY);
            if (LedToPowerOn == brake_lights_off) {
                HAL_GPIO_WritePin(LED_BRAKES_GPIO_Port, LED_BRAKES_Pin, GPIO_PIN_RESET);
            } else if (LedToPowerOn == brake_lights_on) {
                HAL_GPIO_WritePin(LED_BRAKES_GPIO_Port, LED_BRAKES_Pin, GPIO_PIN_SET);
            } else if (LedToPowerOn == front_lights_off) {
                HAL_GPIO_WritePin(LED_FRONT_GPIO_Port, LED_FRONT_Pin, GPIO_PIN_RESET);
            } else if (LedToPowerOn == front_lights_on) {
                HAL_GPIO_WritePin(LED_FRONT_GPIO_Port, LED_FRONT_Pin, GPIO_PIN_SET);
            } else if (LedToPowerOn == reverse_lights_off) {
                HAL_GPIO_WritePin(LED_REVERSE_GPIO_Port, LED_REVERSE_Pin, GPIO_PIN_RESET);
            } else if (LedToPowerOn == reverse_lights_on) {
                HAL_GPIO_WritePin(LED_REVERSE_GPIO_Port, LED_REVERSE_Pin, GPIO_PIN_SET);
            } else if (LedToPowerOn == left_indicators_off) {
                Left_led_flag = OFF;
                vTaskSuspend(LED_Blink_Handle);
            } else if (LedToPowerOn == left_indicators_on) {
                Left_led_flag = ON;
                vTaskResume(LED_Blink_Handle);
            } else if (LedToPowerOn == right_indicators_off) {
                Right_led_flag = OFF;
                vTaskSuspend(LED_Blink_Handle);
            } else if (LedToPowerOn == right_indicators_on) {
                Right_led_flag = ON;
                vTaskResume(LED_Blink_Handle);
            }

            messagesWaiting--;


        }
    }


}

void Blinking(void * pvParameter)
{
    vTaskSuspend(LED_Blink_Handle);

    for (;;) {
        if (Right_led_flag == ON) {
            HAL_GPIO_TogglePin(LED_RIGHT_GPIO_Port, LED_RIGHT_Pin);
        } else if (Right_led_flag == OFF) {
            HAL_GPIO_WritePin(LED_RIGHT_GPIO_Port, LED_RIGHT_Pin, GPIO_PIN_RESET);
        }


        if (Left_led_flag == ON) {
            HAL_GPIO_TogglePin(LED_LEFT_GPIO_Port, LED_LEFT_Pin);
        } else if (Left_led_flag == OFF) {
            HAL_GPIO_WritePin(LED_LEFT_GPIO_Port, LED_LEFT_Pin, GPIO_PIN_RESET);
        }


        osDelay(500);
    }

}




/* NORHAN END */

/* AHMED BEGIN */
/* AHMED END */

/* HOSSAM BEGIN */
/* HOSSAM END */

/* SALMA BEGIN */
/* SALMA END */

/* USER CODE END 3 */


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void) {
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
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }
    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
        Error_Handler();
    }
    /** Enables the Clock Security System
    */
    HAL_RCC_EnableCSS();
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void) {

    /* USER CODE BEGIN I2C1_Init 0 */

    /* USER CODE END I2C1_Init 0 */

    /* USER CODE BEGIN I2C1_Init 1 */

    /* USER CODE END I2C1_Init 1 */
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 400000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN I2C1_Init 2 */

    /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void) {

    /* USER CODE BEGIN TIM1_Init 0 */

    /* USER CODE END TIM1_Init 0 */

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_IC_InitTypeDef sConfigIC = {0};

    /* USER CODE BEGIN TIM1_Init 1 */

    /* USER CODE END TIM1_Init 1 */
    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 71;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = 65535;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_IC_Init(&htim1) != HAL_OK) {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }
    sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
    sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
    sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
    sConfigIC.ICFilter = 0;
    if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_2) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM1_Init 2 */

    /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void) {

    /* USER CODE BEGIN TIM3_Init 0 */

    /* USER CODE END TIM3_Init 0 */

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    /* USER CODE BEGIN TIM3_Init 1 */

    /* USER CODE END TIM3_Init 1 */
    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 0;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 65535;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM3_Init 2 */

    /* USER CODE END TIM3_Init 2 */
    HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void) {

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
    if (HAL_UART_Init(&huart1) != HAL_OK) {
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
static void MX_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOA, MOTOR_EN_A_RIGHT_Pin | MOTOR_IN_1_RIGHT_Pin | MOTOR_IN_2_RIGHT_Pin | MOTOR_IN_3_LEFT_Pin
                             | MOTOR_IN_4_LEFT_Pin | LED_LEFT_Pin | US_TRIGGER_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOB, LED_BRAKES_Pin | LED_REVERSE_Pin | LED_FRONT_Pin | LED_RIGHT_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pins : MOTOR_EN_A_RIGHT_Pin MOTOR_IN_1_RIGHT_Pin MOTOR_IN_2_RIGHT_Pin MOTOR_IN_3_LEFT_Pin
                             MOTOR_IN_4_LEFT_Pin LED_LEFT_Pin US_TRIGGER_Pin */
    GPIO_InitStruct.Pin = MOTOR_EN_A_RIGHT_Pin | MOTOR_IN_1_RIGHT_Pin | MOTOR_IN_2_RIGHT_Pin | MOTOR_IN_3_LEFT_Pin
                          | MOTOR_IN_4_LEFT_Pin | LED_LEFT_Pin | US_TRIGGER_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pins : LED_BRAKES_Pin LED_REVERSE_Pin LED_FRONT_Pin LED_RIGHT_Pin */
    GPIO_InitStruct.Pin = LED_BRAKES_Pin | LED_REVERSE_Pin | LED_FRONT_Pin | LED_RIGHT_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const *argument) {
    /* USER CODE BEGIN 5 */
    /* Infinite loop */
    for (;;) {
        osDelay(1);
    }
    /* USER CODE END 5 */
}


/* USER CODE BEGIN Header_OLED_Function */
/**
  * @brief  Function implementing the oled thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_OLED_Function */
void OLED_Function(void * pvParameters) {
    /* USER CODE BEGIN 5 */
    /* Infinite loop */
    xSemaphoreTake(semaphore_transmissionHandle, portMAX_DELAY);
    for (;;) {
        xSemaphoreTake(semaphore_transmissionHandle, portMAX_DELAY);
        SSD1306_Clear();
        SSD1306_GotoXY(10, 10); // goto 10, 10
        SSD1306_Puts("Current mode:", &Font_7x10, 1); // print Hello

        if (gl_transmission_en == Neutral) {
            SSD1306_GotoXY(40, 30);
            SSD1306_Puts("N", &Font_11x18, 1);
        } else if (gl_transmission_en == Parking) {
            SSD1306_GotoXY(55, 30);
            SSD1306_Puts("P", &Font_11x18, 1);
        } else if (gl_transmission_en == Drive) {
            SSD1306_GotoXY(10, 30);
            SSD1306_Puts("D", &Font_11x18, 1);
        } else if (gl_transmission_en == Reverse) {
            SSD1306_GotoXY(25, 30);
            SSD1306_Puts("R", &Font_11x18, 1);
        } else {
            /*		DO NOTHING		*/
        }
        SSD1306_UpdateScreen(); // update screen
    }
    /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    /* USER CODE BEGIN Callback 0 */

    /* USER CODE END Callback 0 */
    if (htim->Instance == TIM2) {
        HAL_IncTick();
    }
    /* USER CODE BEGIN Callback 1 */

    /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void) {
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
