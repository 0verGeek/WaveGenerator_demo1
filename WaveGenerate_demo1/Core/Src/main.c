/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "dac.h"
#include "dma.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "key.h"
#include "lcd.h"
#include "My_lcd.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
void switch_mode(void);
uint16_t buff[400] = {0};
float amplitude_v   = 1.0f;
float frequency_khz = 1.0f;
uint16_t amplitude_u16;
uint8_t mode = 0;

uint32_t key_update_kick = 0;
uint32_t lcd_update_kick = 0;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_DAC1_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
	LCD_Init();
  LCD_Clear(Black);
  LCD_SetBackColor(Black);
  LCD_SetTextColor(White);
	HAL_TIM_Base_Start(&htim7);//开启定时器
//	amplitude_u16 = (uint16_t)(amplitude_v * 4095/3.3);//4095:12位DAC最大输出值；  3.3：参考电压；
//	float omega = 2 * 3.1415926f * frequency_khz / 400.0f;//提前计算用到的浮点数，使代码运行更流畅
//	for(uint16_t i = 0;i < 400;i++)
//	{
//		buff[i] = amplitude_u16 + amplitude_u16 * (sin(omega * i));
//	}
//	for(uint16_t i = 0;i < 400;i++)
//	{
//		buff[i] = 2000+2000*(sin(2*3.14/400*i));
//	}                          //计算波点对应值并装入数组
//	HAL_DAC_Start_DMA(&hdac1,DAC_CHANNEL_1,(uint32_t *)&buff[0],400,DAC_ALIGN_12B_R);//开启DAC和DMA转运
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		if(HAL_GetTick() - key_update_kick >=10)
		{
			key_serv();//10ms进行一次按键扫描
			key_update_kick = HAL_GetTick();
		}
		key_proc();
		switch_mode();
		if(HAL_GetTick() - lcd_update_kick >=100)
		{
			lcd_proc();//100ms刷新一次lcd
			lcd_update_kick = HAL_GetTick();
		}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		GPIOA->ODR ^= 1 << 5;
    HAL_Delay(4);//延时满足数据计算、写入的时间需求
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV3;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void switch_mode(void)
{
    amplitude_u16   = (uint16_t)(amplitude_v * 4095 / 3.3f);
    float omega = 2 * 3.1415926f * frequency_khz / 400.0f;

    switch (mode)
    {
        case 0: // 正弦波
            for (uint16_t i = 0; i < 400; i++)
            {
                float sine_value = sin(omega * i);
                buff[i] = amplitude_u16 + (uint16_t)(amplitude_u16 * sine_value);
            }
            break;

        case 1: // 方波
            for (int i = 0; i < 400; i++)
            {
                buff[i] = amplitude_u16 + ((sin(omega * i) > 0) ? amplitude_u16 : -amplitude_u16);
            }
            break;

        case 2: // 三角波
            for (int i = 0; i < 400; i++)
            {
                float t = (i * frequency_khz) / (float)400;
                buff[i] = amplitude_u16 + amplitude_u16 * (2.0 * fabs(2.0 * (t - floor(t + 0.5))) - 1.0);
            }
            break;

        case 3: // 锯齿波
            for (int i = 0; i < 400; i++)
            {
                float t = (i * frequency_khz) / (float)400;
                buff[i] = amplitude_u16 + amplitude_u16 * (2.0 * (t - floor(t)) - 1.0);
            }
            break;

        default:
            break;
    }

    HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t *)&buff[0], 400, DAC_ALIGN_12B_R);
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
