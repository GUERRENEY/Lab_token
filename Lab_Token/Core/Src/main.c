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
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd.h"
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

/* USER CODE BEGIN PV */

uint32_t valor_del_counter;
LCD_t lcd;
uint32_t estado;
uint32_t clave = 0xF10ABCD0; // Clave
uint32_t token;
uint32_t tiempo_anterior = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Función para verificar si se detecta el recuadro blanco
uint8_t seDetectoCuadroBlanco(void) {
	estado =HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
    return HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0); // Cambiar según tu configuración real
}

// Función para calcular el TOKEN basado en el valor del contador y la clave
uint32_t calcularToken(uint32_t valor_del_counter, uint32_t clave) {
    return valor_del_counter ^ clave;
}

void lcd_mostrar_token(token) {
    char buffer[16]; // Buffer para convertir el token a una cadena de caracteres
    sprintf(buffer, "   TK: %lx", token); // Convertir el token a una cadena de caracteres en hexadecimal
    lcd_clear(&lcd); // Limpiar la pantalla LCD
    lcd_setCursor(&lcd, 0, 0); // Establecer el cursor en la primera línea
    for (int i = 0; buffer[i] != '\0'; i++) { // Escribir cada carácter de la cadena en la pantalla LCD
        lcd_write(&lcd, buffer[i]); // Escribir el carácter en la pantalla LCD
    }
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
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	// LCD SETTINGS ----------------------------------------------
	lcd.RS_port = LCD_RS_GPIO_Port;
	lcd.RS_pin = LCD_RS_Pin;
	//lcd.RW_port = LCD_RW_GPIO_Port;
	//lcd.RW_pin = LCD_RW_Pin;
	lcd.EN_port = LCD_EN_GPIO_Port;
	lcd.EN_pin = LCD_EN_Pin;
	lcd.D4_port = D4_GPIO_Port;
	lcd.D4_pin = D4_Pin;
	lcd.D5_port = D5_GPIO_Port;
	lcd.D5_pin = D5_Pin;
	lcd.D6_port = D6_GPIO_Port;
	lcd.D6_pin = D6_Pin;
	lcd.D7_port = D7_GPIO_Port;
	lcd.D7_pin = D7_Pin;


	//inicializamos el LCD
	lcd_begin(&lcd, 16, 2, LCD_5x8DOTS);

	//escribimos un mensaje inicial
	lcd_setCursor(&lcd, 0, 0);
	lcd_print(&lcd, "Hola que tal");
	lcd_setCursor(&lcd, 0, 1);
	lcd_print(&lcd, "Embebidos!!");
	HAL_Delay(1500);
	lcd_noDisplay(&lcd);
	lcd_display(&lcd);
	lcd_clear(&lcd);
	lcd_home(&lcd);
	lcd_print(&lcd, "Listos");


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  HAL_TIM_Base_Start_IT(&htim2);



  // Esperar la detección del recuadro blanco


          while (1)
  {
        		 while (!seDetectoCuadroBlanco());

        		       valor_del_counter = __HAL_TIM_GET_COUNTER(&htim2); // Obtener el valor actual del temporizador

        		       // Verificar si han pasado 30 segundos desde el último token generado
        		       if (valor_del_counter - tiempo_anterior >= 30000) { // 30000 cuentas = 30 segundos
        		           tiempo_anterior = valor_del_counter; // Actualizar el tiempo anterior

        		           clave = 0xF10ABCD0;
        		           // Generar un nuevo token utilizando el valor actual del temporizador como parte del cálculo
        		           token = calcularToken(valor_del_counter, clave);

        		           // Mostrar el nuevo token en la pantalla LCD
        		           lcd_mostrar_token(token);
        		           //__HAL_TIM_SET_COUNTER(&htim2, 0);
        		       }


        		       //HAL_Delay(100);


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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 45799;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LCD_EN_Pin|LCD_RS_Pin|D7_Pin|D6_Pin
                          |D5_Pin|D4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_EN_Pin LCD_RS_Pin D7_Pin D6_Pin
                           D5_Pin D4_Pin */
  GPIO_InitStruct.Pin = LCD_EN_Pin|LCD_RS_Pin|D7_Pin|D6_Pin
                          |D5_Pin|D4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
