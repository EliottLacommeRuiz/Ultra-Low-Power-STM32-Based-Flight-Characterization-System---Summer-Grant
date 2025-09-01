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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
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
SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
bool imu_first_time = false;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Imu_Start_Transmission
 * Starts the transmission of the SPI line by setting NCS pin to LOW
 *
 * Takes: void
 * Returns: void
 */
void Imu_Start_Transmission(void)
{
    HAL_GPIO_WritePin(IMU_NCS_GPIO_Port, IMU_NCS_Pin, GPIO_PIN_RESET);
}

/* Imu_Stop_Transmission
 * Stops the transmission of the SPI line by setting NCS pin to HIGH
 *
 * Takes: void
 * Returns: void
 */
void Imu_Stop_Transmission(void)
{
    HAL_GPIO_WritePin(IMU_NCS_GPIO_Port, IMU_NCS_Pin, GPIO_PIN_SET);
}

/* Imu_Reg16_Read
 *
 * Will read data from address given and output 4 byte of data through pointer
 * 	but only 2 are useful (the ones that are saved)
 *
 * Takes: uint8_t address and 2 pointers
 * Returns: uint8_t status
 */
uint8_t Imu_Reg16_Read(uint8_t addr, uint8_t *low_byte, uint8_t *high_byte)
{
    /*
     * Create a transmit array that will set the transmit data to READ
     * Will have 4 bytes [(read set + addr), 0x00, 0x00, 0x00]
     *
     * Command byte: RnW = 1 (read) | Adress [6:0]
     */
    uint8_t tx_buf[4] = { ((addr & 0x7F) | 0x80), 0x00, 0x00, 0x00 };
    uint8_t rx_buf[4] = { 0 };

    Imu_Start_Transmission();

    // Send and get data while seeing if out status is ok
    uint8_t status = (HAL_SPI_TransmitReceive(&hspi1, tx_buf, rx_buf, 4,
    HAL_MAX_DELAY) == HAL_OK);

    Imu_Stop_Transmission();

    *low_byte = rx_buf[2];
    *high_byte = rx_buf[3];
    return status;
}

/* Imu_Reg16_Write
 *
 * Will take in an 8-bit addr and 2 8-bit write instruction and write it
 *
 * Takes: uint8_t addr and 2 uint8_t instructions
 * Returns: uint8_t status
 */
uint8_t Imu_Reg16_Write(uint8_t addr, uint8_t low_instruction_byte,
        uint8_t high_instruction_byte)
{
    // Store the addr, then the instruction with low byte first
    // Command byte: RnW = 0 (write) | Adress [6:0]
    uint8_t tx_buf[3] = { (addr & 0x7F), low_instruction_byte,
            high_instruction_byte };

    // Tranasmit data
    Imu_Start_Transmission();
    uint8_t status = (HAL_SPI_Transmit(&hspi1, tx_buf, sizeof(tx_buf),
            HAL_MAX_DELAY) == HAL_OK);
    Imu_Stop_Transmission();

    //uint8_t status = ( HAL_SPI_Transmit(&hspi1, tx_buf, , Timeout))
    return status;
}

/*Imu_SetUp
 *
 * Will setup the imu and everything required to get it into SPI mode
 *   will also make sure that the chip is ID properly
 *
 * Take: Void
 * Return: Void
 *
 */
void Imu_SetUp(void)
{
    // Set the data bytes
    uint8_t low_byte = 0x00;
    uint8_t high_byte = 0x00;

    // Initialize SPI
    Imu_Reg16_Read(IMU_REG_CHIP_ADDR, 0x00, 0x00);

    // Check chip_id
    Imu_Reg16_Read(IMU_REG_CHIP_ADDR, &low_byte, &high_byte);
    if (low_byte == IMU_REG_CHIP_ID)
    {
        HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
    }

    // Check if power is ok
    Imu_Reg16_Read(IMU_ERR_ADDR, &low_byte, &high_byte);
    if ((low_byte & 0x01) == 0x00) // Bitmask on low_byte to make sure only comparing LSB
    {
        HAL_GPIO_WritePin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin, GPIO_PIN_SET);
    }

    // Check if sensor status is ok
    /*
     * NOTE: Sometimes this needs a hard reset (removing power)
     *         since the register is instantly reset after 1 read
     */
    Imu_Reg16_Read(IMU_STATUS_ADDR, &low_byte, &high_byte);
    if ((low_byte & 0x01) == 0x01)
    {
        HAL_GPIO_WritePin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin, GPIO_PIN_RESET);
    }

    // Setup of Gyro
    Imu_Reg16_Write(IMU_GYRO_CONFIG_ADDR, IMU_GYRO_CONFIG_LOW,
            IMU_GYRO_CONFIG_HIGH);

//    // Setup accel to low power
//    Imu_Reg16_Write(IMU_ACCEL_CONFIG_ADDR, IMU_ACCEL_LOW_POWER_CONFIG_LOW, IMU_ACCEL_LOW_POWER_CONFIG_HIGH);
//
//    // Check sensor status
//    Imu_Reg16_Read(IMU_STATUS_ADDR, &low_byte, &high_byte);
//    while ((low_byte & 0x40) != 0x40) // Bitmask on low_byte to make sure only comparing 7th bit (MSB of the low byte)
//    {
//        HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);
//        Imu_Reg16_Read(IMU_STATUS_ADDR, &low_byte, &high_byte);
//    }
//    HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);

    // Excelente! Successful init!

} /* END of Imu_Setup */

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
    MX_SPI1_Init();
    MX_USB_DEVICE_Init();
    /* USER CODE BEGIN 2 */

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */

//  char txBuf[8];
//  uint8_t count = 1;
    char tx_buf[8];
    // Create rx array to store read information
    while (1)
    {

        /* START of uncommented code for while loop*/
        if (!imu_first_time)
        {
            Imu_SetUp();
            imu_first_time = true;
        }
        // Set the data bytes
        uint8_t data_low_byte = 0x00;
        uint8_t data_high_byte = 0x00;

        uint8_t read_low_byte = 0x00;
        uint8_t read__high_byte = 0x00;

        Imu_Reg16_Read(IMU_STATUS_ADDR, &read_low_byte, &read__high_byte);

        if ((read_low_byte & 0x40) == 0x40)
        {
            Imu_Reg16_Read(0x03, &data_low_byte, &data_high_byte);
            sprintf(tx_buf, "%u", data_high_byte);
            sprintf(tx_buf, "%u\r\n", data_low_byte);
            CDC_Transmit_FS((uint8_t*) tx_buf, strlen(tx_buf));
        }

        /* END of uncommented code for while loop*/

//	  HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
//	  sprintf(txBuf, "%u\r\n", count);
//
//	  if (count >= 100)
//	  {
//		  count = 1;
//	  }
//	  count++;
//
//	  CDC_Transmit_FS((uint8_t *) txBuf, strlen(txBuf));
//
//	  HAL_Delay(500);
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
    RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
    RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
    RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

    /** Configure the main internal regulator output voltage
     */
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_6;
    RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_3;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
            | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
    {
        Error_Handler();
    }
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
    PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void)
{

    /* USER CODE BEGIN SPI1_Init 0 */

    /* USER CODE END SPI1_Init 0 */

    /* USER CODE BEGIN SPI1_Init 1 */

    /* USER CODE END SPI1_Init 1 */
    /* SPI1 parameter configuration*/
    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial = 7;
    if (HAL_SPI_Init(&hspi1) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN SPI1_Init 2 */

    /* USER CODE END SPI1_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = { 0 };
    /* USER CODE BEGIN MX_GPIO_Init_1 */

    /* USER CODE END MX_GPIO_Init_1 */

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOA, FLASH_CS_Pin | FLASH_HOLD_RST_Pin | FLASH_WP_Pin,
            GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOB, IMU_NCS_Pin | LED_YELLOW_Pin | LED_GREEN_Pin,
            GPIO_PIN_RESET);

    /*Configure GPIO pins : FLASH_CS_Pin FLASH_HOLD_RST_Pin FLASH_WP_Pin */
    GPIO_InitStruct.Pin = FLASH_CS_Pin | FLASH_HOLD_RST_Pin | FLASH_WP_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pins : IMU_NCS_Pin LED_YELLOW_Pin LED_GREEN_Pin */
    GPIO_InitStruct.Pin = IMU_NCS_Pin | LED_YELLOW_Pin | LED_GREEN_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /*Configure GPIO pin : IMU_INT1_Pin */
    GPIO_InitStruct.Pin = IMU_INT1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(IMU_INT1_GPIO_Port, &GPIO_InitStruct);

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
