/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Floor Sense firmware, step 3: MPU9250/6500 bring-up
  ******************************************************************************
  * - Finds the motion sensor on I2C1 (PB8 = SCL, PB9 = SDA), reads its ID,
  *   sets +-4 g and +-500 dps, and checks for the MPU9250's magnetometer.
  * - Reads the sensor 100 times a second and sends each sample to the Pi
  *   as an IMU frame. (1 kHz sampling comes later, with DMA sending.)
  * - Heartbeat every second, with imu_ok set while the sensor answers.
  * - Motor commands from the Pi as before; the LED still stands in for motors.
  *
  * LED:  slow blink = sensor OK   fast blink = no sensor   solid = driving
  *
  * Watch these in the debugger (Live Expressions): imu_addr, imu_id, mag_id,
  * i2c_found, i2c_found_n, acc_g, gyr_dps, imu_reads, imu_errors.
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include "floorsense_protocol.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define HEARTBEAT_PERIOD_MS  1000u
#define IMU_PERIOD_MS        10u      /* 100 Hz for bring-up */
#define IMU_RETRY_MS         1000u
#define BLINK_OK_MS          500u
#define BLINK_FAULT_MS       100u
#define I2C_TIMEOUT_MS       10u

/* MPU9250 / MPU6500 registers */
#define MPU_REG_SMPLRT_DIV   0x19u
#define MPU_REG_CONFIG       0x1Au
#define MPU_REG_GYRO_CFG     0x1Bu
#define MPU_REG_ACCEL_CFG    0x1Cu
#define MPU_REG_ACCEL_CFG2   0x1Du
#define MPU_REG_INT_PIN_CFG  0x37u
#define MPU_REG_ACCEL_XOUT_H 0x3Bu
#define MPU_REG_PWR_MGMT_1   0x6Bu
#define MPU_REG_PWR_MGMT_2   0x6Cu
#define MPU_REG_WHO_AM_I     0x75u
#define AK8963_ADDR          0x0Cu    /* magnetometer inside a real MPU9250 */
#define AK8963_REG_WIA       0x00u    /* reads 0x48 */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static uint8_t tx_buf[FS_MAX_FRAME];
static uint8_t tx_seq;

/* Written by the UART interrupt, read by the main loop */
static fs_parser_t rx_parser;
static uint8_t rx_byte;
static volatile int16_t cmd_left, cmd_right;
static volatile uint32_t cmd_time_ms;
static volatile uint8_t cmd_seen;

/* Sensor state. Global and volatile so the debugger can show them live. */
volatile uint8_t imu_ok;          /* 1 while the sensor answers */
volatile uint8_t imu_addr;        /* 0x68 or 0x69, 0 = not found */
volatile uint8_t imu_id;          /* 0x71 MPU9250, 0x73 MPU9255, 0x70 MPU6500 */
volatile uint8_t mag_id;          /* 0x48 if the MPU9250 magnetometer answers */
volatile uint8_t i2c_found[8];    /* addresses that answered on the bus */
volatile uint8_t i2c_found_n;
volatile float acc_g[3];          /* latest sample, in g */
volatile float gyr_dps[3];        /* latest sample, in degrees per second */
volatile uint32_t imu_reads, imu_errors;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
static void send_frame(uint8_t type, const void *payload, size_t len);
static int commands_fresh(uint32_t now_ms, int16_t *left, int16_t *right);
static int imu_init(void);
static int imu_read(int16_t s[6]);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Build one frame and send it to the Pi (blocking; fine at 100 Hz). */
static void send_frame(uint8_t type, const void *payload, size_t len)
{
  size_t n = fs_build(tx_buf, sizeof tx_buf, type, tx_seq++, payload, len);
  if (n > 0u)
  {
    HAL_UART_Transmit(&huart2, tx_buf, (uint16_t)n, 20u);
  }
}

/* ---- motion sensor ---- */

static HAL_StatusTypeDef mpu_write(uint8_t reg, uint8_t val)
{
  return HAL_I2C_Mem_Write(&hi2c1, (uint16_t)(imu_addr << 1), reg, I2C_MEMADD_SIZE_8BIT,
                           &val, 1u, I2C_TIMEOUT_MS);
}

static HAL_StatusTypeDef mpu_read(uint8_t reg, uint8_t *buf, uint16_t n)
{
  return HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(imu_addr << 1), reg, I2C_MEMADD_SIZE_8BIT,
                          buf, n, I2C_TIMEOUT_MS);
}

/* List every address that answers, for the debugger. */
static void i2c_scan(void)
{
  uint8_t n = 0u;
  for (uint8_t a = 0x08u; a < 0x78u; a++)
  {
    if (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(a << 1), 1u, 2u) == HAL_OK
        && n < sizeof i2c_found)
    {
      i2c_found[n++] = a;
    }
  }
  i2c_found_n = n;
}

/* Find, reset and configure the sensor. Returns 1 on success. */
static int imu_init(void)
{
  uint8_t v = 0u;

  imu_ok = 0u;
  imu_addr = 0u;
  HAL_I2C_DeInit(&hi2c1);             /* clears a bus left stuck by a failed transfer */
  MX_I2C1_Init();
  i2c_scan();

  if (HAL_I2C_IsDeviceReady(&hi2c1, 0x68u << 1, 3u, I2C_TIMEOUT_MS) == HAL_OK)
  {
    imu_addr = 0x68u;
  }
  else if (HAL_I2C_IsDeviceReady(&hi2c1, 0x69u << 1, 3u, I2C_TIMEOUT_MS) == HAL_OK)
  {
    imu_addr = 0x69u;
  }
  else
  {
    return 0;
  }

  if (mpu_read(MPU_REG_WHO_AM_I, &v, 1u) != HAL_OK) return 0;
  imu_id = v;

  if (mpu_write(MPU_REG_PWR_MGMT_1, 0x80u) != HAL_OK) return 0;   /* reset */
  HAL_Delay(100u);
  if (mpu_write(MPU_REG_PWR_MGMT_1, 0x01u) != HAL_OK) return 0;   /* clock from gyro PLL */
  HAL_Delay(10u);
  if (mpu_write(MPU_REG_PWR_MGMT_2, 0x00u) != HAL_OK) return 0;   /* all axes on */
  if (mpu_write(MPU_REG_SMPLRT_DIV, 0x00u) != HAL_OK) return 0;   /* 1 kHz internal rate */
  if (mpu_write(MPU_REG_CONFIG, 0x01u) != HAL_OK) return 0;       /* gyro filter 184 Hz */
  if (mpu_write(MPU_REG_GYRO_CFG, 0x08u) != HAL_OK) return 0;     /* +-500 dps */
  if (mpu_write(MPU_REG_ACCEL_CFG, 0x08u) != HAL_OK) return 0;    /* +-4 g */
  if (mpu_write(MPU_REG_ACCEL_CFG2, 0x01u) != HAL_OK) return 0;   /* accel filter 218 Hz */
  if (mpu_write(MPU_REG_INT_PIN_CFG, 0x02u) != HAL_OK) return 0;  /* bypass: expose magnetometer */

  /* Read the ranges back: a fake or damaged chip often fails here */
  if (mpu_read(MPU_REG_GYRO_CFG, &v, 1u) != HAL_OK || (v & 0x18u) != 0x08u) return 0;
  if (mpu_read(MPU_REG_ACCEL_CFG, &v, 1u) != HAL_OK || (v & 0x18u) != 0x08u) return 0;

  HAL_Delay(10u);
  v = 0u;
  if (HAL_I2C_Mem_Read(&hi2c1, AK8963_ADDR << 1, AK8963_REG_WIA, I2C_MEMADD_SIZE_8BIT,
                       &v, 1u, I2C_TIMEOUT_MS) == HAL_OK)
  {
    mag_id = v;
  }
  else
  {
    mag_id = 0u;
  }
  i2c_scan();                         /* again, now that the magnetometer may be visible */

  imu_ok = 1u;
  return 1;
}

/* Read one sample: accel x,y,z then gyro x,y,z, raw. Returns 1 on success. */
static int imu_read(int16_t s[6])
{
  uint8_t b[14];
  if (mpu_read(MPU_REG_ACCEL_XOUT_H, b, sizeof b) != HAL_OK)
  {
    imu_errors++;
    return 0;
  }
  for (int i = 0; i < 3; i++)
  {
    s[i] = (int16_t)(((uint16_t)b[2 * i] << 8) | b[2 * i + 1]);           /* accel */
    s[3 + i] = (int16_t)(((uint16_t)b[8 + 2 * i] << 8) | b[9 + 2 * i]);   /* gyro, after temp */
    acc_g[i] = (float)s[i] / (float)FS_ACCEL_LSB_PER_G;
    gyr_dps[i] = (float)s[3 + i] / FS_GYRO_LSB_PER_DPS;
  }
  imu_reads++;
  return 1;
}

/* ---- motor commands from the Pi ---- */

/* Copy the latest command; returns 1 if it arrived less than FS_CMD_TIMEOUT_MS ago. */
static int commands_fresh(uint32_t now_ms, int16_t *left, int16_t *right)
{
  __disable_irq();                  /* read left, right and time as one consistent set */
  int16_t l = cmd_left, r = cmd_right;
  uint32_t t = cmd_time_ms;
  uint8_t seen = cmd_seen;
  __enable_irq();

  if (!seen || (now_ms - t) >= FS_CMD_TIMEOUT_MS)
  {
    *left = 0;
    *right = 0;
    return 0;
  }
  *left = l;
  *right = r;
  return 1;
}

/* One byte arrived from the Pi: feed the parser, keep MOTOR commands, re-arm. */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance != USART2)
  {
    return;
  }
  if (fs_parse_byte(&rx_parser, rx_byte) && rx_parser.type == FS_MSG_MOTOR
      && rx_parser.len >= sizeof(fs_motor_t))
  {
    fs_motor_t m;
    memcpy(&m, rx_parser.payload, sizeof m);
    if (m.left > FS_MOTOR_MAX) m.left = FS_MOTOR_MAX;
    if (m.left < -FS_MOTOR_MAX) m.left = -FS_MOTOR_MAX;
    if (m.right > FS_MOTOR_MAX) m.right = FS_MOTOR_MAX;
    if (m.right < -FS_MOTOR_MAX) m.right = -FS_MOTOR_MAX;
    cmd_left = m.left;
    cmd_right = m.right;
    cmd_time_ms = HAL_GetTick();
    cmd_seen = 1u;
  }
  HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
}

/* A UART error (noise, overrun) can stop reception; start it again. */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
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
  uint32_t last_hb = 0u, last_blink = 0u, last_imu = 0u, last_try = 0u;
  uint8_t fail_streak = 0u;
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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart2, &rx_byte, 1);   /* start listening for the Pi */
  imu_init();
  last_try = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t now = HAL_GetTick();
    int16_t left, right;
    int fresh = commands_fresh(now, &left, &right);

    /* Sensor: sample at 100 Hz, or retry finding it once a second */
    if (imu_ok && now - last_imu >= IMU_PERIOD_MS)
    {
      int16_t s[6];
      last_imu = now;
      if (imu_read(s))
      {
        fs_imu_t m = { now * 1000u, s[0], s[1], s[2], s[3], s[4], s[5] };
        fail_streak = 0u;
        send_frame(FS_MSG_IMU, &m, sizeof m);
      }
      else if (++fail_streak >= 10u)
      {
        imu_ok = 0u;                /* ten misses in a row: treat it as gone */
      }
    }
    else if (!imu_ok && now - last_try >= IMU_RETRY_MS)
    {
      last_try = now;
      fail_streak = 0u;
      imu_init();
    }

    /* LED stands in for the motors; its blink speed shows the sensor state */
    if (left != 0 || right != 0)
    {
      HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
    }
    else if (now - last_blink >= (imu_ok ? BLINK_OK_MS : BLINK_FAULT_MS))
    {
      last_blink = now;
      HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    }

    if (now - last_hb >= HEARTBEAT_PERIOD_MS)
    {
      fs_heartbeat_t hb;
      last_hb = now;
      hb.t_us = now * 1000u;
      hb.flags = (uint16_t)((fresh ? FS_FLAG_MOTORS_ARMED : FS_FLAG_CMD_TIMEOUT)
                            | (imu_ok ? FS_FLAG_IMU_OK : 0u));
      send_frame(FS_MSG_HEARTBEAT, &hb, sizeof hb);
    }
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00F12981;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

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
