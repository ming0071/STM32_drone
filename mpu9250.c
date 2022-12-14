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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// AK8963
#define AK8963_ADDRESS 0x0C << 1
#define AK8963_WHO_AM_I 0x00 // should return 0x48
#define AK8963_INFO 0x01
#define AK8963_ST1 0x02 // data ready status bit 0
#define AK8963_XOUT_L 0x03
#define AK8963_XOUT_H 0x04
#define AK8963_YOUT_L 0x05
#define AK8963_YOUT_H 0x06
#define AK8963_ZOUT_L 0x07
#define AK8963_ZOUT_H 0x08
#define AK8963_ST2 0x09
#define AK8963_CNTL1 0x0A  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define AK8963_ASTC 0x0C   // Self test control
#define AK8963_I2CDIS 0x0F // I2C disable
#define AK8963_ASAX 0x10   // Fuse ROM x-axis sensitivity adjustment value
#define AK8963_ASAY 0x11   // Fuse ROM y-axis sensitivity adjustment value
#define AK8963_ASAZ 0x12   // Fuse ROM z-axis sensitivity adjustment value

// MPU9250 ACCEL
#define ACCEL_CONFIG 0x1C   // meansure range of ACCEL
#define ACCEL_CONFIG_2 0x1D // Accelerometer Data Rates and Bandwidths
#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40

// MPU9250 GYRO
#define SMPLRT_DIV 0x19  // GYRO sampling rate
#define GYRO_CONFIG 0x1B // meansure range of GYRO
#define GYRO_XOUT_H 0x43
#define GYRO_XOUT_L 0x44
#define GYRO_YOUT_H 0x45
#define GYRO_YOUT_L 0x46
#define GYRO_ZOUT_H 0x47
#define GYRO_ZOUT_L 0x48

// MPU9250
#define CONFIG 0x1A
#define FIFO_EN 0x23
#define INT_PIN_CFG 0x37
#define INT_ENABLE 0x38
#define USER_CTRL 0x6A
#define PWR_MGMT_1 0x6B
#define PWR_MGMT_2 0x6C
#define MPU9250_ADDRESS 0x68 << 1
#define WHO_AM_I_MPU9250 0x75 // Should return 0x71

// variables
#define DEG2RAD 0.017453293f
#define RAD2DEG 57.29578f

short ACCELX;
short ACCELY;
short ACCELZ;
short GYROX;
short GYROY;
short GYROZ;
short MagX;
short MagY;
short MagZ;

float Kp = 2.0f; /*比�?��?��??*/
float Ki = 0.1f; /*积�?��?��??*/
float exInt = 0.0f;
float eyInt = 0.0f;
float ezInt = 0.0f; /*积�?�误差累�???????????*/

static float q0 = 1.0f; /*??��?�数*/
static float q1 = 0.0f;
static float q2 = 0.0f;
static float q3 = 0.0f;
volatile uint32_t last_update, now_update;
float yaw;
float ref_yaw = 0.0f;
float pitch;
float roll;

float ASA[3] = {0}; // ASA[0] = ASAX , ASA[1] = ASAY , ASA[2] = ASAZ

struct Axisf
{
    float x;
    float y;
    float z;
};
struct MPU9250_t
{
    struct Axisf gyro;
    struct Axisf acc;
    struct Axisf mag;
    struct Axisf attitude;
};
struct MPU9250_t mpu9250;

// Motor Controll Variables
struct PID
{
    float Kp;
    float Ki;
    float Kd;
    float pid;
    float pre_error;
    float prev_degree;
};
struct Euler_angle
{
    struct PID ro11;
    struct PID pitch;
    struct PID yaw;
};
struct Euler_angle euler;

float error, pre_error = 0.0f;
float Pterm, Iterm, Dterm = 0.0f;
float motor_1, motor_2, motor_3, motor_4 = 0.0f;
float I_add = 0.0f, I_max = 20.0f, I_min = -20.0f;
float throttle = 1410.0f;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
DMA_HandleTypeDef hdma_tim1_ch1;
DMA_HandleTypeDef hdma_tim1_ch2;

/* USER CODE BEGIN PV */

const uint16_t i2c_timeoutB = 100;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
void MPU9250_Init();
void GetGyroBiasData();
void GetAccBiasData();
void GetMagBiasData();
void GetIMUData();
void GetEulerAngle();
void ESCcalibration();
float PID(float ref, float degree, float *pre_error, float *prev_degree, float kp, float ki, float kd);
float motor_limit(float motor, int upper, int lower);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// printf program
__attribute__((weak)) int _write(int file, char *ptr, int len)
{
    int DataIdx;
    for (DataIdx = 0; DataIdx < len; DataIdx++)
    {
        ITM_SendChar(*ptr++);
    }
    return len;
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
    MX_I2C1_Init();
    MX_TIM1_Init();
    /* USER CODE BEGIN 2 */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    MPU9250_Init();

    // roll
    euler.ro11.Kp = 0.56; // no setting sampling time
    euler.ro11.Ki = 0.00005;
    euler.ro11.Kd = 12.4;
    euler.ro11.pre_error = 0;
    // pitch
    euler.pitch.Kp = 0.56; // no setting sampling time
    euler.pitch.Ki = 0.00005;
    euler.pitch.Kd = 12.4;
    euler.pitch.pre_error = 0;
    // yaw
    euler.yaw.Kp = 0.8;
    euler.yaw.Ki = 0.12;
    euler.yaw.Kd = 4.8;
    euler.yaw.pre_error = 0;
    // order
    short stateD1, stateD2, stateD3 = 0;
    short cmpD2, cmpD3 = 0;

    HAL_Delay(5000); // wait 10s start
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */

        // GetGyroBiasData();
        // GetAccBiasData();
        // GetMagBiasData();
        GetIMUData();
        GetEulerAngle();

        // ESCcalibration();
        stateD1 = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_2);
        stateD2 = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3);
        stateD3 = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_4);

        if (stateD1 == 0)
        {
            motor_1 = 1000;
            motor_2 = 1000;
            motor_3 = 1000;
            motor_4 = 1000;
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, motor_1); // PE9  	(V)
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, motor_2); // PE11	(V)
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, motor_3); // PE13
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, motor_4); // PE14
            printf("motor1: %f \r\n", motor_1);
            printf("motor2: %f \r\n", motor_2);
            printf("motor3: %f \r\n", motor_3);
            printf("motor4: %f \r\n", motor_4);
            printf("D1: %d\r\n", stateD1);
            ref_yaw = yaw;
            printf("ref of yaw: %f\r\n", ref_yaw);
        }
        // PID_roll
        else if (stateD1 == 1)
        {
            euler.ro11.pid = PID(0.0f, roll, &euler.ro11.pre_error, &euler.ro11.prev_degree, euler.ro11.Kp, euler.ro11.Ki, euler.ro11.Kd);
            printf("Dterm: %f \r\n", Dterm);
            euler.pitch.pid = PID(0.0f, pitch, &euler.pitch.pre_error, &euler.pitch.prev_degree, euler.pitch.Kp, euler.pitch.Ki, euler.pitch.Kd);
            euler.yaw.pid = PID(ref_yaw, yaw, &euler.yaw.pre_error, &euler.yaw.prev_degree, euler.yaw.Kp, euler.yaw.Ki, euler.yaw.Kd);
            motor_1 = throttle - euler.ro11.pid + euler.yaw.pid + 12; // TODO: + or -
            motor_2 = throttle + euler.ro11.pid + euler.yaw.pid;      // TODO: + or -
            motor_3 = throttle + euler.pitch.pid - euler.yaw.pid;     // TODO: + or -
            motor_4 = throttle - euler.pitch.pid - euler.yaw.pid;     // TODO: + or -
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, motor_1);    // PE9
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, motor_2);    // PE11
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, motor_3);    // PE13
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, motor_4);    // PE14
            printf("motor1: %f \r\n", motor_1);
            printf("motor2: %f \r\n", motor_2);
            printf("motor3: %f \r\n", motor_3);
            printf("motor4: %f \r\n", motor_4);
            printf("roll.pid: %f\r\n", euler.ro11.pid);
            if (cmpD2 != stateD2) // 110
            {
                throttle = throttle + 10;
            }
            if (cmpD3 != stateD3) // 101
            {
                throttle = throttle - 10;
            }
            cmpD2 = stateD2;
            cmpD3 = stateD3;
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
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
    hi2c1.Init.ClockSpeed = 400000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK)
    {
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
static void MX_TIM1_Init(void)
{

    /* USER CODE BEGIN TIM1_Init 0 */

    /* USER CODE END TIM1_Init 0 */

    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

    /* USER CODE BEGIN TIM1_Init 1 */

    /* USER CODE END TIM1_Init 1 */
    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 168 - 1;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = 20000 - 1;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
    {
        Error_Handler();
    }
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
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{

    /* DMA controller clock enable */
    __HAL_RCC_DMA2_CLK_ENABLE();

    /* DMA interrupt init */
    /* DMA2_Stream1_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
    /* DMA2_Stream2_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
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
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);

    /*Configure GPIO pins : PE2 PE3 PE4 */
    GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /*Configure GPIO pin : PA0 */
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pin : PA1 */
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */
void MPU9250_Init()
{
    uint8_t readData;
    uint8_t writeData;
    unsigned char pdata;

    // Reset MPU
    pdata = 0x80;
    HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, PWR_MGMT_1, 1, &pdata, 1, HAL_MAX_DELAY);
    HAL_I2C_IsDeviceReady(&hi2c1, MPU9250_ADDRESS, 10, HAL_MAX_DELAY);
    HAL_Delay(100); // wait for Reset

    // Check MPU and Mag---------------------------------------------------------------------------------------------------
    // read MPU9255 WHOAMI
    HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDRESS, WHO_AM_I_MPU9250, 1, &readData, 1, i2c_timeoutB);
    printf("MPU WHO AM I is (Must return 113): %d\r\n", readData);

    // Actually we don't need this step cause the reset value of the register 106 is 0x00
    writeData = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, USER_CTRL, 1, &writeData, 1, i2c_timeoutB);
    HAL_Delay(12); // wait for Mag setting

    // enable Mag bypass
    writeData = 0x02;
    HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, INT_PIN_CFG, 1, &writeData, 1, i2c_timeoutB);
    HAL_Delay(12); // wait for Mag setting

    // read AK8963 WHOAMI
    HAL_I2C_Mem_Read(&hi2c1, AK8963_ADDRESS, AK8963_WHO_AM_I, 1, &readData, 1, i2c_timeoutB);
    HAL_Delay(12);
    printf("MAG WHO AM I is (Must return 72): %d\r\n\n", readData);
    printf("------------------------------------------------\r\n");

    // MPU9250 Setting---------------------------------------------------------------------------------------
    // check device and i2c is ready
    HAL_I2C_IsDeviceReady(&hi2c1, MPU9250_ADDRESS, 10, HAL_MAX_DELAY);
    HAL_I2C_GetState(&hi2c1);

    // awake MPU & set the clock reference to X axis gyroscope to get a better accuracy
    pdata = 0x01;
    HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, PWR_MGMT_1, 1, &pdata, 1, HAL_MAX_DELAY);

    // initialize meansure range(accel & gyro & mag)--------------------------------------------------
    // ACC
    pdata = 01 << 3; // Accel Full Scale Select ±4g (01)
    HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, ACCEL_CONFIG, 1, &pdata, 1, i2c_timeoutB);
    HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDRESS, ACCEL_CONFIG, 1, &readData, 1, i2c_timeoutB);
    printf("ACCEL meansure range : %d\r\n", readData);

    // GYRO
    pdata = 01 << 3; // Gyro Full Scale Select ±500dps (01)
    HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, GYRO_CONFIG, 1, &pdata, 1, i2c_timeoutB);
    HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDRESS, GYRO_CONFIG, 1, &readData, 1, i2c_timeoutB);
    printf("GYRO meansure range : %d\r\n", readData);

    // Set gyro sample rate to 1 kHz
    pdata = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, SMPLRT_DIV, 1, &pdata, 1, HAL_MAX_DELAY);

    // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
    pdata = 0x02;
    HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, ACCEL_CONFIG_2, 1, &pdata, 1, HAL_MAX_DELAY);

    // Set gyro sample rate to 1 kHz and DLPF to 92 Hz
    pdata = 0x02;
    HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, CONFIG, 1, &pdata, 1, HAL_MAX_DELAY);

    // turn off all interrupt
    pdata = 0;
    HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, INT_ENABLE, 1, &pdata, 1, HAL_MAX_DELAY);

    // could use gyro and accel
    pdata = 0;
    HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, PWR_MGMT_2, 1, &pdata, 1, HAL_MAX_DELAY);

    // Mag---------------------------------------------------------------------------------
    // Power down magnetometer
    writeData = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, AK8963_ADDRESS, AK8963_CNTL1, 1, &writeData, 1, i2c_timeoutB);
    HAL_Delay(12);

    // Enter Fuse ROM access mode
    writeData = 0x1F;
    HAL_I2C_Mem_Write(&hi2c1, AK8963_ADDRESS, AK8963_CNTL1, 1, &writeData, 1, i2c_timeoutB);
    HAL_Delay(12);

    // Read the x-, y-, and z-axis calibration values
    uint8_t rawMagCalData[3];
    HAL_I2C_Mem_Read(&hi2c1, AK8963_ADDRESS, AK8963_ASAX, 1, &rawMagCalData[0], 3, i2c_timeoutB);
    ASA[0] = (float)(rawMagCalData[0] - 128.) / 256. + 1.; // Return x-axis sensitivity adjustment values, etc.
    ASA[1] = (float)(rawMagCalData[1] - 128.) / 256. + 1.;
    ASA[2] = (float)(rawMagCalData[2] - 128.) / 256. + 1.;

    printf("Mag cal off X: %f\r\n", ASA[0]); // ASA[0] = ASAX
    printf("Mag cal off Y: %f\r\n", ASA[1]); // ASA[1] = ASAY
    printf("Mag cal off Z: %f\r\n", ASA[2]); // ASA[2] = ASAZ
    HAL_Delay(12);

    // Power down magnetometer
    writeData = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, AK8963_ADDRESS, AK8963_CNTL1, 1, &writeData, 1, i2c_timeoutB);
    HAL_Delay(12);

    // Set magnetometer data resolution and sample ODR
    // set the Magnetometer to continuous mode 2�???????????100Hz) and 16-bit output
    writeData = 0x16;
    printf("writeData: %x\r\n", writeData);
    HAL_I2C_Mem_Write(&hi2c1, AK8963_ADDRESS, AK8963_CNTL1, 1, &writeData, 1, i2c_timeoutB);
    HAL_Delay(12);

    printf("------------------------------------------------\r\n");
}
void GetGyroBiasData()
{
#define test_times 2

    int i = 0;
    short avergyro[3] = {0};
    int32_t sumgyro[3] = {0};
    uint8_t rawGYROData[6] = {0};

    printf("test times %d (about 1 min) \n\n", test_times);

    for (i = 0; i < test_times; i++)
    {
        // Read GYRO data
        HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDRESS, GYRO_XOUT_H, 1, rawGYROData, 6, i2c_timeoutB);
        GYROX = (rawGYROData[0] << 8) | rawGYROData[1];
        GYROY = (rawGYROData[2] << 8) | rawGYROData[3];
        GYROZ = (rawGYROData[4] << 8) | rawGYROData[5];

        sumgyro[0] += GYROX;
        sumgyro[1] += GYROY;
        sumgyro[2] += GYROZ;
    }

    for (i = 0; i < 3; i++)
    {
        avergyro[i] = (short)(sumgyro[i] / test_times);
        printf("gyro %d = %d\r\n", i, avergyro[i]);
    }
    printf("-------------------------------------------\n");
}
void GetAccBiasData()
{
    uint8_t rawACCELData[6] = {0};

    // Read ACCEL data.  Actually we don't need do this
    HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDRESS, ACCEL_XOUT_H, 1, &rawACCELData[0], 6, i2c_timeoutB);
    ACCELX = (rawACCELData[0] << 8) | rawACCELData[1];
    ACCELY = (rawACCELData[2] << 8) | rawACCELData[3];
    ACCELZ = (rawACCELData[4] << 8) | rawACCELData[5];

    printf("%d,%d,%d\r\n", ACCELX, ACCELY, ACCELZ);
}
void GetMagBiasData()
{
    uint8_t readData;

    HAL_I2C_Mem_Read(&hi2c1, AK8963_ADDRESS, AK8963_ST1, 1, &readData, 1, i2c_timeoutB);
    HAL_Delay(12);
    if ((readData & 0x01) == 0x01) // check Data ready (DRDY is 1)
    {
        // Read Mag data
        uint8_t rawMagData[7];
        HAL_I2C_Mem_Read(&hi2c1, AK8963_ADDRESS, AK8963_XOUT_L, 1, &rawMagData[0], 7, i2c_timeoutB);
        uint8_t c = rawMagData[6]; // ST2

        if (!(c & 0x08)) // check whether the magnetic sensor is overflown (ST2)
        {
            MagX = (rawMagData[1] << 8) | rawMagData[0];
            MagY = (rawMagData[3] << 8) | rawMagData[2];
            MagZ = (rawMagData[5] << 8) | rawMagData[4];

            printf("%d,%d,%d\r\n", MagX, MagY, MagZ);

            // mag_count = 0; // magnetometer can't read too often , 2rd method
            HAL_Delay(12); // at 100 Hz ODR, new mag data is available every 10 ms
        }
    }
    else
    {
        printf("No Data? \r\n");
        printf("------------------------------------------------\r\n");
    }
}
void GetIMUData()
{
#define print_flag 0    // 0-> no printf , 1 ->printf acc, 2 ->printf gyro , 3 ->printf mag
                        //               , 4 ->printf all data
#define position_flag 1 // 0 -> (-),1-> (+)

    static short mag_count = 0;
    // Read ACCEL----------------------------------------------------------------------------------
    // Read ACCEL data
    uint8_t rawACCELData[6];
    HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDRESS, ACCEL_XOUT_H, 1, &rawACCELData[0], 6, i2c_timeoutB);
    ACCELX = (rawACCELData[0] << 8) | rawACCELData[1];
    ACCELY = (rawACCELData[2] << 8) | rawACCELData[3];
    ACCELZ = (rawACCELData[4] << 8) | rawACCELData[5];

    // transfer to real unit(accel)
    float tfa = (float)(4 * 9.80665 / 32768.0);
    mpu9250.acc.x = (float)ACCELX * tfa;
    mpu9250.acc.y = (float)ACCELY * tfa;
    mpu9250.acc.z = (float)ACCELZ * tfa;

    // position
    if (position_flag == 0)
    {
        mpu9250.acc.x *= (-1.0);
        mpu9250.acc.y *= (-1.0);
        mpu9250.acc.z *= (-1.0);
    }

    if (print_flag == 1 || print_flag == 4)
    {
        printf("Accelerometer :\r\n");
        printf("data AX: %d\r\n", ACCELX);
        printf("ACCEL X: %.2f m/s^2\r\n", mpu9250.acc.x);
        printf("data AY: %d\r\n", ACCELY);
        printf("ACCEL Y: %.2f m/s^2\r\n", mpu9250.acc.y);
        printf("data AZ: %d\r\n", ACCELZ);
        printf("ACCEL Z: %.2f m/s^2\r\n\n", mpu9250.acc.z);
    }

    // Read GYRO-----------------------------------------------------------------------------------
    // Read GYRO data
    uint8_t rawGYROData[6] = {0};

    HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDRESS, GYRO_XOUT_H, 1, rawGYROData, 6, i2c_timeoutB);
    GYROX = ((rawGYROData[0] << 8) | rawGYROData[1]) - (-106); // bias Corrected
    GYROY = ((rawGYROData[2] << 8) | rawGYROData[3]) - 36;     // bias Corrected
    GYROZ = ((rawGYROData[4] << 8) | rawGYROData[5]) - 24;     // bias Corrected
    // 500  dps : x = -106 , y = 36 , z = 24
    // 2000 dps : x = -24 , y = 2529 , z = 5

    // transfer to real unit(gyro)
    float tfg = (float)(500.0 / 32768.0);

    mpu9250.gyro.x = (float)GYROX * tfg;
    mpu9250.gyro.y = (float)GYROY * tfg;
    mpu9250.gyro.z = (float)GYROZ * tfg;
    mpu9250.gyro.x *= DEG2RAD;
    mpu9250.gyro.y *= DEG2RAD;
    mpu9250.gyro.z *= DEG2RAD;

    // position
    if (position_flag == 0)
    {
        mpu9250.gyro.x *= (-1.0);
        mpu9250.gyro.y *= (-1.0);
        mpu9250.gyro.z *= (-1.0);
    }

    if (print_flag == 2 || print_flag == 4)
    {
        printf("Gyroscope(Bias Corrected) :\r\n");
        printf("data GX: %d\r\n", GYROX);
        printf("GYRO X: %.4f rad/s\r\n", mpu9250.gyro.x);
        printf("data GY: %d\r\n", GYROY);
        printf("GYRO Y: %.4f rad/s\r\n", mpu9250.gyro.y);
        printf("data GZ: %d\r\n", GYROZ);
        printf("GYRO Z: %.4f rad/s\r\n\n", mpu9250.gyro.z);
    }

    mag_count++;
    uint8_t readData;

    // Read MAG-----------------------------------------------------------------------------------

    HAL_I2C_Mem_Read(&hi2c1, AK8963_ADDRESS, AK8963_ST1, 1, &readData, 1, i2c_timeoutB);
    HAL_Delay(12);
    if ((readData & 0x01) == 0x01) // check Data ready (DRDY is 1)
    {
        // Read Mag data
        uint8_t rawMagData[7];
        HAL_I2C_Mem_Read(&hi2c1, AK8963_ADDRESS, AK8963_XOUT_L, 1, &rawMagData[0], 7, i2c_timeoutB);
        uint8_t c = rawMagData[6]; // ST2

        if (!(c & 0x08)) // check whether the magnetic sensor is overflown (ST2)
        {
            MagX = (rawMagData[1] << 8) | rawMagData[0];
            MagY = (rawMagData[3] << 8) | rawMagData[2];
            MagZ = (rawMagData[5] << 8) | rawMagData[4];

            // Ellipsoid Fit
            MagX = MagX - (-143.22);
            MagY = (MagY - (-17.815)) * 243.21 / 236.71;
            MagZ = (MagZ - (-25.165)) * 243.21 / 223.45;

            // Print to Com port via STLINK
            mpu9250.mag.x = (float)MagY * ASA[1] * (4912.0 / 32760.0);  // ASA[1] = ASAY(4912.0 / 32760.0)
            mpu9250.mag.y = (float)MagX * ASA[0] * (4912.0 / 32760.0);  // ASA[0] = ASAX
            mpu9250.mag.z = (float)-MagZ * ASA[2] * (4912.0 / 32760.0); // ASA[2] = ASAZ

            // position
            if (position_flag == 0)
            {
                mpu9250.mag.x *= (-1.0);
                mpu9250.mag.y *= (-1.0);
                mpu9250.mag.z *= (-1.0);
            }
            if (print_flag == 3 || print_flag == 4)
            {
                printf("Magnetometer(Direction Corrected) :\r\n");
                printf("data MX: %d\r\n", MagY);
                printf("Mag X: %.4f µT\r\n", mpu9250.mag.x);
                printf("data MY: %d\r\n", MagX);
                printf("Mag Y: %.4f µT\r\n", mpu9250.mag.y);
                printf("data MZ: %d\r\n", -MagZ);
                printf("Mag Z: %.4f µT\r\n\n", mpu9250.mag.z);
                printf("------------------------------------------------\r\n");
            }

            // mag_count = 0; // magnetometer can't read too often , 2rd method
            HAL_Delay(20); // at 100 Hz ODR, new mag data is available every 10 ms
        }
    }
    else
    {
        printf("No Data? \r\n");
        printf("------------------------------------------------\r\n");
    }
}
void GetEulerAngle() // use MahonyAHRS
{
    // Auxiliary variables to avoid repeated arithmetic
    float q0q0 = q0 * q0;
    float q1q1 = q1 * q1;
    float q2q2 = q2 * q2;
    float q3q3 = q3 * q3;

    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q0q3 = q0 * q3;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q3 = q2 * q3;

    float normalise;
    float ex, ey, ez;
    float halfT;
    float hx, hy, hz, bx, bz;
    float vx, vy, vz, wx, wy, wz;

    now_update = HAL_GetTick(); // unit : ms
    halfT = ((float)(now_update - last_update) / 2000.0f);
    last_update = now_update;

    // Normalise accelerometer measurement
    if (mpu9250.acc.x != 0 || mpu9250.acc.y != 0 || mpu9250.acc.z != 0)
    {
        normalise = sqrt(mpu9250.acc.x * mpu9250.acc.x + mpu9250.acc.y * mpu9250.acc.y + mpu9250.acc.z * mpu9250.acc.z);
        mpu9250.acc.x /= normalise;
        mpu9250.acc.y /= normalise;
        mpu9250.acc.z /= normalise;
    }

    // Normalise magnetometer measurement
    if (mpu9250.mag.x != 0 || mpu9250.mag.y != 0 || mpu9250.mag.z != 0)
    {
        normalise = sqrt(mpu9250.mag.x * mpu9250.mag.x + mpu9250.mag.y * mpu9250.mag.y + mpu9250.mag.z * mpu9250.mag.z);
        mpu9250.mag.x /= normalise;
        mpu9250.mag.y /= normalise;
        mpu9250.mag.z /= normalise;
    }

    /* 計算磁力計投影到物體座標上的各個分量 */
    hx = 2.0f * (mpu9250.mag.x * (0.5f - q2q2 - q3q3) + mpu9250.mag.y * (q1q2 - q0q3) + mpu9250.mag.z * (q1q3 + q0q2));
    hy = 2.0f * (mpu9250.mag.x * (q1q2 + q0q3) + mpu9250.mag.y * (0.5f - q1q1 - q3q3) + mpu9250.mag.z * (q2q3 - q0q1));
    hz = 2.0f * (mpu9250.mag.x * (q1q3 - q0q2) + mpu9250.mag.y * (q2q3 + q0q1) + mpu9250.mag.z * (0.5f - q1q1 - q2q2));
    bx = sqrt((hx * hx) + (hy * hy));
    bz = hz;

    /* 計算加速度計投影到物體座標上的各個分量*/
    vx = 2.0f * (q1q3 - q0q2);
    vy = 2.0f * (q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;

    /* 處理過後的磁力計新分量 */
    wx = 2.0f * (bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2));
    wy = 2.0f * (bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3));
    wz = 2.0f * (bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2));

    /* 差積誤差累計，用以修正陀螺儀 */
    ex = (mpu9250.acc.y * vz - mpu9250.acc.z * vy) + (mpu9250.mag.y * wz - mpu9250.mag.z * wy);
    ey = (mpu9250.acc.z * vx - mpu9250.acc.x * vz) + (mpu9250.mag.z * wx - mpu9250.mag.x * wz);
    ez = (mpu9250.acc.x * vy - mpu9250.acc.y * vx) + (mpu9250.mag.x * wy - mpu9250.mag.y * wx);

    /* 互補濾波 PI */
    exInt += ex * Ki * halfT;
    eyInt += ey * Ki * halfT;
    ezInt += ez * Ki * halfT;
    mpu9250.gyro.x += Kp * ex + exInt;
    mpu9250.gyro.y += Kp * ey + eyInt;
    mpu9250.gyro.z += Kp * ez + ezInt;

    /* 使用 一階龍格-庫塔 更新四元數 */
    q0 += (-q1 * mpu9250.gyro.x - q2 * mpu9250.gyro.y - q3 * mpu9250.gyro.z) * halfT;
    q1 += (q0 * mpu9250.gyro.x + q2 * mpu9250.gyro.z - q3 * mpu9250.gyro.y) * halfT;
    q2 += (q0 * mpu9250.gyro.y - q1 * mpu9250.gyro.z + q3 * mpu9250.gyro.x) * halfT;
    q3 += (q0 * mpu9250.gyro.z + q1 * mpu9250.gyro.y - q2 * mpu9250.gyro.x) * halfT;

    // Normalise quaternion
    normalise = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 /= normalise;
    q1 /= normalise;
    q2 /= normalise;
    q3 /= normalise;

    /* 四元數 -> 歐拉角 */
    mpu9250.attitude.x = -asinf(-2 * q1 * q3 + 2 * q0 * q2) * RAD2DEG;                                // pitch
    mpu9250.attitude.y = atan2f(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * RAD2DEG; // roll
    mpu9250.attitude.z = atan2f(2 * q1 * q2 + 2 * q0 * q3, -2 * q2 * q2 - 2 * q3 * q3 + 1) * RAD2DEG; // yaw

    yaw = mpu9250.attitude.z;
    pitch = mpu9250.attitude.x;
    roll = mpu9250.attitude.y;

    printf("yaw: %f \r\n", yaw);
    printf("pitch: %f \r\n", pitch);
    printf("roll: %f \r\n\n", roll);
    printf("---------------------------------\n");
}
void ESCcalibration()
{
    int state, pulse = 0;
    state = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0); // KEY_UP
    if (state == 0)
    {
        pulse = 1000;
        printf("%d already\r\n", pulse);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pulse);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pulse);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pulse);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET); // LED0
    }
    HAL_Delay(50);
    if (state == 1) // normal
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET); // LED0
        pulse = 2000;
        printf("%d already\r\n", pulse);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pulse);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pulse);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pulse);
    }
    HAL_Delay(50);
}
float PID(float ref, float degree, float *pre_error, float *prev_degree, float kp, float ki, float kd)
{
    float PID_value = 0;
    error = ref - degree;
    // Proportion
    Pterm = kp * error;
    // Integralc
    I_add = 0.61 * I_add + error;
    *prev_degree = degree;
    Iterm = ki * I_add; // no setting sampling time
    // Differential                     //no setting sampling time
    Dterm = kd * (error - *pre_error); // called by address
    *pre_error = error;
    // Motor Power Calculate
    PID_value = Pterm + Iterm + Dterm;

    if (PID_value > 40)
    {
        PID_value = 40;
    }
    else if (PID_value < -40)
    {
        PID_value = -40;
    }
    return PID_value;
}
float motor_limit(float motor, int upper, int lower)
{
    if (motor > upper)
    {
        motor = upper;
    }
    else if (motor < lower)
    {
        motor = lower;
    }
    return motor;
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
