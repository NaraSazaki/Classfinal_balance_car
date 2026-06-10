#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"
#include "usbd_cdc_if.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

/* ===================== MPU6050 ?? ===================== */

#define MPU6050_ADDR           0xD0

#define MPU6050_SMPRT_DIV      0x19
#define MPU6050_WHO_AM_I       0x75
#define MPU6050_CONFIG         0x1A
#define MPU6050_GYRO_CONFIG    0x1B
#define MPU6050_ACCEL_CONFIG   0x1C
#define MPU6050_ACCEL_XOUT_H   0x3B
#define MPU6050_PWR_MGMT_1     0x6B

typedef struct _MPU6050
{
    short acc_x_raw;
    short acc_y_raw;
    short acc_z_raw;
    short temperature_raw;
    short gyro_x_raw;
    short gyro_y_raw;
    short gyro_z_raw;

    float acc_x;
    float acc_y;
    float acc_z;
    float temperature;
    float gyro_x;
    float gyro_y;
    float gyro_z;
} Struct_MPU6050;

Struct_MPU6050 MPU6050;

/* ===================== ????? ===================== */

#define PWM_MAX        100
#define PWM_MIN        10
#define FALL_ANGLE     35.0f
#define LOOP_TIME_MS   5





float Kp = 20.0f;
float Ki = 0.0f;
float Kd = 1.0f;

float target_angle = 15.0f;
float angle = 0.0f;
float gyro_offset = 0.0f;
float integral = 0.0f;
float last_error = 0.0f;

char msg[128];

/* ===================== ???? ===================== */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK |
                                  RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 |
                                  RCC_CLOCKTYPE_PCLK2;

    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
    PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;

    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
        Error_Handler();
    }
}

void Error_Handler(void)
{
    __disable_irq();

    while (1)
    {
    }
}



static void MPU6050_Writebyte(uint8_t reg_addr, uint8_t val);
static void MPU6050_Readbyte(uint8_t reg_addr, uint8_t *data);
static void MPU6050_Readbytes(uint8_t reg_addr, uint8_t len, uint8_t *data);
static void MPU6050_Initialization(void);
static void MPU6050_ProcessData(Struct_MPU6050 *mpu6050);
static void MPU6050_Calibrate(void);
static float MPU6050_GetBalanceAngle(float dt);

static void Motor_Left_SetSpeed(int speed);
static void Motor_Right_SetSpeed(int speed);
static void Motor_SetBoth(int left, int right);
static void Motor_Stop(void);

/* ===================== UART Debug ===================== */

static void Debug_SendString(char *str)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)str, strlen(str), 100);
}
/* ===================== MPU6050 ?? ===================== */

static void MPU6050_Writebyte(uint8_t reg_addr, uint8_t val)
{
    HAL_I2C_Mem_Write(&hi2c1,
                      MPU6050_ADDR,
                      reg_addr,
                      I2C_MEMADD_SIZE_8BIT,
                      &val,
                      1,
                      100);
}

static void MPU6050_Readbyte(uint8_t reg_addr, uint8_t *data)
{
    HAL_I2C_Mem_Read(&hi2c1,
                     MPU6050_ADDR,
                     reg_addr,
                     I2C_MEMADD_SIZE_8BIT,
                     data,
                     1,
                     100);
}

static void MPU6050_Readbytes(uint8_t reg_addr, uint8_t len, uint8_t *data)
{
    HAL_I2C_Mem_Read(&hi2c1,
                     MPU6050_ADDR,
                     reg_addr,
                     I2C_MEMADD_SIZE_8BIT,
                     data,
                     len,
                     100);
}

static void MPU6050_Initialization(void)
{
    HAL_Delay(100);

    // ?? MPU6050
    MPU6050_Writebyte(MPU6050_PWR_MGMT_1, 0x00);
    HAL_Delay(100);

    // Sample Rate = 1kHz / (1 + 9) = 100Hz
    MPU6050_Writebyte(MPU6050_SMPRT_DIV, 0x09);

    // DLPF ??
    MPU6050_Writebyte(MPU6050_CONFIG, 0x03);

    // Gyro ±250 dps
    MPU6050_Writebyte(MPU6050_GYRO_CONFIG, 0x00);

    // Accel ±2g
    MPU6050_Writebyte(MPU6050_ACCEL_CONFIG, 0x00);
}

static void MPU6050_ProcessData(Struct_MPU6050 *mpu6050)
{
    uint8_t data[14];

    MPU6050_Readbytes(MPU6050_ACCEL_XOUT_H, 14, data);

    mpu6050->acc_x_raw = (short)((data[0] << 8) | data[1]);
    mpu6050->acc_y_raw = (short)((data[2] << 8) | data[3]);
    mpu6050->acc_z_raw = (short)((data[4] << 8) | data[5]);

    mpu6050->temperature_raw = (short)((data[6] << 8) | data[7]);

    mpu6050->gyro_x_raw = (short)((data[8] << 8) | data[9]);
    mpu6050->gyro_y_raw = (short)((data[10] << 8) | data[11]);
    mpu6050->gyro_z_raw = (short)((data[12] << 8) | data[13]);

    mpu6050->acc_x = mpu6050->acc_x_raw / 16384.0f;
    mpu6050->acc_y = mpu6050->acc_y_raw / 16384.0f;
    mpu6050->acc_z = mpu6050->acc_z_raw / 16384.0f;

    mpu6050->gyro_x = mpu6050->gyro_x_raw / 131.0f;
    mpu6050->gyro_y = mpu6050->gyro_y_raw / 131.0f;
    mpu6050->gyro_z = mpu6050->gyro_z_raw / 131.0f;

    mpu6050->temperature = mpu6050->temperature_raw / 340.0f + 36.53f;
}

static void MPU6050_Calibrate(void)
{
    float sum = 0.0f;

    Debug_SendString("Calibrating MPU6050...\r\n");

    for (int i = 0; i < 500; i++)
    {
        MPU6050_ProcessData(&MPU6050);
        sum += (float)MPU6050.gyro_y_raw;
        HAL_Delay(2);
    }

    gyro_offset = sum / 500.0f;

    snprintf(msg, sizeof(msg), "gyro_offset=%.2f\r\n", gyro_offset);
    Debug_SendString(msg);
}

static float MPU6050_GetBalanceAngle(float dt)
{
    MPU6050_ProcessData(&MPU6050);

    float ax = MPU6050.acc_x;
    float az = MPU6050.acc_z;

    float gyro_y = ((float)MPU6050.gyro_y_raw - gyro_offset) / 131.0f;

    float acc_angle = atan2f(ax, az) * 57.2958f;

    angle = 0.98f * (angle + gyro_y * dt) + 0.02f * acc_angle;

    return angle;
}

/* ===================== ???? ===================== */

static void Motor_Left_SetSpeed(int speed)
{
    if (speed > PWM_MAX) speed = PWM_MAX;
    if (speed < -PWM_MAX) speed = -PWM_MAX;

    if (speed > 0)
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    }
    else if (speed < 0)
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
        speed = -speed;
    }
    else
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    }

    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim2);
    uint32_t pulse = speed * (arr + 1) / 100;

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse);
}

static void Motor_Right_SetSpeed(int speed)
{
    if (speed > PWM_MAX) speed = PWM_MAX;
    if (speed < -PWM_MAX) speed = -PWM_MAX;

    if (speed > 0)
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
    }
    else if (speed < 0)
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);
        speed = -speed;
    }
    else
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
    }

    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim2);
    uint32_t pulse = speed * (arr + 1) / 100;

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pulse);
}

static void Motor_SetBoth(int left, int right)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

    if (left > 0 && left < PWM_MIN) left = PWM_MIN;
    if (left < 0 && left > -PWM_MIN) left = -PWM_MIN;

    if (right > 0 && right < PWM_MIN) right = PWM_MIN;
    if (right < 0 && right > -PWM_MIN) right = -PWM_MIN;

    Motor_Left_SetSpeed(left);
    Motor_Right_SetSpeed(right);
}

static void Motor_Stop(void)
{
    Motor_Left_SetSpeed(0);
    Motor_Right_SetSpeed(0);
}

/* ===================== main ===================== */

int main(void)
{
    uint8_t who = 0;

    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();
    MX_USART1_UART_Init();
    MX_I2C1_Init();
		
    HAL_Delay(1000);

    Debug_SendString("Balance Car Start\r\n");

    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET); // STBY
    Motor_Stop();

    MPU6050_Readbyte(MPU6050_WHO_AM_I, &who);
    snprintf(msg, sizeof(msg), "WHO=0x%02X\r\n", who);
    Debug_SendString(msg);

    MPU6050_Initialization();
    HAL_Delay(500);

    MPU6050_Calibrate();

    uint32_t last_time = HAL_GetTick();
    uint32_t last_print = HAL_GetTick();

    while (1)
    {
        uint32_t now = HAL_GetTick();

        if (now - last_time >= LOOP_TIME_MS)
        {
            float dt = (now - last_time) / 1000.0f;
            last_time = now;

            float current_angle = MPU6050_GetBalanceAngle(dt);

            if (current_angle > FALL_ANGLE || current_angle < -FALL_ANGLE)
            {
                Motor_Stop();
                integral = 0;
                last_error = 0;
                continue;
            }

            float error = target_angle - current_angle;

            integral += error * dt;

            if (integral > 100) integral = 100;
            if (integral < -100) integral = -100;

            float derivative = (error - last_error) / dt;
            last_error = error;

            float output = Kp * error + Ki * integral + Kd * derivative;

            if (output > PWM_MAX) output = PWM_MAX;
            if (output < -PWM_MAX) output = -PWM_MAX;

            int pwm = (int)output;

            Motor_SetBoth(pwm, pwm);

            if (now - last_print >= 100)
            {
                last_print = now;

                snprintf(msg, sizeof(msg),
                         "angle=%.2f pwm=%d err=%.2f\r\n",
                         current_angle,
                         pwm,
                         error);

                Debug_SendString(msg);
            }
									HAL_UART_Transmit(&huart1,
                  (uint8_t*)"START\r\n",
                  7,
                  100);

					Debug_SendString(msg);
        }
				
				
    }
}