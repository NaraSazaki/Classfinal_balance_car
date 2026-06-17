#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

#include <math.h>
#include <stdint.h>

/* ===================== MPU6050 ===================== */

#define MPU6050_ADDR           0xD0
#define MPU6050_SMPRT_DIV      0x19
#define MPU6050_WHO_AM_I       0x75
#define MPU6050_CONFIG         0x1A
#define MPU6050_GYRO_CONFIG    0x1B
#define MPU6050_ACCEL_CONFIG   0x1C
#define MPU6050_ACCEL_XOUT_H   0x3B
#define MPU6050_PWR_MGMT_1     0x6B

typedef struct
{
    int16_t acc_x_raw;
    int16_t acc_y_raw;
    int16_t acc_z_raw;
    int16_t temperature_raw;
    int16_t gyro_x_raw;
    int16_t gyro_y_raw;
    int16_t gyro_z_raw;

    float acc_x;
    float acc_y;
    float acc_z;
    float temperature;
    float gyro_x;
    float gyro_y;
    float gyro_z;
} Struct_MPU6050;

Struct_MPU6050 MPU6050;

/* ===================== Parameters ===================== */

#define PWM_MAX        100
#define PWM_MIN        10
#define FALL_ANGLE     32.0f
#define FLAT_FORWARD_FALL_ANGLE 45.0f
#define RAMP_FALL_ANGLE 55.0f
#define LOOP_TIME_MS   5
#define INTEGRAL_LIMIT 18.0f
#define INTEGRAL_ZONE  8.0f
#define BALANCE_TRIM_ANGLE 4.0f
#define RECOVERY_ANGLE 22.0f
#define RECOVERY_PWM_MIN 15
#define CONTROL_PWM_LIMIT 42
#define RAMP_CONTROL_PWM_LIMIT 68
#define PWM_SLEW_STEP 2
#define RAMP_PWM_SLEW_STEP 5
#define GYRO_INTEGRAL_ZONE 35.0f
#define FINE_ANGLE 2.0f
#define FINE_GYRO_ZONE 8.0f
#define DRIVE_MIN_ANGLE 4.0f

#define LINE_TURN_PWM           24
#define LINE_FORWARD_MIN_PWM    30
#define LINE_TURN_MIN_PWM       28
#define LINE_LOST_HOLD_MS       250

#define RAMP_ASSIST_PWM_THRESHOLD   38.0f
#define RAMP_ASSIST_FILTER_ALPHA    0.10f
#define RAMP_ASSIST_RATE_DPS        8.0f
#define RAMP_ASSIST_DECAY_DPS       3.0f
#define RAMP_ASSIST_MAX_ANGLE       15.0f
#define RAMP_ASSIST_ACTIVE_ANGLE    2.0f
#define RAMP_FORWARD_MIN_PWM        55
#define RAMP_TURN_MIN_PWM           75
#define RAMP_FIXED_ANGLE            38.0f
#define RAMP_RELEASE_ANGLE          25.0f
#define RAMP_FIXED_PWM              75

float Kp = 5.5f;
float Ki = 0.18f;
float Kd = 1.8f;

float target_angle = 0.0f;

/* Forward command direction: positive PWM is the physical forward direction. */
float forward_angle = 15.0f;

/* Small forward bias while correcting the line. */
float line_forward_angle = 5.0f;

float angle = 0.0f;
float gyro_offset = 0.0f;
float balance_gyro_y = 0.0f;
float integral = 0.0f;
float last_error = 0.0f;
int last_pwm = 0;
float ramp_assist_angle = 0.0f;
float ramp_pwm_average = 0.0f;

/* TCRT module: indicator LED on = white, LED off = black. */
#define TCRT_BLACK_STATE GPIO_PIN_RESET

/* ===================== Prototypes ===================== */

void SystemClock_Config(void);
void Error_Handler(void);

static void MPU6050_Writebyte(uint8_t reg_addr, uint8_t val);
static void MPU6050_Readbyte(uint8_t reg_addr, uint8_t *data);
static void MPU6050_Readbytes(uint8_t reg_addr, uint8_t len, uint8_t *data);
static uint8_t MPU6050_Initialization(void);
static void MPU6050_ProcessData(Struct_MPU6050 *mpu6050);
static void MPU6050_Calibrate(void);
static float MPU6050_GetBalanceAngle(float dt);

static uint8_t TCRT_Left_IsBlack(void);
static uint8_t TCRT_Right_IsBlack(void);

static void Motor_Left_SetSpeed(int speed);
static void Motor_Right_SetSpeed(int speed);
static void Motor_SetBoth(int left, int right);
static void Motor_Stop(void);

static float Clamp_Float(float value, float min_value, float max_value);
static int Keep_Forward_MinPwm(int pwm, int min_pwm);
static void RampAssist_Update(uint8_t forward_mode, float pwm, float dt);

/* ===================== MPU6050 ===================== */

static void MPU6050_Writebyte(uint8_t reg_addr, uint8_t val)
{
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, reg_addr,
                      I2C_MEMADD_SIZE_8BIT, &val, 1, 100);
}

static void MPU6050_Readbyte(uint8_t reg_addr, uint8_t *data)
{
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, reg_addr,
                     I2C_MEMADD_SIZE_8BIT, data, 1, 100);
}

static void MPU6050_Readbytes(uint8_t reg_addr, uint8_t len, uint8_t *data)
{
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, reg_addr,
                     I2C_MEMADD_SIZE_8BIT, data, len, 100);
}

static uint8_t MPU6050_Initialization(void)
{
    uint8_t who = 0;

    HAL_Delay(50);
    MPU6050_Readbyte(MPU6050_WHO_AM_I, &who);

    if (who != 0x68)
    {
        return 0;
    }

    MPU6050_Writebyte(MPU6050_PWR_MGMT_1, 0x80);
    HAL_Delay(100);

    MPU6050_Writebyte(MPU6050_PWR_MGMT_1, 0x00);
    HAL_Delay(50);

    MPU6050_Writebyte(MPU6050_SMPRT_DIV, 0x09);
    MPU6050_Writebyte(MPU6050_CONFIG, 0x03);
    MPU6050_Writebyte(MPU6050_GYRO_CONFIG, 0x00);
    MPU6050_Writebyte(MPU6050_ACCEL_CONFIG, 0x00);

    return 1;
}

static void MPU6050_ProcessData(Struct_MPU6050 *mpu6050)
{
    uint8_t data[14];

    MPU6050_Readbytes(MPU6050_ACCEL_XOUT_H, 14, data);

    mpu6050->acc_x_raw = (int16_t)((data[0] << 8) | data[1]);
    mpu6050->acc_y_raw = (int16_t)((data[2] << 8) | data[3]);
    mpu6050->acc_z_raw = (int16_t)((data[4] << 8) | data[5]);

    mpu6050->temperature_raw = (int16_t)((data[6] << 8) | data[7]);

    mpu6050->gyro_x_raw = (int16_t)((data[8] << 8) | data[9]);
    mpu6050->gyro_y_raw = (int16_t)((data[10] << 8) | data[11]);
    mpu6050->gyro_z_raw = (int16_t)((data[12] << 8) | data[13]);

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
    float angle_sum = 0.0f;

    for (int i = 0; i < 500; i++)
    {
        MPU6050_ProcessData(&MPU6050);
        sum += (float)MPU6050.gyro_y_raw;
        angle_sum += atan2f(MPU6050.acc_x, MPU6050.acc_z) * 57.2958f;
        HAL_Delay(2);
    }

    gyro_offset = sum / 500.0f;
    target_angle = (angle_sum / 500.0f) + BALANCE_TRIM_ANGLE;
    angle = target_angle;
    integral = 0.0f;
    last_error = 0.0f;
    last_pwm = 0;
    ramp_assist_angle = 0.0f;
    ramp_pwm_average = 0.0f;
}

static float MPU6050_GetBalanceAngle(float dt)
{
    MPU6050_ProcessData(&MPU6050);

    float acc_angle = atan2f(MPU6050.acc_x, MPU6050.acc_z) * 57.2958f;
    balance_gyro_y = ((float)MPU6050.gyro_y_raw - gyro_offset) / 131.0f;

    angle = 0.98f * (angle + balance_gyro_y * dt) + 0.02f * acc_angle;

    return angle;
}

/* ===================== Ramp assist ===================== */

static float Clamp_Float(float value, float min_value, float max_value)
{
    if (value > max_value) return max_value;
    if (value < min_value) return min_value;
    return value;
}

static int Keep_Forward_MinPwm(int pwm, int min_pwm)
{
    int forward_pwm = (forward_angle < 0.0f) ? -min_pwm : min_pwm;

    if (forward_pwm < 0)
    {
        return (pwm > forward_pwm) ? forward_pwm : pwm;
    }

    return (pwm < forward_pwm) ? forward_pwm : pwm;
}

static void RampAssist_Update(uint8_t forward_mode, float pwm, float dt)
{
    float decay_step = RAMP_ASSIST_DECAY_DPS * dt;

    if (!forward_mode)
    {
        ramp_pwm_average = 0.0f;

        if (ramp_assist_angle > decay_step)
        {
            ramp_assist_angle -= decay_step;
        }
        else if (ramp_assist_angle < -decay_step)
        {
            ramp_assist_angle += decay_step;
        }
        else
        {
            ramp_assist_angle = 0.0f;
        }

        return;
    }

    ramp_pwm_average += (pwm - ramp_pwm_average) * RAMP_ASSIST_FILTER_ALPHA;

    if (ramp_pwm_average < -RAMP_ASSIST_PWM_THRESHOLD)
    {
        ramp_assist_angle -= RAMP_ASSIST_RATE_DPS * dt;
    }
    else if (ramp_pwm_average > RAMP_ASSIST_PWM_THRESHOLD)
    {
        ramp_assist_angle += RAMP_ASSIST_RATE_DPS * dt;
    }
    else if (ramp_assist_angle > decay_step)
    {
        ramp_assist_angle -= decay_step;
    }
    else if (ramp_assist_angle < -decay_step)
    {
        ramp_assist_angle += decay_step;
    }
    else
    {
        ramp_assist_angle = 0.0f;
    }

    ramp_assist_angle = Clamp_Float(ramp_assist_angle,
                                    -RAMP_ASSIST_MAX_ANGLE,
                                    RAMP_ASSIST_MAX_ANGLE);
}

/* ===================== TCRT GPIO ===================== */

static uint8_t TCRT_Left_IsBlack(void)
{
    return (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == TCRT_BLACK_STATE) ? 1 : 0;
}

static uint8_t TCRT_Right_IsBlack(void)
{
    return (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == TCRT_BLACK_STATE) ? 1 : 0;
}

/* ===================== Motor ===================== */
/*
PWMA = PA0 / TIM2_CH1
PWMB = PA1 / TIM2_CH2

AIN1 = PA4
AIN2 = PA5
BIN1 = PA6
BIN2 = PA7
*/

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
    if (left > 0 && left < PWM_MIN) left = PWM_MIN;
    if (left < 0 && left > -PWM_MIN) left = -PWM_MIN;

    if (right > 0 && right < PWM_MIN) right = PWM_MIN;
    if (right < 0 && right > -PWM_MIN) right = -PWM_MIN;

    /* Logical left/right are mapped to the crossed motor channels on this chassis. */
    Motor_Left_SetSpeed(right);
    Motor_Right_SetSpeed(left);
}

static void Motor_Stop(void)
{
    Motor_Left_SetSpeed(0);
    Motor_Right_SetSpeed(0);
}

/* ===================== main ===================== */

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_TIM2_Init();
    MX_I2C1_Init();

    HAL_Delay(500);

    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

    Motor_Stop();

    if (!MPU6050_Initialization())
    {
        while (1)
        {
            Motor_Stop();
        }
    }

    HAL_Delay(500);
    MPU6050_Calibrate();

    uint32_t last_time = HAL_GetTick();
    uint32_t last_line_time = HAL_GetTick();
    int8_t last_turn_direction = 0;
    uint8_t ramp_climb_mode = 0;

    while (1)
    {
        uint32_t now = HAL_GetTick();

        if (now - last_time >= LOOP_TIME_MS)
        {
            float dt = (now - last_time) / 1000.0f;
            last_time = now;

            float current_angle = MPU6050_GetBalanceAngle(dt);

            uint8_t left_black = TCRT_Left_IsBlack();
            uint8_t right_black = TCRT_Right_IsBlack();

            float balance_error = target_angle - current_angle;
            if (left_black == right_black)
            {
                if (fabsf(balance_error) >= RAMP_FIXED_ANGLE)
                {
                    ramp_climb_mode = 1;
                }
                else if (ramp_climb_mode &&
                         fabsf(balance_error) <= RAMP_RELEASE_ANGLE)
                {
                    ramp_climb_mode = 0;
                    ramp_assist_angle = 0.0f;
                    ramp_pwm_average = 0.0f;
                    last_pwm = 0;
                }
            }
            else if (ramp_climb_mode &&
                     fabsf(balance_error) <= RAMP_RELEASE_ANGLE)
            {
                ramp_climb_mode = 0;
                ramp_assist_angle = 0.0f;
                ramp_pwm_average = 0.0f;
                last_pwm = 0;
            }

            float fall_angle = FALL_ANGLE;

            if (left_black == right_black || ramp_climb_mode)
            {
                fall_angle = ramp_climb_mode ?
                             RAMP_FALL_ANGLE :
                             FLAT_FORWARD_FALL_ANGLE;
            }

            if (fabsf(balance_error) > fall_angle)
            {
                Motor_Stop();
                integral = 0.0f;
                last_error = 0.0f;
                last_pwm = 0;
                ramp_assist_angle = 0.0f;
                ramp_pwm_average = 0.0f;
                continue;
            }

            int8_t turn_direction = 0;

            if (left_black == 1 && right_black == 0)
            {
                turn_direction = -1;
                last_turn_direction = turn_direction;
                last_line_time = now;
            }
            else if (left_black == 0 && right_black == 1)
            {
                turn_direction = 1;
                last_turn_direction = turn_direction;
                last_line_time = now;
            }
            else if (left_black == 0 && right_black == 0 &&
                     last_turn_direction != 0 &&
                     now - last_line_time <= LINE_LOST_HOLD_MS)
            {
                turn_direction = last_turn_direction;
            }
            else if (left_black == 1 && right_black == 1)
            {
                last_turn_direction = 0;
            }

            uint8_t line_forward_mode = (turn_direction == 0) ? 1 : 0;
            uint8_t line_turn_mode = (turn_direction != 0) ? 1 : 0;
            uint8_t ramp_fixed_mode = (line_forward_mode && ramp_climb_mode) ? 1 : 0;

            float used_forward_angle = forward_angle + ramp_assist_angle;

            if (line_turn_mode)
            {
                used_forward_angle = line_forward_angle;
            }

            if (line_turn_mode)
            {
                RampAssist_Update(0, 0.0f, dt);
            }

            float move_target = target_angle + used_forward_angle;
            float error = move_target - current_angle;

            if (fabsf(error) < INTEGRAL_ZONE && fabsf(balance_gyro_y) < GYRO_INTEGRAL_ZONE)
            {
                integral += error * dt;

                if (integral > INTEGRAL_LIMIT) integral = INTEGRAL_LIMIT;
                if (integral < -INTEGRAL_LIMIT) integral = -INTEGRAL_LIMIT;
            }
            else
            {
                integral = 0.0f;
            }

            float derivative = -balance_gyro_y;
            last_error = error;

            float balance_output = Kp * error + Ki * integral + Kd * derivative;
            float output = balance_output;
            uint8_t ramp_power_mode = 0;
            float output_limit = CONTROL_PWM_LIMIT;

            if (ramp_climb_mode ||
                (line_forward_mode &&
                 (fabsf(ramp_assist_angle) > RAMP_ASSIST_ACTIVE_ANGLE ||
                  fabsf(balance_output) > RAMP_ASSIST_PWM_THRESHOLD)))
            {
                ramp_power_mode = 1;
                output_limit = RAMP_CONTROL_PWM_LIMIT;
            }

            if (output > output_limit)
            {
                output = output_limit;
                if (error > 0.0f) integral -= error * dt;
            }
            if (output < -output_limit)
            {
                output = -output_limit;
                if (error < 0.0f) integral -= error * dt;
            }

            int pwm = (int)output;
            int turn = 0;

            if (fabsf(error) < FINE_ANGLE && fabsf(balance_gyro_y) < FINE_GYRO_ZONE)
            {
                pwm = 0;
            }
            else if (fabsf(error) > RECOVERY_ANGLE && pwm != 0)
            {
                if (pwm > 0 && pwm < RECOVERY_PWM_MIN) pwm = RECOVERY_PWM_MIN;
                if (pwm < 0 && pwm > -RECOVERY_PWM_MIN) pwm = -RECOVERY_PWM_MIN;
            }	
            else if (fabsf(error) > DRIVE_MIN_ANGLE && pwm != 0)
            {
                if (pwm > 0 && pwm < PWM_MIN) pwm = PWM_MIN;
                if (pwm < 0 && pwm > -PWM_MIN) pwm = -PWM_MIN;
            }

            int slew_step = ramp_power_mode ? RAMP_PWM_SLEW_STEP : PWM_SLEW_STEP;

            if (pwm > last_pwm + slew_step) pwm = last_pwm + slew_step;
            if (pwm < last_pwm - slew_step) pwm = last_pwm - slew_step;
            last_pwm = pwm;

            if (line_forward_mode)
            {
                RampAssist_Update(1, balance_output, dt);
                pwm = Keep_Forward_MinPwm(pwm,
                                          ramp_power_mode ?
                                          RAMP_FORWARD_MIN_PWM :
                                          LINE_FORWARD_MIN_PWM);
            }
            else
            {
                pwm = Keep_Forward_MinPwm(pwm,
                                          ramp_climb_mode ?
                                          RAMP_TURN_MIN_PWM :
                                          LINE_TURN_MIN_PWM);
            }

            if (ramp_fixed_mode)
            {
                ramp_assist_angle = RAMP_ASSIST_MAX_ANGLE;
                ramp_pwm_average = 0.0f;
                integral = 0.0f;
                last_pwm = RAMP_FIXED_PWM;
                Motor_SetBoth(RAMP_FIXED_PWM, RAMP_FIXED_PWM);
                continue;
            }

            turn = turn_direction * LINE_TURN_PWM;

            int left_pwm  = pwm - turn;
            int right_pwm = pwm + turn;

            if (turn < 0)
            {
                left_pwm += 10;
            }

            Motor_SetBoth(left_pwm, right_pwm);
        }
    }
}

/* ===================== Clock ===================== */

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_HSI_ON;
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
}

void Error_Handler(void)
{
  __disable_irq();

  while (1)
  {
  }
}
