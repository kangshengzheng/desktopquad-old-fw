#include <breezystm32.h>

#include "flash.h"

#include "board.h"

extern void SetSysClock(bool overclock);
serialPort_t *Serial1;

void init_board(void)
{
  // Configure clock, this figures out HSE for hardware autodetect
  SetSysClock(0);
  systemInit();
}

void board_reset(bool bootloader)
{
  if (bootloader) systemResetToBootloader();
  else systemReset();
}

// clock

uint32_t clock_millis()
{
  return millis();
}

uint64_t clock_micros()
{
  return micros();
}

void clock_delay(uint32_t milliseconds)
{
  delay(milliseconds);
}

// serial

void serial_init(uint32_t baud_rate)
{
  Serial1 = uartOpen(USART1, NULL, baud_rate, MODE_RXTX);
}

void serial_write(uint8_t byte)
{
  serialWrite(Serial1, byte);
}

uint16_t serial_bytes_available(void)
{
  return serialTotalBytesWaiting(Serial1);
}

uint8_t serial_read(void)
{
  return serialRead(Serial1);
}

// sensors

static bool _baro_present;
static bool _mag_present;
static bool _sonar_present;
static bool _diff_pressure_present;

static float _accel_scale;
static float _gyro_scale;

void sensors_init(int board_revision)
{
    (void)(board_revision);

    // IMU
    uint16_t acc1G;
    mpu6050_init(true, &acc1G, &_gyro_scale, board_revision);
    _accel_scale = 9.80665f/acc1G;
}

void imu_register_callback(void (*callback)(void))
{
  mpu6050_register_interrupt_cb(callback);
}

void imu_read_accel(float accel[3])
{
  // Convert to NED
  int16_t accel_raw[3];
  mpu6050_read_accel(accel_raw);
  accel[0] = accel_raw[0] * _accel_scale;
  accel[1] = -accel_raw[1] * _accel_scale;
  accel[2] = -accel_raw[2] * _accel_scale;
}

void imu_read_gyro(float gyro[3])
{
  //  Convert to NED
  int16_t gyro_raw[3];
  mpu6050_read_gyro(gyro_raw);
  gyro[0] = gyro_raw[0] * _gyro_scale;
  gyro[1] = -gyro_raw[1] * _gyro_scale;
  gyro[2] = -gyro_raw[2] * _gyro_scale;
}

float imu_read_temperature(void)
{
  int16_t temperature_raw;
  mpu6050_read_temperature(&temperature_raw);
  return temperature_raw/340.0f + 36.53f;
}

bool mag_present(void)
{
  return false;
}

void mag_read(float mag[3])
{
  // Convert to NED
  int16_t raw_mag[3];
  hmc5883l_update();
  hmc5883l_read(raw_mag);
  mag[0] = (float)raw_mag[0];
  mag[1] = -(float)raw_mag[1];
  mag[2] = -(float)raw_mag[2];
}

bool mag_check(void)
{
  return false;
}

bool baro_present(void)
{
  return false;
}

void baro_read(float *altitude, float *pressure, float *temperature)
{
  (*altitude) = 0.0f;
  (*pressure) = 0.0f;
  (*temperature) = 0.0f;
}

void baro_calibrate() {}

bool diff_pressure_present(void)
{
  return false;
}

bool diff_pressure_check(void)
{
  return false;
}

void diff_pressure_calibrate() {}

void diff_pressure_set_atm(float barometric_pressure) {}

void diff_pressure_read(float *diff_pressure, float *temperature, float *velocity)
{
  *diff_pressure = 0.0f;
  *temperature = 0.0f;
  *velocity = 0.0f;
}

bool sonar_present(void)
{
  return false;
}

bool sonar_check(void)
{
  return false;
}

float sonar_read(void)
{
  return 0.0f;
}

// PWM

void pwm_init(bool cppm, uint32_t refresh_rate, uint16_t idle_pwm)
{
  pwmInit(cppm, false, false, refresh_rate, idle_pwm);
}

uint16_t pwm_read(uint8_t channel)
{
  return pwmRead(channel);
}

void pwm_write(uint8_t channel, uint16_t value)
{
  pwmWriteMotor(channel, value);
}

bool pwm_lost()
{
    return ((millis() - pwmLastUpdate()) > 40);
}

// non-volatile memory

void memory_init(void) {}

bool memory_read(void * dest, size_t len)
{
  (void)(dest);
  (void)(len);
  return false;
}

bool memory_write(const void * src, size_t len)
{
  return false;
}

// LED

void led0_on(void) { /*LED0_ON;*/ }
void led0_off(void) { /*LED0_OFF;*/ }
void led0_toggle(void) { /*LED0_TOGGLE;*/ }

void led1_on(void) { /*LED1_ON;*/ }
void led1_off(void) { /*LED1_OFF;*/ }
void led1_toggle(void) { /*LED1_TOGGLE;*/ }
