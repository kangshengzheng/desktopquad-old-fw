#include "platform.h"

// #include "common/axis.h"
// #include "common/color.h"
// #include "common/maths.h"
#include "common/printf.h"

// #include "config/config_eeprom.h"
// #include "config/config_profile.h"
// #include "config/feature.h"
// #include "config/parameter_group.h"
// #include "config/parameter_group_ids.h"

// #include "drivers/nvic.h"
// #include "drivers/sensor.h"
#include "drivers/system.h"
// #include "drivers/dma.h"
#include "drivers/io.h"
// #include "drivers/light_led.h"
#include "drivers/timer.h"
#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "drivers/gpio.h"
#include "drivers/accgyro.h"
#include "drivers/accgyro_mpu.h"
#include "drivers/accgyro_mpu6500.h"
#include "drivers/accgyro_spi_mpu6500.h"
// #include "drivers/pwm_esc_detect.h"
// #include "drivers/rx_pwm.h"
#include "drivers/pwm_output.h"
// #include "drivers/adc.h"
// #include "drivers/bus_i2c.h"
#include "drivers/bus_spi.h"
// #include "drivers/inverter.h"
// #include "drivers/usb_io.h"
// #include "drivers/exti.h"
#include "drivers/serial_usb_vcp.h"

#include "flash.h"

#include "board.h"
#include "board_sensors.h"

static serialPort_t *serial0 = NULL;
static serialPort_t *cereal = NULL;

void init_board(void)
{
  printfSupportInit();
  
  // this enables the FPU, sets up clocks, NVIC, SysTick
  systemInit();

  // initialize IO (needed for all IO operations)
  IOInitGlobal();

  // setup external interrupt driver
  EXTIInit();

  delay(100);

  // timer must be initialized before any channel is allocated
  timerInit();

  // led GPIOB_Pin_8
  LEDInit();

  // open serial ports
  serial0 = usbVcpOpen();
  cereal = uartOpen(USART1, NULL, 115200, MODE_RXTX, SERIAL_NOT_INVERTED);
  setPrintfSerialPort(cereal);

  led0_on();
  delay(100);
  led0_off();
  delay(100);
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
  // serial0 = uartOpen(USART1, NULL, baud_rate, MODE_RXTX, SERIAL_NOT_INVERTED);
  // serial0 = usbVcpOpen();
}

void serial_write(uint8_t byte)
{
  serialWrite(serial0, byte);
}

uint16_t serial_bytes_available(void)
{
  return serialRxBytesWaiting(serial0);
}

uint8_t serial_read(void)
{
  // here I could add a check for the bootloader character
  // to send the system into bootloader reset
  return serialRead(serial0);
}

// sensors

static bool _baro_present;
static bool _mag_present;
static bool _sonar_present;
static bool _diff_pressure_present;

static float _accel_scale;
static float _gyro_scale;

static accDev_t _accDev;
static gyroDev_t _gyroDev;

void sensors_init(int board_revision)
{
    (void)(board_revision);

    // For MPU6500
    spiInit(SPIDEV_1);

    // Gyro (do gyro first!)
    gyroInit(&_gyroDev);

    // Accelerometer
    accInit(&_accDev, &_gyroDev);

    // Set acc1G. Modified once by mpu6050CheckRevision for old (hopefully nonexistent outside of clones) parts
    const uint16_t acc1G = 512 * 8; // 256?
    _accel_scale = 9.80665f/acc1G;

    // 16.4 dps/lsb scalefactor for all Invensense devices
    _gyro_scale = (1.0f / 16.4f) * (3.14159 / 180.0f);
}

void imu_register_callback(void (*callback)(void))
{
  mpuSetISR(callback);
}

void imu_read_accel(float accel[3])
{
  // Convert to NED
  _accDev.read(&_accDev);
  accel[0] = _accDev.ADCRaw[0] * _accel_scale;
  accel[1] = -_accDev.ADCRaw[1] * _accel_scale;
  accel[2] = -_accDev.ADCRaw[2] * _accel_scale;
}

void imu_read_gyro(float gyro[3])
{
  //  Convert to NED
  _gyroDev.read(&_gyroDev);
  gyro[0] = _gyroDev.gyroADCRaw[0] * _gyro_scale;
  gyro[1] = -_gyroDev.gyroADCRaw[1] * _gyro_scale;
  gyro[2] = -_gyroDev.gyroADCRaw[2] * _gyro_scale;
}

float imu_read_temperature(void)
{
  // int16_t temperature_raw;
  // mpu6050_read_temperature(&temperature_raw);
  // return temperature_raw/340.0f + 36.53f;

  // TODO: implement
  return 0.0f;
}

void imu_not_responding_error()
{
  // // If the IMU is not responding, then we need to change where we look for the
  // // interrupt
  // _board_revision = (_board_revision < 4) ? 5 : 2;
  // sensors_init();
}

bool mag_present(void)
{
  return false;
}

void mag_read(float mag[3])
{
  // Convert to NED
  mag[0] = 0.0f;
  mag[1] = 0.0f;
  mag[2] = 0.0f;
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
static motorDevConfig_t motorDev;

void pwm_init(bool cppm, uint32_t refresh_rate, uint16_t idle_pwm)
{
  // pwmInit(cppm, false, false, refresh_rate, idle_pwm);
}

uint16_t pwm_read(uint8_t channel)
{
  // return pwmRead(channel);
  return 0;
}

void pwm_write(uint8_t channel, uint16_t value)
{
  // pwmWriteMotor(channel, value);
}

bool pwm_lost()
{
    // return ((millis() - pwmLastUpdate()) > 40);
    return false;
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

void led0_on(void) { digitalLo(GPIOB, GPIO_Pin_8);/*LED0_ON;*/ }
void led0_off(void) { digitalHi(GPIOB, GPIO_Pin_8);/*LED0_OFF;*/ }
void led0_toggle(void) { /*LED0_TOGGLE;*/ }

void led1_on(void) { /*LED1_ON;*/ }
void led1_off(void) { /*LED1_OFF;*/ }
void led1_toggle(void) { /*LED1_TOGGLE;*/ }
