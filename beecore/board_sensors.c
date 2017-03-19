#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#include "common/printf.h"

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

extern const timerHardware_t timerHardware[];

static void (*mpuCallback)(void) = NULL;

static const extiConfig_t *selectMPUIntExtiConfig(void) {
    static const extiConfig_t mpuIntExtiConfig = { .tag = IO_TAG(MPU_INT_EXTI) };
    return &mpuIntExtiConfig;
}

static bool gyroUpdateISR(gyroDev_t* gyroDev) {

    if (mpuCallback != NULL) {
        mpuCallback();
    }

    return true;
}

bool gyroDetect(gyroDev_t *dev) {
    if (mpu6500SpiGyroDetect(dev)) {
        dev->gyroAlign = GYRO_MPU6500_ALIGN;
        printf("Gyro SPI Found!\n");
        return true;
    }
    return false;
}

bool gyroInit(gyroDev_t *dev) {
    memset(dev, 0, sizeof(*dev));

    dev->mpuIntExtiConfig = selectMPUIntExtiConfig();
    mpuDetect(dev);

    if (!gyroDetect(dev))
        return false;

    dev->init(dev);

    mpuGyroSetIsrUpdate(dev, gyroUpdateISR);

    return true;
}

bool accDetect(accDev_t *dev) {
    if (mpu6500SpiAccDetect(dev)) {
        dev->accAlign = ACC_MPU6500_ALIGN;
        printf("SPI found\n");
        return true;
    }
    return false;
}

bool accInit(accDev_t *dev, gyroDev_t *gyroDev) {
    memset(dev, 0, sizeof(*dev));

    // copy over the common gyro mpu settings
    dev->mpuConfiguration = gyroDev->mpuConfiguration;
    dev->mpuDetectionResult = gyroDev->mpuDetectionResult;

    if (!accDetect(dev))
        return false;

    dev->acc_1G = 256; // set default
    dev->init(dev); // driver initialisation

    return true;
}

void mpuSetISR(gyroDev_t *gyroDev, void (*callback)(void)) {
    mpuCallback = callback;
}

void pwmMotorInit(motorDevConfig_t *dev) {
    memset(dev, 0, sizeof(*dev));

    // minthrottle == 1000
    // maxthrottle == 2000

    // for some reason these motors need to be at the same rate as BRUSHLESS_MOTORS_PWM_RATE
    dev->motorPwmRate = 480; //BRUSHLESS_MOTORS_PWM_RATE; //16000; //BRUSHED_MOTORS_PWM_RATE;
    dev->motorPwmProtocol = PWM_TYPE_ONESHOT125; //PWM_TYPE_BRUSHED;
    dev->useUnsyncedPwm = true;


    int motorIndex = 0;
    for (int i = 0; i < USABLE_TIMER_CHANNEL_COUNT && motorIndex < MAX_SUPPORTED_MOTORS; i++) {
        if (timerHardware[i].usageFlags & TIM_USE_MOTOR) {
            dev->ioTags[motorIndex] = timerHardware[i].tag;
            motorIndex++;
        }
    }
}