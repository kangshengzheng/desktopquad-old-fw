#pragma once

bool gyroDetect(gyroDev_t *dev);
bool gyroInit(gyroDev_t *dev);
bool accDetect(accDev_t *dev);
bool accInit(accDev_t *dev, gyroDev_t *gyroDev);
void mpuSetISR(void mpuSetISR(void (*callback)(void)));
void resetMotorConfig();