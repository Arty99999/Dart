#include "stm32h7xx_hal.h"

#ifndef Driver_FLASH_H
#define Driver_FLASH_H

typedef struct {
    uint32_t version;
    float calibration_value;

} SystemParams;


extern SystemParams params;



#endif