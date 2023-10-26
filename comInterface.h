#pragma once

#include <stdint.h>
#include "max11254.h"

int32_t comInterfaceInit(MAX11254 *adc);
void comInterfaceAddSample(int32_t adcValue, uint8_t channel, bool clipped, bool rangeExceeded, bool error);
void comInterfaceIRQHandler();
