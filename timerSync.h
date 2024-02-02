#pragma once

// Include any necessary libraries or headers
#include "pico/stdlib.h"

// Define any constants or macros

// Declare any classes or structs

// Declare any function prototypes
void timerSync_init(uint32_t inputFreq, uint32_t timesPerSync, uint32_t timer, void (*syncCallback)(bool isSync));
void timerSync_tick();
