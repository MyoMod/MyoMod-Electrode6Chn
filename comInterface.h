#pragma once
/**
 * @file comInterface.h
 * @author Leon Farchau (leon2225)
 * @brief 
 * @version 0.1
 * @date 03.01.2024
 * 
 * @copyright Copyright (c) 2024
 * 
 */

/************************************************************************************
 * INCLUDES
 ************************************************************************************/
#include <stdint.h>
#include "hardware/i2c.h"

#include "specificRegisters.h"

/************************************************************************************
 * DEFINES
 ************************************************************************************/
#define HOUT_NUMBER_OF_CHANNELS 0                                                                 // number of channels for the H_OUT stream
#define HOUT_ELEMENT_SIZE 1                                                                       // size in bytes of one element for the H_OUT stream
#define HOUT_BUFFER_ELEMENTS_MAX 1                                                                // max number of elements in the buffer for the H_OUT stream
#define HOUT_BUFFER_SIZE (HOUT_BUFFER_ELEMENTS_MAX * HOUT_ELEMENT_SIZE * HOUT_NUMBER_OF_CHANNELS) // size of the buffer in bytes for the H_OUT stream

#define HIN_NUMBER_OF_CHANNELS 6                                                              // number of channels for the H_IN stream
#define HIN_ELEMENT_SIZE 4                                                                    // size in bytes of one element for the H_IN stream
#define HIN_BUFFER_ELEMENTS_MAX 15                                                             // max number of elements in the buffer for the H_IN stream
#define HIN_BUFFER_SIZE (HIN_BUFFER_ELEMENTS_MAX * HIN_ELEMENT_SIZE * HIN_NUMBER_OF_CHANNELS) // size of the buffer in bytes for the H_IN stream


/************************************************************************************
 * Data types
 * *********************************************************************************/
struct cominterfaceConfiguration
{
    // I2C Hardware interface
    i2c_inst_t* g_i2c;
    uint8_t g_i2cAddr;
    uint32_t g_sdaPin;
    uint32_t g_sclPin;

    // Callbacks
    void (*HOut_Callback)(void* data, uint32_t length);
    void (*UpdateConfig_Callback)(DeviceSpecificConfiguration_t* config, DeviceSpecificConfiguration_t* oldConfig);
    void (*sync_callback)(void);
};


/************************************************************************************
 * Functions
 * *********************************************************************************/

int32_t comInterfaceInit(cominterfaceConfiguration* config);
void comInterfaceAddSample(void *sample, uint32_t channel);
void comInterfaceGetStatus(DeviceSpecificStatus_t *status);
void comInterfaceSetStatus(DeviceSpecificStatus_t *status, bool generateWarning, bool generateError);
