#pragma once

/**
 * @brief 
 * 
 */

/************************************************************************************
 * INCLUDES
 ************************************************************************************/
#include <stdint.h>

/************************************************************************************
 * DEFINES
 ************************************************************************************/

/************************************************************************************
 * PROTOTYPES
 * *********************************************************************************/

/************************************************************************************
 * DATA TYPES
 * *********************************************************************************/

/**
 * @brief Device specific status
 * 
 */
struct __attribute__((packed)) DeviceSpecificStatus_t
{
    uint8_t Synchronized : 1;
    uint8_t Clipped : 6;
    uint8_t RangeExceeded : 6;
    uint8_t ADCError : 1;
    uint8_t NoADC : 1;
    uint8_t Simulated : 1;
};

/**
 * @brief Device specific information
 * 
 */
struct __attribute__((packed)) DeviceSpecificInfo_t
{
    uint8_t DeviceSpecificInfo[10];
};

/**
 * @brief Device specific configuration
 * 
 */
struct DeviceSpecificConfiguration_t
{
    uint8_t SamplesPerCycle;
    uint8_t Gain;
    uint8_t UsePGA : 1;
    uint8_t UseFaultControl : 1;
    uint8_t reserved : 6;
};