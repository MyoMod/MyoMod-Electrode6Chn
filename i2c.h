/**
 * @file i2c.h
 * @author Leon Farchau (leon2225)
 * @brief 
 * @version 0.1
 * @date 14.12.2023
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once
// Includes
#include "pico/stdlib.h"
#include "hardware/i2c.h"


// Datatypes
struct i2cInitConfiguration_t {
    i2c_inst_t *i2c;
    uint8_t i2cAddr;
    uint32_t sdaPin;
    uint32_t sclPin;

    uint32_t longestRegisterLength; // Length of the longest register in bytes.
    uint32_t HIn_pdsLength; // Length of the HIn pds data in bytes.
    void* HOut_pdsBuffer; // Pointer to the HOut pds data.
    uint32_t HOut_pdsLength; // Length of the HOut pds data in bytes.

    // callbacks
    void (*H_Out_NotifyPdsBufferFull)(uint32_t bufferIndex);
    bool (*H_Out_RegisterCallback)(void* buffer, uint32_t length, uint32_t registerAddr);
    bool (*H_In_GetRegisterCallback)(void* buffer, uint32_t *length, uint32_t registerAddr);
    bool (*H_In_GetStatusCallback)(uint8_t *status);
    void (*sync_callback)(void);
};

// Functions
void I2C_Init(i2cInitConfiguration_t *nitConfiguration);
void I2C_send_H_In_PDSData(uint8_t *data, uint32_t len);
