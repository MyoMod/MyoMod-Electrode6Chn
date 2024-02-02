// SPDX-License-Identifier: CC0-1.0

#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/gpio.h"
#include "string.h"
#include "hardware/irq.h"
#include "pico/sync.h"
#include <span>

#include "comInterface.h"
#include "debug_pins.h"
#include "i2c.h"

#include "commonRegisters.h"
#include "specificRegisters.h"

// Defines
#define CMD_UPDATE_CONFIG 0x01 // command to update the configuration
#define CMD_NEW_DATA 0x02      // command to process new HOut data
#define CMD_SYNC 0x03          // command that indicates a sync from host

// Variables
/**** Register interface ****/
StatusByte_t g_statusByte =
    {
        CommErrorState_t::Ok, // errorState
        CommWarning_t::Ok,    // warningState
        1,                    // H_IN_FIFO_AVAIL
        1,                    // H_OUT_FIFO_NFULL
};
CommonDeviceStatus_t g_commonDeviceStatus =
    {
        1, // notInitialized
        0, // ill_HostInBurstSize
        0, // ill_HostOutBurstSize
        0, // ill_ConfigurationAccess
        0, // reserved
};
CommonDeviceInfo_t g_commonDeviceInfo =
    {
        0,                         // H_In_PacketSize
        5,                         // H_Out_PacketSize
    "Elctr6Ch", // Identifier
        {0, 1, 0},                 // DeviceVersion
        {0, 1, 0},                 // ProtocolVersion
        StreamDir_t::HostInHostOut // SupportedStreamDirections
};
CommonDeviceConfiguration_t g_commonDeviceConfiguration =
    {
        0, // H_In_BurstSize
        0, // H_Out_BurstSize
        0, // DeviceIntialized
        0, // reserved
};

DeviceSpecificStatus_t g_deviceSpecificStatus = 
{
    0, // Synchronized
    0, // Clipped
    0, // RangeExceeded
    0, // ADCError
    0, // NoADC
    1, // Simulated
};
DeviceSpecificInfo_t g_deviceSpecificInfo = 
{
    "MAX11254", // DeviceSpecificInfo
};
DeviceSpecificConfiguration_t g_deviceSpecificConfiguration = 
{
    15, // SamplesPerCycle
    1, // Gain
    0, // UsePGA
    0, // UseFaultControl
    0, // reserved
};

uint32_t g_regLength[] = {sizeof(g_statusByte), sizeof(g_commonDeviceStatus),
                          sizeof(g_commonDeviceInfo), sizeof(g_commonDeviceConfiguration),
                          sizeof(g_deviceSpecificStatus), sizeof(g_deviceSpecificInfo),
                          sizeof(g_deviceSpecificConfiguration)};
uint8_t *g_regPointers[] = {(uint8_t *)&g_statusByte, (uint8_t *)&g_commonDeviceStatus,
                            (uint8_t *)&g_commonDeviceInfo, (uint8_t *)&g_commonDeviceConfiguration,
                            (uint8_t *)&g_deviceSpecificStatus, (uint8_t *)&g_deviceSpecificInfo,
                            (uint8_t *)&g_deviceSpecificConfiguration};
AccessRights_t g_regAccessRights[] = {AccessRights_t::Read, AccessRights_t::Read,
                                      AccessRights_t::Read, AccessRights_t::ReadWrite,
                                      AccessRights_t::Read, AccessRights_t::Read,
                                      AccessRights_t::ReadWrite};

mutex_t g_regMutexes[NUM_REGISTERS];

static uint8_t g_HIn_Buffer[2][HIN_BUFFER_SIZE]; // Ping-Pong Buffer for the HIn data
static uint32_t g_HInBufferIndex = 0;             // index of the current HIn buffer being filled
static uint32_t g_HInBufferOffset = 0;            // offset in the current HIn buffer being filled

static uint8_t g_HOut_Buffer[2][HOUT_BUFFER_SIZE]; // Ping-Pong Buffer for the HOut data

// true if the configuration register has been updated and the configuration needs to be updated
volatile static bool g_updateConfig = false;

// I2C Hardware interface
i2c_inst_t *g_i2c;
uint8_t g_i2cAddr;
uint32_t g_sdaPin;
uint32_t g_sclPin;

// Callbacks
void (*HOut_Callback)(void *data, uint32_t length);
void (*UpdateConfig_Callback)(DeviceSpecificConfiguration_t *config, DeviceSpecificConfiguration_t *oldConfig);
void (*Sync_Callback)(void);

// Private function prototypes
void core1_main(void);
int core1_init(void);
void comInterfaceRun(void);
int comInterfaceSendData(void *buffer, uint32_t length);
bool WriteToRegister(void *buffer, uint32_t length, uint32_t registerName);
void comInterfaceHandleHOutPDS(uint32_t bufferIndex);
bool ReadFromRegister(void *buffer, uint32_t *length, uint32_t registerName);
bool __always_inline ReadStatus(uint8_t *status);
void __isr multicoreFiFoIRQHandler(void);
void comInterfaceHandleConfigUpdate();
void comInterfaceHandleSync();

// Public functions

/**
 * @brief Initializes the communication interface
 *
 * @param adc   pointer to the MAX11254 object
 * @return int32_t
 */
int32_t comInterfaceInit(cominterfaceConfiguration *config)
{
    // copy the configuration
    g_i2c = config->g_i2c;
    g_i2cAddr = config->g_i2cAddr;
    g_sdaPin = config->g_sdaPin;
    g_sclPin = config->g_sclPin;

    HOut_Callback = config->HOut_Callback;
    UpdateConfig_Callback = config->UpdateConfig_Callback;
    Sync_Callback = config->sync_callback;

    // initialize the mutexes
    for (uint32_t i = 0; i < NUM_REGISTERS; i++)
    {
        mutex_init(&g_regMutexes[i]);
    }

    // initialize the buffers
    if(HOUT_BUFFER_SIZE > 0)
    {
        memset(g_HOut_Buffer, 0, HOUT_BUFFER_SIZE * 2);
    }
    if(HIN_BUFFER_SIZE > 0)
    {
        memset(g_HIn_Buffer, 0, HIN_BUFFER_SIZE * 2);
    }

    multicore_launch_core1(core1_main);

    // activate interrupt for incomming multi-core fifo
    irq_set_exclusive_handler(SIO_IRQ_PROC0, multicoreFiFoIRQHandler);
    irq_set_enabled(SIO_IRQ_PROC0, true);
    return 0;
}

/**
 * @brief Adds a sample to the buffer and handles the buffer management
 * 
 * @note This function assumes that the samples are added in chronological order
 *       (that means e.g. that all sammples of t0 are added before the first samples of t1).
 *       The order of the channels does not matter.
 *
 * @param sample        pointer to the sample
 * @param channel       channel number
 */
void comInterfaceAddSample(void *sample, uint32_t channel)
{
    uint32_t bufferIndex = (g_HInBufferOffset / (HIN_NUMBER_OF_CHANNELS * HIN_ELEMENT_SIZE)) * HIN_ELEMENT_SIZE + channel * (HIN_ELEMENT_SIZE * HIN_BUFFER_ELEMENTS_MAX);
    memcpy(&g_HIn_Buffer[g_HInBufferIndex][bufferIndex], sample, HIN_ELEMENT_SIZE);
    g_HInBufferOffset += HIN_ELEMENT_SIZE;

    if (g_HInBufferOffset >= HIN_BUFFER_SIZE)
    {
        // command core1 to send the buffer
        if (!multicore_fifo_wready())
        {
            //__breakpoint();
        }
        multicore_fifo_push_blocking(g_HInBufferIndex);

        // check if there are new commands and issue them
        // now, so that there is no newstart of the buffers
        if (g_updateConfig)
        {
            comInterfaceHandleConfigUpdate();
            g_updateConfig = false;
        }

        g_HInBufferOffset = 0;
        g_HInBufferIndex = (g_HInBufferIndex + 1) % 2;
    }
}

/**
 * @brief Returns the device specific status
 *
 * @param status
 */
void comInterfaceGetStatus(DeviceSpecificStatus_t *status)
{
    mutex_enter_blocking(&g_regMutexes[REG_DeviceSpecificStatus]);
    memcpy(status, &g_deviceSpecificStatus, sizeof(DeviceSpecificStatus_t));
    mutex_exit(&g_regMutexes[REG_DeviceSpecificStatus]);
}

/**
 * @brief Sets the device specific status
 *
 * @param status
 */
void comInterfaceSetStatus(DeviceSpecificStatus_t *status, bool generateWarning, bool generateError)
{
    mutex_enter_blocking(&g_regMutexes[REG_DeviceSpecificStatus]);
    memcpy(&g_deviceSpecificStatus, status, sizeof(DeviceSpecificStatus_t));
    mutex_exit(&g_regMutexes[REG_DeviceSpecificStatus]);

    if (generateWarning)
    {
        mutex_enter_blocking(&g_regMutexes[REG_StatusByte]);
        g_statusByte.warningState = CommWarning_t::DeviceSpecificWarning;
        mutex_exit(&g_regMutexes[REG_StatusByte]);
    }
    if (generateError)
    {
        mutex_enter_blocking(&g_regMutexes[REG_StatusByte]);
        g_statusByte.errorState = CommErrorState_t::DeviceSpecificError;
        mutex_exit(&g_regMutexes[REG_StatusByte]);
    }
}

/**
 * @brief Handles the interrupt that is fired when core1 writes a command to the multi-core fifo
 *
 */
void __isr multicoreFiFoIRQHandler(void)
{
    volatile uint32_t status = multicore_fifo_get_status();

    if (status & SIO_FIFO_ST_ROE_BITS)
    {
        __breakpoint();
    }
    if (status & SIO_FIFO_ST_WOF_BITS)
    {
        __breakpoint();
    }

    if (status & SIO_FIFO_ST_VLD_BITS)
    {
        volatile uint32_t fifoData = multicore_fifo_pop_blocking();
        uint32_t command = fifoData & 0x0000FFFF;
        uint32_t bufferIndex = (fifoData & 0xFFFF0000) >> 16;

        assert(bufferIndex < 2);

        switch (command)
        {
        case CMD_UPDATE_CONFIG:
            g_updateConfig = true;
            break;
        case CMD_NEW_DATA:
            if (HOut_Callback != NULL)
            {
                HOut_Callback(g_HOut_Buffer[bufferIndex], HOUT_BUFFER_SIZE);
            }
            break;
        case CMD_SYNC:
            if (Sync_Callback != NULL)
            {
                Sync_Callback();
            }
            break;
        default:
            //__breakpoint();
            break;
        }
    }
}

/**
 * @brief Handles the configuration update
 *
 */
void comInterfaceHandleConfigUpdate()
{
    static DeviceSpecificConfiguration_t oldConfig;
    static DeviceSpecificConfiguration_t currentConfig;

    // Read the new configuration from the register
    mutex_enter_blocking(&g_regMutexes[REG_DeviceSpecificConfiguration]);
    memcpy(&currentConfig, &g_deviceSpecificConfiguration, sizeof(DeviceSpecificConfiguration_t));
    mutex_exit(&g_regMutexes[REG_DeviceSpecificConfiguration]);

    // call the callback if it is set
    if (UpdateConfig_Callback != NULL)
    {
        UpdateConfig_Callback(&currentConfig, &oldConfig);
    }

    // copy the new configuration to the old configuration so that the callback
    //  can compare the two in the next call
    memcpy(&oldConfig, &currentConfig, sizeof(DeviceSpecificConfiguration_t));
}

/******** CORE 1 ********************************/
void core1_main(void)
{
    core1_init();

    while (1)
    {
        comInterfaceRun();
    }
}

int core1_init(void)
{
    uart_init(uart0, 921600);

    // init i2c
    uint32_t longestRegisterLength = 0;
    for (uint32_t i = 0; i < NUM_REGISTERS; i++)
    {
        if (g_regLength[i] > longestRegisterLength)
        {
            longestRegisterLength = g_regLength[i];
        }
    }

    i2cInitConfiguration_t i2cConfig;
    i2cConfig.i2c = g_i2c;
    i2cConfig.i2cAddr = g_i2cAddr;
    i2cConfig.sdaPin = g_sdaPin;
    i2cConfig.sclPin = g_sclPin;

    i2cConfig.longestRegisterLength = longestRegisterLength;
    i2cConfig.HIn_pdsLength = HIN_BUFFER_SIZE;
    i2cConfig.HOut_pdsBuffer = g_HOut_Buffer;
    i2cConfig.HOut_pdsLength = HOUT_BUFFER_SIZE;

    i2cConfig.H_Out_NotifyPdsBufferFull = comInterfaceHandleHOutPDS;
    i2cConfig.H_In_GetRegisterCallback = ReadFromRegister;
    i2cConfig.H_In_GetStatusCallback = ReadStatus;
    i2cConfig.H_Out_RegisterCallback = WriteToRegister;

    i2cConfig.sync_callback = comInterfaceHandleSync;
    I2C_Init(&i2cConfig);

    return 0;
}

void comInterfaceRun(void)
{
    // check if there is a HIn buffer to send
    if (multicore_fifo_rvalid())
    {
        gpio_put(DEBUG_PIN2, 1);
        uint32_t bufferIndex = multicore_fifo_pop_blocking();
        uint8_t *buffer = g_HIn_Buffer[bufferIndex];
        uint32_t length = HIN_BUFFER_SIZE; // length in bytes
        I2C_send_H_In_PDSData((uint8_t *)g_HIn_Buffer[bufferIndex], length);
        gpio_put(DEBUG_PIN2, 0);
    }
}

/**
 * @brief Writes data to a register.
 *  This function writes the provided data to the specified register. It performs various checks to ensure the validity of the operation.
 *  If the length of the data, the register name, and the access rights are all valid, the data is copied to the register.
 *  If the register name is equal to or greater than REG_DeviceSpecificStatus, it informs core0 that the configuration has changed.
 *  If any of the checks fail, the error bit in the status byte is set and the error state is updated.
 *
 * @param buffer Pointer to the data buffer.
 * @param length Length of the data buffer.
 * @param registerName The name/index of the register to write to.
 * @return void
 */
bool WriteToRegister(void *buffer, uint32_t length, uint32_t registerName)
{
    // check if the length is valid
    bool valid = true;
    valid &= (length == g_regLength[registerName]);
    valid &= (registerName < NUM_REGISTERS);
    valid &= (g_regAccessRights[registerName] == AccessRights_t::Write) ||
             (g_regAccessRights[registerName] == AccessRights_t::ReadWrite);

    if (valid)
    {
        // copy the data to the register
        mutex_enter_blocking(&g_regMutexes[registerName]);
        memcpy(g_regPointers[registerName], buffer, length);
        mutex_exit(&g_regMutexes[registerName]);

        // inform core0 that the configuration has changed
        if (registerName = REG_DeviceSpecificConfiguration)
        {
            if (!multicore_fifo_wready())
            {
                __breakpoint();
            }
            multicore_fifo_push_blocking(CMD_UPDATE_CONFIG);
        }
    }
    else
    {
        // set the error bit in the status byte
        mutex_enter_blocking(&g_regMutexes[REG_CommonDeviceStatus]);
        g_commonDeviceStatus.ill_ConfigurationAccess = 1;
        mutex_exit(&g_regMutexes[REG_CommonDeviceStatus]);
        mutex_enter_blocking(&g_regMutexes[REG_StatusByte]);
        g_statusByte.errorState = CommErrorState_t::CommonError;
        mutex_exit(&g_regMutexes[REG_StatusByte]);
    }
    return valid;
}

/**
 * @brief This function should be called from i2c when new data is
 *          available for the HOut stream.
 *
 * @param data      pointer to the data buffer
 * @param length    length of the data buffer
 */
void comInterfaceHandleHOutPDS(uint32_t bufferIndex)
{
    // command core1 to send the buffer
    if (!multicore_fifo_wready())
    {
        __breakpoint();
    }
    multicore_fifo_push_blocking(CMD_NEW_DATA | bufferIndex << 16);
}

/**
 * @brief This function should be called from i2c when a sync is received.
 *
 */
void comInterfaceHandleSync()
{
    if (!multicore_fifo_wready())
    {
        __breakpoint();
    }
    multicore_fifo_push_blocking(CMD_SYNC);
}

/**
 * @brief Reads data from a register.
 *
 * This function reads data from a specified register and copies it into the provided buffer.
 * It performs various checks to ensure the validity of the operation, such as checking the length,
 * the register name, and the access rights. If the operation is valid, the data is copied from the register
 * into the buffer. If the operation is invalid, error bits in the status byte are set accordingly.
 * The caller needs to read the register using mutexes to ensure that the data is not changed while it is being read.
 *
 * @param buffer The content of the register is copied into this buffer.
 * @param length Pointer to the length of the data to be read, if 0 the length will be written into the pointer.
 * @param registerName The name of the register to read from.
 * @return true if the operation is successful, false otherwise.
 */
bool ReadFromRegister(void *buffer, uint32_t *length, uint32_t registerName)
{
    // check if the length is valid
    bool valid = true;
    if (*length == 0)
    {
        *length = g_regLength[registerName];
    }
    else
    {
        valid &= (*length == g_regLength[registerName]);
    }
    valid &= (registerName < NUM_REGISTERS);
    valid &= (g_regAccessRights[registerName] == AccessRights_t::Read) ||
             (g_regAccessRights[registerName] == AccessRights_t::ReadWrite);

    if (valid)
    {
        // copy the data from the register
        memcpy(buffer, g_regPointers[registerName], *length);
    }
    else
    {
        // set the error bit in the status byte
        mutex_enter_blocking(&g_regMutexes[REG_CommonDeviceStatus]);
        g_commonDeviceStatus.ill_ConfigurationAccess = 1;
        mutex_exit(&g_regMutexes[REG_CommonDeviceStatus]);
        mutex_enter_blocking(&g_regMutexes[REG_StatusByte]);
        g_statusByte.errorState = CommErrorState_t::CommonError;
        mutex_exit(&g_regMutexes[REG_StatusByte]);
    }
    return valid;
}

/**
 * @brief Reads the status byte from the register and checks the access rights.
 *
 *
 * @param status Pointer to the variable where the status byte will be stored.
 * @return true if the access rights are valid and the status byte is successfully read,
 *         false otherwise.
 */
bool __always_inline ReadStatus(uint8_t *status)
{
    // copy the data from the register
    *status = *(uint8_t *)g_regPointers[REG_StatusByte];

    return true;
}
