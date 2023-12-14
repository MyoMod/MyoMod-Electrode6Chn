// SPDX-License-Identifier: CC0-1.0

#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/gpio.h"
#include "string.h"
#include "hardware/irq.h"
#include "pico/sync.h"

#include "comInterface.h"
#include "debug_pins.h"
#include "stateDisplay.h"
#include "processingUnit.h"
#include "i2c.h"

// Defines
#define NUMBER_OF_CHANNELS 6 // number of channels of the ADC
#define ELEMENT_SIZE 4 // size in bytes of one element in sampleBuffer
#define BUFFER_ELEMENTS_MAX 20 // max number of elements in the buffer
#define BUFFER_SIZE (BUFFER_ELEMENTS_MAX * ELEMENT_SIZE * NUMBER_OF_CHANNELS) // size of the buffer in bytes

#define I2C_PORT i2c0 // i2c port to use
#define I2C_ADDR 0x08 // i2c address of the device
#define I2C_SDA_PIN 16 // sda pin of the i2c interface
#define I2C_SCL_PIN 17 // scl pin of the i2c interface


// Data types

/*** Fields common to all devices for the communication interface ***/
/**
 * @brief Control byte for the communication interface
 * 
 */
struct __attribute__((packed)) ControlByte_t
{
    uint32_t PDS_nRegister : 1;
    uint32_t HostIn_nHostOut : 1;
    
    uint32_t ADDR : 6; // used only for register access, otherwise reserved
};

/**
 * @brief Error states for the onebyte status byte
 * 
 */
enum class CommErrorState_t
{
    Ok = 1,
    CommonError,
    DeviceSpecificError,
    AlignmentError,
    OverflowH_IN,
    UnderflowH_IN,
    OverflowH_OUT,
    UnderflowH_OUT
};

/**
 * @brief Warning states for the onebyte status byte
 * 
 */
enum class CommWarning_t
{
    Ok = 1,
    CommonWarning,
    DeviceSpecificWarning,
};

/**
 * @brief Status byte for the communication interface
 * 
 */
struct __attribute__((packed)) StatusByte_t
{
    CommErrorState_t errorState : 4;
    CommWarning_t warningState : 2;
    bool H_IN_FIFO_AVAIL : 1;
    bool H_OUT_FIFO_NFULL : 1;
};

/**
 * @brief Common Device Status
 * 
 */
struct __attribute__((packed)) CommonDeviceStatus_t
{
    uint8_t notInitialized : 1;
    uint8_t ill_HostInBurstSize : 1;
    uint8_t ill_HostOutBurstSize : 1;
    uint8_t ill_ConfigurationAccess : 1;
    uint8_t reserved : 4;
};

/**
 * @brief Direction of the stream supported by the device
 * 
 */
enum class StreamDir_t
{
    None = 0,
    HostIn = 1,
    HostOut = 2,
    HostInHostOut = 3
};

/**
 * @brief Common Device Information
 * 
 */
struct __attribute__((packed)) CommonDeviceInfo_t
{
    uint8_t H_In_PacketSize;
    uint8_t H_Out_PacketSize;
    char Identifier[10];
    uint8_t DeviceVersion[3];
    uint8_t ProtocolVersion[3];
    StreamDir_t SupportedStreamDirections;
};

/**
 * @brief Common Device Configuration
 * 
 */
struct __attribute__((packed)) CommonDeviceConfiguration_t
{
    uint8_t H_In_BurstSize;
    uint8_t H_Out_BurstSize;
    uint8_t DeviceIntialized : 1;
    uint8_t reserved : 7;
};

/*** Device Specific fields for the communication interface ***/

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

/**
 * @brief Register Names
 * 
 */
enum RegisterName_t
{
    REG_StatusByte,
    REG_CommonDeviceStatus,
    REG_CommonDeviceInfo,
    REG_CommonDeviceConfiguration,
    REG_DeviceSpecificStatus,
    REG_DeviceSpecificInfo,
    REG_DeviceSpecificConfiguration,
    NUM_REGISTERS,
};

/**
 * @brief Accessrights for the registers
 * 
 */
enum class AccessRights_t
{
    NoAccess = 0,
    Read = 1,
    Write = 2,
    ReadWrite = 3
};


// Variables
/**** Register interface ****/
StatusByte_t g_statusByte = 
{
    CommErrorState_t::Ok, // errorState
    CommWarning_t::Ok,  // warningState
    1, // H_IN_FIFO_AVAIL
    1, // H_OUT_FIFO_NFULL
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
    0, // H_In_PacketSize
    0, // H_Out_PacketSize
    "Elctr6Ch", // Identifier
    {0, 1, 0}, // DeviceVersion
    {0, 1, 0}, // ProtocolVersion
    StreamDir_t::HostIn // SupportedStreamDirections
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
uint8_t* g_regPointers[] = {(uint8_t*)&g_statusByte, (uint8_t*)&g_commonDeviceStatus,
                            (uint8_t*)&g_commonDeviceInfo, (uint8_t*)&g_commonDeviceConfiguration,
                            (uint8_t*)&g_deviceSpecificStatus, (uint8_t*)&g_deviceSpecificInfo,
                            (uint8_t*)&g_deviceSpecificConfiguration};
AccessRights_t g_regAccessRights[] = {AccessRights_t::Read, AccessRights_t::Read, 
                                        AccessRights_t::Read, AccessRights_t::ReadWrite, 
                                        AccessRights_t::Read, AccessRights_t::Read, 
                                        AccessRights_t::ReadWrite};

mutex_t g_regMutexes[NUM_REGISTERS];



static uint32_t g_sampleBuffer[2][BUFFER_SIZE/4]; // 2 buffers for double buffering
static uint32_t g_sampleBufferIndex = 0; // index of the current buffer being filled
static uint32_t g_sampleBufferOffset = 0; // offset in the current buffer being filled

static uint32_t g_outputBuffer[2][BUFFER_SIZE/4]; // buffer for the output data
static uint32_t g_outputBufferIndex = 0; // index of the current buffer being sent

static bool g_outputBufferReady = false; // flag indicating if the output buffer is ready to be sent

static MAX11254 *g_adc;
static StateDisplay g_stateDisplay;

static uint32_t g_BufferElements = 6 * 10; // the dynamic buffersize

// Private function prototypes
void core1_main(void);
int core1_init(void);
void comInterfaceRun(void);
int comInterfaceSendData(void* buffer, uint32_t length);
bool WriteToRegister(void* buffer, uint32_t length, uint32_t registerName);
bool ReadFromRegister(void* buffer, uint32_t *length, uint32_t registerName);
bool __always_inline ReadStatus(uint8_t* status);

// Public functions

/**
 * @brief Initializes the communication interface
 * 
 * @param adc   pointer to the MAX11254 object
 * @return int32_t 
 */
int32_t comInterfaceInit(MAX11254 *adc)
{
    g_adc = adc;
    g_stateDisplay = StateDisplay(PICO_DEFAULT_LED_PIN);

    // initialize the mutexes
    for (uint32_t i = 0; i < NUM_REGISTERS; i++)
    {
        mutex_init(&g_regMutexes[i]);
    }

    // initialize the buffers
    memset(g_sampleBuffer, 0, BUFFER_SIZE*2);

    multicore_launch_core1(core1_main);
    return 0;
}

/**
 * @brief Adds a sample to the buffer and handles the buffer management
 * 
 * @param adcValue          the value read from the ADC
 * @param channel           the channel the value was read from
 * @param clipped           true if the value was clipped
 * @param rangeExceeded     true if the value was out of range
 * @param error             true if there was an error
 */
void comInterfaceAddSample(int32_t adcValue, uint8_t channel, bool clipped, bool rangeExceeded, bool error)
{
    static uint8_t bufferClipped = 0;
    static uint8_t bufferRangeExceeded = 0;
    static bool bufferError = false;

    g_sampleBuffer[g_sampleBufferIndex][g_sampleBufferOffset] = adcValue;
    g_sampleBufferOffset++;

    // sticky error handling
    bufferClipped |= clipped << channel;
    bufferRangeExceeded |= rangeExceeded << channel;
    bufferError |= error;

    // make sure that the buffer is not overrun and that 
    // the buffer is zero padded at the end so that the elements are a mutliple of 6
    uint32_t maxElements = ((g_BufferElements ) / 6) * 6;
    if (g_sampleBufferOffset >= maxElements)
    {
        // device specific status
        mutex_enter_blocking(&g_regMutexes[REG_DeviceSpecificStatus]);
        g_deviceSpecificStatus.Clipped = bufferClipped;
        g_deviceSpecificStatus.RangeExceeded = bufferRangeExceeded;
        g_deviceSpecificStatus.ADCError = bufferError;
        mutex_exit(&g_regMutexes[REG_DeviceSpecificStatus]);
        bufferClipped = 0;
        bufferRangeExceeded = 0;
        bufferError = false;

        // command core1 to send the buffer
        if(!multicore_fifo_wready())
        {
            __breakpoint();
        }
        multicore_fifo_push_blocking(g_sampleBufferIndex);

        // check if there are new commands and issue them
        // now, so that there is no newstart of the buffers
        comInterfaceInputHandler();

        g_sampleBufferOffset = 0;
        g_sampleBufferIndex = (g_sampleBufferIndex + 1) % 2;
    }

    if (bufferError)
    {
        g_stateDisplay.displayError();
    }
    else
    {
        g_stateDisplay.clearError();
    }
}
void comInterfaceInputHandler()
{
    // check if there is data in the fifo containing a header
    if(multicore_fifo_rvalid())
    {
        // digest one element from the fifo
        uint32_t dummy = multicore_fifo_pop_blocking();
        mutex_enter_blocking(&g_regMutexes[REG_DeviceSpecificConfiguration]);
        uint32_t samplesPerCycle = g_deviceSpecificConfiguration.SamplesPerCycle;
        uint32_t gain = g_deviceSpecificConfiguration.Gain;
        bool usePGA = g_deviceSpecificConfiguration.UsePGA;
        bool useFaultControl = g_deviceSpecificConfiguration.UseFaultControl;
        mutex_exit(&g_regMutexes[REG_DeviceSpecificConfiguration]);

        // calculate the number of elements in the buffer
        g_BufferElements = samplesPerCycle * 6;

        // TODO: update the timer sync interval

        g_adc->setGain(gain);

        // not implemented yet
#if 0
        g_adc->setUsePGA(usePGA);
        g_adc->setUseFaultControl(useFaultControl);
#endif
    }
}


/******** CORE 1 ********************************/
void core1_main(void)
{
    core1_init();

    while(1)
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
    i2cConfig.i2c = I2C_PORT;
    i2cConfig.i2cAddr = I2C_ADDR;
    i2cConfig.sdaPin = I2C_SDA_PIN;
    i2cConfig.sclPin = I2C_SCL_PIN;
    i2cConfig.longestRegisterLength = longestRegisterLength;
    i2cConfig.pdoDataLen = g_BufferElements * ELEMENT_SIZE;
    i2cConfig.H_Out_PDSCallback = NULL;
    i2cConfig.H_In_RegisterCallback = ReadFromRegister;
    i2cConfig.H_In_StatusCallback = ReadStatus;
    i2cConfig.H_Out_RegisterCallback = WriteToRegister;
    I2C_Init(&i2cConfig);

    return 0;
}

void comInterfaceRun(void)
{

    // check if there is a buffer to send
    if(multicore_fifo_rvalid())
    {
        gpio_put(DEBUG_PIN2, 1);
        uint32_t bufferIndex = multicore_fifo_pop_blocking();
        uint32_t* buffer = g_sampleBuffer[bufferIndex];
        uint32_t length = g_BufferElements * ELEMENT_SIZE; // length in bytes
        g_outputBufferIndex = bufferIndex;
        processData(buffer, g_outputBuffer[bufferIndex], length);
        I2C_send_H_In_PDSData((uint8_t*) g_outputBuffer[bufferIndex], length);
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
bool WriteToRegister(void* buffer, uint32_t length, uint32_t registerName)
{
    // check if the length is valid
    bool valid = true;
    valid &= (length == g_regLength[registerName]);
    valid &= (registerName < NUM_REGISTERS);
    valid &= (g_regAccessRights[registerName] == AccessRights_t::Write) || 
                (g_regAccessRights[registerName] == AccessRights_t::ReadWrite);
    
    if(valid)
    {
        // copy the data to the register
        mutex_enter_blocking(&g_regMutexes[registerName]);
        memcpy(g_regPointers[registerName], buffer, length);
        mutex_exit(&g_regMutexes[registerName]);

        // inform core0 that the configuration has changed
        if(registerName >= REG_DeviceSpecificStatus)
        {
            if(!multicore_fifo_wready())
            {
                __breakpoint();
            }
            multicore_fifo_push_blocking(0);
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
bool ReadFromRegister(void* buffer, uint32_t *length, uint32_t registerName)
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
    
    if(valid)
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
bool __always_inline ReadStatus(uint8_t* status)
{
    // copy the data from the register
    *status = *(uint8_t*)g_regPointers[REG_StatusByte];
    
    return true;
}
