// SPDX-License-Identifier: CC0-1.0

#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/gpio.h"

#include "bsp/board.h"
#include "tusb.h"
#include "tusb_option.h"
#include "device/usbd.h"
#include "device/usbd_pvt.h"

#include "usbInterface.h"
#include "comInterface.h"
#include "debug_pins.h"

// Defines


#define BUFFER_SIZE (32 * 1024) // size of the buffer in bytes
#define ELEMENT_SIZE 4 // size in bytes of one element in sampleBuffer
#define HEADER_SIZE 12 // size of the header in bytes
#define BUFFER_ELEMENTS_MAX ((((BUFFER_SIZE - HEADER_SIZE) / ELEMENT_SIZE))) // number of elements in the buffer

static const uint32_t LED_ON_IDLE = 1000;
static const uint32_t LED_ON_ACTIVITY = 10;
static const uint32_t LED_ON_ERROR = 100;
static const uint32_t LED_OFF_IDLE = 1500;
static const uint32_t LED_OFF_ERROR1 = 100;
static const uint32_t LED_OFF_ERROR2 = 1000;
static const uint32_t LED_ERROR_COUNT = 3;
static const uint32_t LED_TIMEOUT = 1000; // after this time the state is set to IDLE


// Data types

struct __attribute__((packed)) Header
{
    uint32_t magic : 32; // magic number to identify the header
    uint16_t sampleRate : 16; // in Hz
    uint16_t payload : 16; // payload size in samples (32 bit)
    uint8_t gain : 8; // gain setting
    uint8_t channels : 8; // active channels
    uint32_t reserved : 16;

    // default Header
    Header()
    {
        magic = 0xFFFFFFFF;
        sampleRate = 2000;
        payload = BUFFER_ELEMENTS_MAX;
        gain = 1;
        channels = 1<<3;
        reserved = 0;
    }
};

struct __attribute__((packed)) Sample
{
    int32_t value : 24;
    uint8_t channel : 3;
    uint8_t clipped : 1;
    uint8_t rangeExceeded : 1;
    uint8_t error : 1;
    uint8_t reserved : 1;
    uint8_t EOF_GUARD : 1; // should always be 0 so that EOF can't be produced by a random data pattern
};

enum class LEDState
{
    IDLE,
    ERROR,
    SENDING_DATA
};


// Variables
// Variables for State-LED mangement
static uint32_t g_lastEvent, g_ledErrorCount = 0;
static LEDState g_ledState = LEDState::IDLE;
static bool g_ledOn = false;

static uint32_t g_sampleBuffer[2][BUFFER_SIZE/4]; // 2 buffers for double buffering
static uint32_t g_sampleBufferIndex = 0; // index of the current buffer being filled
static uint32_t g_sampleBufferOffset = 0; // offset in the current buffer being filled

static MAX11254 *g_adc;

static uint32_t g_BufferElements = BUFFER_ELEMENTS_MAX; // the dynamic buffersize

static Header g_incomingHeader[2]; // the header received from the host
static uint32_t g_activeHeader = 0; // the header that is currently being used by core 1

// Private function prototypes
void board_led_error(void);
void board_led_ack_error(void);
void board_led_display_error(uint32_t timeDiff);
void board_led_display_idle(uint32_t timeDiff);
void board_led_activity(void);
static void board_led_task(void);
void core1_main(void);
int core1_init(void);
void comInterfaceRun(void);
int comInterfaceSendData(void* buffer, uint32_t length);
int32_t comInterfaceSetHeader(MAX11254* adc, Header header);
void receiveDataCallback(void* buffer, uint32_t length);
Header constructHeader(MAX11254 *adc);

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

    // initialize the buffers
    memset(g_sampleBuffer, 0, BUFFER_SIZE*2);

    // initialize the header
    Header header = constructHeader(adc);
    comInterfaceSetHeader(adc, header);

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
    Sample sample = {adcValue, channel, clipped, rangeExceeded, error, 0};
    g_sampleBuffer[g_sampleBufferIndex][g_sampleBufferOffset + HEADER_SIZE/4] = *(uint32_t*)&sample;
    g_sampleBufferOffset++;

    // make sure that the buffer is not overrun and that 
    // the buffer is zero padded at the end so that the elements are a mutliple of 6
    uint32_t maxElements = ((g_BufferElements ) / 6) * 6;
    if (g_sampleBufferOffset >= maxElements)
    {

        // command core1 to send the buffer
        if(!multicore_fifo_wready())
        {
            __breakpoint();
        }
        multicore_fifo_push_blocking(g_sampleBufferIndex);

        // check if there are new commands and issue them
        // now, so that there is no newstart of the buffers
        comInterfaceIRQHandler();

        g_sampleBufferOffset = 0;
        g_sampleBufferIndex = (g_sampleBufferIndex + 1) % 2;
    }

    if (error)
    {
        board_led_error();
    }
    else
    {
        board_led_ack_error();
    }
}
void comInterfaceIRQHandler()
{
    // check if there is data in the fifo containing a header
    if(multicore_fifo_rvalid())
    {
        // digest two 32 bit words from the fifo and build the header
        Header incomingHeader;
        uint32_t activeHeader = multicore_fifo_pop_blocking();
        incomingHeader = g_incomingHeader[activeHeader];
        
        // update the ADC settings if necessary

        if(incomingHeader.sampleRate != 0)
        {
            g_adc->setSampleRate(incomingHeader.sampleRate);
        }

        if(incomingHeader.gain != 0)
        {
            g_adc->setGain(incomingHeader.gain);
        }

        if(incomingHeader.channels != 0)
        {
            g_adc->setChannels(incomingHeader.channels);
        }

        if(incomingHeader.payload != 0)
        {
            assert(incomingHeader.payload <= BUFFER_ELEMENTS_MAX);

            g_sampleBufferOffset = 0; // reset the offset
            memset(g_sampleBuffer, 0, BUFFER_SIZE*2); // clear the buffers
            g_BufferElements = incomingHeader.payload;
        }

        // update the header
        Header outgoingHeader = constructHeader(g_adc);
        comInterfaceSetHeader(g_adc, outgoingHeader);
    }
}

// Private functions
/**
 * @brief Writes the header to the buffers
 * 
 * @param adc       pointer to the MAX11254 object
 * @param header    the header to write
 * @return int32_t 
 */
int32_t comInterfaceSetHeader(MAX11254* adc, Header header)
{
    memcpy(&g_sampleBuffer[0][0], &header, HEADER_SIZE);
    memcpy(&g_sampleBuffer[1][0], &header, HEADER_SIZE);
    return 0;
}

/**
 * @brief Get the Header object
 * 
 * @return Header 
 */
Header getHeader(void)
{
    Header header;

    memcpy(&header, &g_sampleBuffer[0][0], HEADER_SIZE);
    return header;
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
    board_init();
    tusb_init();
    usbInterfaceRegisterCallback(receiveDataCallback);

    uart_init(uart0, 921600);

    printf("USB Interface is set up\n");
    return 0;
}

void comInterfaceRun(void)
{
    static bool firstRun = true;
    tud_task();
    board_led_task();

    // check if there is a buffer to send
    if(multicore_fifo_rvalid())
    {
        if (firstRun)
        {
            firstRun = false;
            board_led_activity();
        }

        gpio_put(DEBUG_PIN2, 1);
        gpio_put(DEBUG_PIN2, 0);
        uint32_t bufferIndex = multicore_fifo_pop_blocking();
        uint32_t* buffer = g_sampleBuffer[bufferIndex];
        uint32_t length = g_BufferElements * ELEMENT_SIZE + HEADER_SIZE;
        comInterfaceSendData(buffer, length);
    }
}

/**
 * @brief Callback function that is called when data is received over USB
 * 
 * @param buffer    pointer to the received data
 * @param length    length of the received data
 */
void receiveDataCallback(void* buffer, uint32_t length)
{
    // convert the buffer to a header
    Header header;
    memcpy(&header, buffer, HEADER_SIZE);
    g_incomingHeader[g_activeHeader] = header;
    multicore_fifo_push_blocking(g_activeHeader);
    g_activeHeader = (g_activeHeader + 1) % 2;
}

int comInterfaceSendData(void* buffer, uint32_t length)
{
    
    if (usbInterfaceSendBuffer(buffer, length))
    {
        board_led_activity();
    }
    return 0;
}

void board_led_error(void)
{
    g_ledState = LEDState::ERROR;
}

void board_led_ack_error(void)
{
    g_ledState = LEDState::IDLE;
}

void board_led_activity(void)
{
    if (g_ledState == LEDState::IDLE)
    {
        g_ledState = LEDState::SENDING_DATA;
        g_lastEvent = board_millis();
    }
}

void board_led_display_error(uint32_t timeDiff)
{
    uint32_t totalTime = (LED_OFF_ERROR1 + LED_ON_ERROR) * LED_ERROR_COUNT + LED_OFF_ERROR2;
    uint32_t cyclePos = timeDiff % totalTime;

    if (cyclePos < LED_OFF_ERROR2)
    {
        board_led_off();
    }
    if (cyclePos >= LED_OFF_ERROR2)
    {
        bool ledOn = (cyclePos - LED_OFF_ERROR2) % (LED_OFF_ERROR1 + LED_ON_ERROR) < LED_ON_ERROR;
        if (ledOn)
        {
            board_led_on();
        }
        else
        {
            board_led_off();
        }
    }
}

void board_led_display_idle(uint32_t timeDiff)
{
    uint32_t totalTime = LED_OFF_IDLE + LED_ON_IDLE;
    uint32_t cyclePos = timeDiff % totalTime;

    if (cyclePos < LED_OFF_IDLE)
    {
        board_led_off();
    }
    else
    {
        board_led_on();
    }
}

static void board_led_task(void)
{
    static bool hasLeftIdle = false;

    uint32_t now_ms = board_millis();
    uint32_t timeDiff = now_ms - g_lastEvent;

    switch (g_ledState)
    {
    case LEDState::IDLE:

        if (timeDiff > LED_TIMEOUT && !hasLeftIdle)
        {
            // we never received data - go into Error state
            g_ledState = LEDState::ERROR;
            g_lastEvent = now_ms;
        }
        else
        {
            board_led_display_idle(timeDiff);
        }
        break;
    case LEDState::SENDING_DATA:
        if (timeDiff > LED_TIMEOUT)
        {
            g_ledState = LEDState::IDLE;
            g_lastEvent = now_ms;
            hasLeftIdle = true;
        }
        else
        {
            if (timeDiff > LED_ON_ACTIVITY)
            {
                board_led_off();
            }
            else
            {
                board_led_on();
            }
        }
        break;
    case LEDState::ERROR:
        board_led_display_error(timeDiff);
        break;
    default:
        break;
    }
}

/**
 * @brief Constructs a header from the current ADC settings
 * 
 * @return Header 
 */
Header constructHeader(MAX11254 *adc)
{
    Header header;
    header.sampleRate = adc->getSampleRate();
    header.gain = adc->getGain();
    header.channels = adc->getChannels();
    header.payload = g_BufferElements;
    header.magic = 0xFFFFFFFF;
    return header;
}