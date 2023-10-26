// SPDX-License-Identifier: CC0-1.0

#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"

#include "bsp/board.h"
#include "tusb.h"
#include "tusb_option.h"
#include "device/usbd.h"
#include "device/usbd_pvt.h"

#include "usbInterface.h"
#include "comInterface.h"

// Defines
#define DEBUG_PIN1 2
#define DEBUG_PIN2 3


#define BUFFER_SIZE (32 * 1024) // size of the buffer in bytes
#define ELEMENT_SIZE 4 // size in bytes of one element in sampleBuffer
#define HEADER_SIZE 12 // size of the header in bytes
#define BUFFER_ELEMENTS_MAX ((BUFFER_SIZE - HEADER_SIZE) / ELEMENT_SIZE) // number of elements in the buffer

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


// Variables
static uint32_t led_on_ms, led_off_ms;

static uint32_t sampleBuffer[2][BUFFER_SIZE/4]; // 2 buffers for double buffering
static uint32_t sampleBufferIndex = 0; // index of the current buffer being filled
static uint32_t sampleBufferOffset = 0; // offset in the current buffer being filled

static MAX11254 *g_adc;

static uint32_t g_BufferElements = BUFFER_ELEMENTS_MAX; // the dynamic buffersize

static Header g_incomingHeader[2]; // the header received from the host
static uint32_t g_activeHeader = 0; // the header that is currently being used by core 1

// Private function prototypes
static void board_led_blink_on(uint32_t off_ms);
static void board_led_blink_off(uint32_t on_ms);
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
    memset(sampleBuffer, 0, BUFFER_SIZE*2);

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
    sampleBuffer[sampleBufferIndex][sampleBufferOffset + 3] = *(uint32_t*)&sample;
    sampleBufferOffset++;
    if (sampleBufferOffset >= g_BufferElements)
    {

        // command core1 to send the buffer
        if(!multicore_fifo_wready())
        {
            __breakpoint();
        }
        multicore_fifo_push_blocking(sampleBufferIndex);

        // check if there are new commands and issue them
        // now, so that there is no newstart of the buffers
        comInterfaceIRQHandler();

        sampleBufferOffset = 0;
        sampleBufferIndex = (sampleBufferIndex + 1) % 2;
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

            sampleBufferOffset = 0; // reset the offset
            memset(sampleBuffer, 0, BUFFER_SIZE*2); // clear the buffers
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
    memcpy(&sampleBuffer[0][0], &header, HEADER_SIZE);
    memcpy(&sampleBuffer[1][0], &header, HEADER_SIZE);
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

    memcpy(&header, &sampleBuffer[0][0], HEADER_SIZE);
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

    board_led_blink_on(board_millis() + 100);
    return 0;
}

void comInterfaceRun(void)
{
    tud_task();
    board_led_task();

    // check if there is a buffer to send
    if(multicore_fifo_rvalid())
    {
        uint32_t bufferIndex = multicore_fifo_pop_blocking();
        uint32_t* buffer = sampleBuffer[bufferIndex];
        uint32_t length = BUFFER_SIZE;
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

static void board_led_blink_on(uint32_t off_ms)
{
    board_led_on();
    led_on_ms = 0;
    led_off_ms = off_ms;
}

static void board_led_blink_off(uint32_t on_ms)
{
    board_led_off();
    led_on_ms = on_ms;
    led_off_ms = 0;
}

void board_led_activity(void)
{
    board_led_blink_on(board_millis() + 10); // blink on 10ms
}

static void board_led_task(void)
{
    uint32_t now_ms = board_millis();

    if (led_off_ms && now_ms >= led_off_ms)
        board_led_blink_off(now_ms + 2000); // schedule new idle blink in 5 seconds
    else if (led_on_ms && now_ms >= led_on_ms)
        board_led_blink_on(now_ms + 1000); // idle blink on for one second
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