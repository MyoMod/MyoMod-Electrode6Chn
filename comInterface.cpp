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

// Defines
#define DEBUG_PIN1 2
#define DEBUG_PIN2 3


#define BUFFER_SIZE (1 * 1024) // size of the buffer in bytes
#define ELEMENT_SIZE 4 // size in bytes of one element in sampleBuffer
#define HEADER_SIZE 4 // size of the header in bytes
#define BUFFER_ELEMENTS ((BUFFER_SIZE - HEADER_SIZE) / ELEMENT_SIZE) // number of elements in the buffer

// Data types

typedef struct
{
    uint16_t samplePeriod; // in us
    uint16_t activeChannels; // bitfield of active channels
} Header;


// Variables
static uint32_t led_on_ms, led_off_ms;

uint32_t sampleBuffer[2][BUFFER_SIZE/4]; // 2 buffers for double buffering
uint32_t sampleBufferIndex = 0; // index of the current buffer being filled
uint32_t sampleBufferOffset = 0; // offset in the current buffer being filled


// Private function prototypes
static void board_led_blink_on(uint32_t off_ms);
static void board_led_blink_off(uint32_t on_ms);
void board_led_activity(void);
static void board_led_task(void);
void core1_main(void);
int core1_init(void);
void comInterfaceRun(void);
int comInterfaceSendData(void* buffer, uint32_t length);

// Public functions

int comInterfaceInit(void)
{
    multicore_launch_core1(core1_main);
    return 0;
}

int comInterfaceAddSample(uint32_t sample)
{
    sampleBuffer[sampleBufferIndex][sampleBufferOffset] = sample;
    sampleBufferOffset++;
    if (sampleBufferOffset >= BUFFER_ELEMENTS)
    {

        // command core1 to send the buffer
        if(!multicore_fifo_wready())
        {
            __breakpoint();
            return -1;
        }
        multicore_fifo_push_blocking(sampleBufferIndex);

        sampleBufferOffset = 0;
        sampleBufferIndex = (sampleBufferIndex + 1) % 2;
    }
    return 0;
}

int comInterfaceSetHeader(uint16_t samplePeriod, uint16_t channels)
{
    Header header = {samplePeriod, channels};
    memcpy(&sampleBuffer[0][BUFFER_ELEMENTS], &header, HEADER_SIZE);
    memcpy(&sampleBuffer[1][BUFFER_ELEMENTS], &header, HEADER_SIZE);
    return 0;
}


// Private functions

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
        board_led_blink_off(now_ms + 5000); // schedule new idle blink in 5 seconds
    else if (led_on_ms && now_ms >= led_on_ms)
        board_led_blink_on(now_ms + 1000); // idle blink on for one second
}
