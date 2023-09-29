#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/i2c.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "hardware/interp.h"
#include "hardware/timer.h"
#include "hardware/watchdog.h"
#include "hardware/clocks.h"
#include "hardware/adc.h"

#include "math.h"

// Defines
#define BUFFER_SIZE 1000
#define DEBUG_PIN1 2
#define DEBUG_PIN2 3
#define EOF_MARKER 0xFFFFFFFF
#define EOF_MARKER_SIZE 2 // size of the EOF marker in sampleBuffer Size, a pair of data and timestamp has to be 0xff

//variables
uint32_t sampleRate = 20000; //Hz
uint32_t samplePeriod = 1'000'000/sampleRate; //us
uint32_t gain = 1;

uint32_t nextSampleTime = 0;

int16_t sampleBuffer[2][BUFFER_SIZE+EOF_MARKER_SIZE]; // 2 buffers for double buffering
uint32_t sampleBufferIndex = 0; // index of the current buffer being filled
uint32_t sampleBufferOffset = 0; // offset in the current buffer being filled


// private function prototypes
void setup();
void sendSampleBuffer(int16_t* buffer, uint32_t length);

int main()
{
    setup();

    // start DMA
    uart_puts(uart1, "Starting DMA\n");
    

    while(1)
    {

        volatile uint32_t currentTime = time_us_32();
        if(currentTime >= nextSampleTime)
        {

            gpio_put(DEBUG_PIN1, 1);

            uint16_t timeDiff = currentTime - nextSampleTime;
            // calculate next sample time
            if((currentTime - nextSampleTime) > samplePeriod)
            {
                // we are too late, skip samples
                while(currentTime >= nextSampleTime)
                {
                    nextSampleTime += samplePeriod;
                }
                continue;
            }
            else
            {
                // we are on time, calculate next sample time
                nextSampleTime += samplePeriod;
            }


            // read ADC and store in buffer
            sampleBuffer[sampleBufferIndex][sampleBufferOffset++] = adc_read();

            // add timestamp to buffer
            sampleBuffer[sampleBufferIndex][sampleBufferOffset++] = timeDiff & 0x7FFF;

            // check if buffer is full
            if(sampleBufferOffset >= BUFFER_SIZE)
            {
                // send buffer to UART
                sendSampleBuffer(sampleBuffer[sampleBufferIndex], (BUFFER_SIZE+EOF_MARKER_SIZE) * sizeof(sampleBuffer[0][0]));

                // switch buffers
                sampleBufferIndex = (sampleBufferIndex + 1) % 2;
                sampleBufferOffset = 0;
            }
            gpio_put(DEBUG_PIN1, 0);
        }
    }

    return 0;
}

void setup()
{
    stdio_init_all();

    // check if sample rate is valid
    assert(samplePeriod < 0x7FFF); // time difference is stored in 15 bits

    // Initialise UART 0
    uart_init(uart0, 921600);

    // Initialise UART 1
    uart_init(uart1, 921600);
    gpio_set_function(4, GPIO_FUNC_UART); // UART1 TX
    gpio_set_function(5, GPIO_FUNC_UART); // UART1 RX

    // Initialise DEBUG_PINs as outputs
    gpio_init(DEBUG_PIN1);
    gpio_init(DEBUG_PIN2);
    gpio_set_dir(DEBUG_PIN1, GPIO_OUT);
    gpio_set_dir(DEBUG_PIN2, GPIO_OUT);

    // Setup dma for UART 1
    dma_channel_config dma_config = dma_channel_get_default_config(0);
    channel_config_set_transfer_data_size(&dma_config, DMA_SIZE_8);
    channel_config_set_read_increment(&dma_config, true);
    channel_config_set_write_increment(&dma_config, false);
    channel_config_set_dreq(&dma_config, DREQ_UART1_TX);
    dma_channel_configure(
        0, &dma_config,
        &uart1_hw->dr, // write address
        sampleBuffer[0], // read address
        BUFFER_SIZE+1, // element count
        false // don't start yet
    );

    //prepare buffers
    for (size_t i = 0; i < EOF_MARKER_SIZE; i++)
    {
        sampleBuffer[0][BUFFER_SIZE+i] = 0xFFFF; // add end of buffer marker
        sampleBuffer[1][BUFFER_SIZE+i] = 0xFFFF; // add end of buffer marker
    }
    
    


    // Setup up ADC 0
    adc_init();
    adc_gpio_init(26);
    adc_select_input(0);
}

/**
 * @brief   Sends the buffer to the UART using DMA. 
 *              It also sends a 0xFFFFFFFF at the end of the buffer to signal the end of the buffer.
 * 
 * @param buffer    pointer to the buffer to send
 * @param length    length of the buffer in bytes
 */
void sendSampleBuffer(int16_t* buffer, uint32_t length)
{
    dma_channel_transfer_from_buffer_now(0, buffer, length);
}
    
