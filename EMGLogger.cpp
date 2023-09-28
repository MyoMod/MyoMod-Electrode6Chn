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
#define SIMULATION 1

//variables
uint32_t sampleRate = 1000; //Hz
uint32_t samplePeriod = 1'000'000/sampleRate; //us
uint32_t gain = 1;

uint32_t lastSampleTime = 0;


// private function prototypes
void setup();
float readVoltage(uint32_t gain);

int main()
{
    setup();
    

    while(1)
    {
        uint32_t currentTime = time_us_32();
        if(currentTime - lastSampleTime >= samplePeriod)
        {
            lastSampleTime = currentTime;
            // read ADC
            float rawValue = readVoltage(gain);

            // send to UART
            printf(">V:%d:%f§ Volts\n", currentTime/1000, rawValue);

        }
    }

    return 0;
}

void setup()
{
    stdio_init_all();

    // Initialise UART 0
    uart_init(uart0, 921600);

    // Setup up ADC 0
    adc_init();
    adc_gpio_init(26);
    adc_select_input(0);
}

float readVoltage(uint32_t gain)
{    
    // 12-bit conversion, assume max value == ADC_VREF == 3.3 V
    const float conversion_factor = 3.3f / (1 << 12);

    return adc_read() * conversion_factor - 1.5;
}

/**
 * @brief Create a Sample that is escaped for UART transmission
 *         so that the MSB is never 1
 * 
 * @param sample        The max 28bit sample to escape
 * @return uint32_t     The escaped sample
 */

uint32_t createEscapedSample(uint32_t sample)
{
    uint32_t escapedSample = 0;
    escapedSample |= (sample & 0x0000007F);
    escapedSample |= (sample & (0x0000007F<< 7)) << 1;
    escapedSample |= (sample & (0x0000007F<<14)) << 2;
    escapedSample |= (sample & (0x0000007F<<21)) << 3;
    return escapedSample;
}
