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

#include "comInterface.h"
#include "max11254_hal.h"

//include memcpy
#include <string.h>
#include "math.h"

// Defines
#define DEBUG_PIN1 2
#define DEBUG_PIN2 3

#define ADC_CLK_PIN 10
#define ADC_MOSI_PIN 11
#define ADC_MISO_PIN 12
#define ADC_CS_PIN 13
#define ADC_RDYB_PIN 2
#define ADC_RESET_PIN 5

#define USE_ONBOARD_ADC 0

// Data types
typedef struct
{
    uint16_t samplePeriod; // in us
    uint16_t gain;
} Header;

//variables
uint16_t sampleRate = 2'000; //Hz
uint16_t samplePeriod = 1'000'000/sampleRate; //us
uint16_t gain = 1;
Header header = {samplePeriod, gain};

uint32_t nextSampleTime = 0;



// private function prototypes
void setup();
void sendSampleBuffer(void* buffer, uint32_t length);

int main(){
    setup();
   

    while(1)
    {
        #if USE_ONBOARD_ADC
        volatile uint32_t currentTime = time_us_32();
        if(currentTime >= nextSampleTime)
        {

            gpio_put(DEBUG_PIN1, 1);

            volatile uint32_t timeDiff = currentTime - nextSampleTime + samplePeriod;
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
            uint32_t adcValue = adc_read() & 0x00FFFFFE;
            comInterfaceAddSample(adcValue);

            gpio_put(DEBUG_PIN1, 0);
        }
        #endif

        // check if a conversion is finished
        if(gpio_get(ADC_RDYB_PIN) == 0)
        {
            // read ADC and store in buffer
            int32_t adcValue;
            adcValue = (int32_t) max11254_hal_read_reg(MAX11254_DATA3_OFFSET);
            adcValue = (adcValue & 0x00800000)? (adcValue | 0xFF000000) : adcValue;
            //printf("ADC value: %d\n", adcValue);
            comInterfaceAddSample(adcValue);
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

    // Initialise DEBUG_PINs as outputs
    gpio_init(DEBUG_PIN1);
    gpio_init(DEBUG_PIN2);
    gpio_set_dir(DEBUG_PIN1, GPIO_OUT);
    gpio_set_dir(DEBUG_PIN2, GPIO_OUT);
    
    //init comInterface
    comInterfaceInit();
    comInterfaceSetHeader(samplePeriod, 1);

    // Setup up ADC 0
    adc_init();
    adc_gpio_init(26);
    adc_select_input(0);

    // Init MAX11254
    spi_inst_t *spiADC = spi1;
    gpio_set_function(ADC_CLK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(ADC_MOSI_PIN, GPIO_FUNC_SPI);
    gpio_set_function(ADC_MISO_PIN, GPIO_FUNC_SPI);
    gpio_init(ADC_CS_PIN);
    gpio_set_dir(ADC_CS_PIN, GPIO_OUT);
    gpio_init(ADC_RDYB_PIN);
    gpio_set_dir(ADC_RDYB_PIN, GPIO_IN);
    gpio_init(ADC_RESET_PIN);
    gpio_set_dir(ADC_RESET_PIN, GPIO_OUT);

    gpio_put(ADC_CS_PIN, 1);
    gpio_put(ADC_RESET_PIN, 1);

    volatile uint32_t actualBaudRate = 0;
    actualBaudRate = spi_init(spiADC, 8000000);
    printf("Actual baud rate: %d\n", actualBaudRate);

    // reset ADC
    gpio_put(ADC_RESET_PIN, 0);
    sleep_ms(1);
    gpio_put(ADC_RESET_PIN, 1);

    max11254_hal_init(spiADC, ADC_CS_PIN);
}
    
