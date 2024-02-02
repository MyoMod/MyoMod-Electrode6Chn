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
#include "pico/multicore.h"

#include "comInterface.h"
#include "max11254.h"
#include "debug_pins.h"
#include "timerSync.h"

//include memcpy
#include <string.h>
#include "math.h"

// Defines

#define ADC_CLK_PIN 10
#define ADC_MOSI_PIN 11
#define ADC_MISO_PIN 12
#define ADC_CS_PIN 13
#define ADC_RDYB_PIN 2
#define ADC_RESET_PIN 5

#define POWER_SAVING_PIN 23

#define USE_ONBOARD_ADC 0

#define I2C_UNIT i2c1
#define I2C_ADDR 0x08
#define SDA_PIN 2
#define SCL_PIN 3

//variables
static uint16_t sampleRate = 16'000; //Hz

static MAX11254 *g_adc;


// private function prototypes
void setup();
void configCallback(DeviceSpecificConfiguration_t* config, DeviceSpecificConfiguration_t* oldConfig);
void newDataCallback(int32_t measurement, uint8_t channel, bool clipped, bool rangeExceeded, bool error);

int main(){
    setup();

    while(1)
    {
        // check if a conversion is finished
        if(g_adc->dataAvailable())
        {
            g_adc->async_handler();
        }
    }

    return 0;
}

inline void syncCallback(bool isSync)
{
    if(1)
    {
        g_adc->IRQ_handler();
    }
}

void setup()
{
    stdio_init_all();

    // Initialise UART 0
    uart_init(uart0, 921600);

    // Initialise DEBUG_PINs as outputs
    gpio_init(DEBUG_PIN1);
    gpio_init(DEBUG_PIN2);
    gpio_init(DEBUG_PIN3);
    gpio_set_dir(DEBUG_PIN1, GPIO_OUT);
    gpio_set_dir(DEBUG_PIN2, GPIO_OUT);
    gpio_set_dir(DEBUG_PIN3, GPIO_OUT);

    // Setup up ADC 0
    adc_init();
    adc_gpio_init(26);
    adc_select_input(0);

    // Init MAX11254
    // gpio_init(POWER_SAVING_PIN);
    // gpio_set_dir(POWER_SAVING_PIN, GPIO_OUT);
    // gpio_put(POWER_SAVING_PIN, 1); // turn off power saving

    spi_inst_t *spiADC = spi1;
    gpio_set_function(ADC_CLK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(ADC_MOSI_PIN, GPIO_FUNC_SPI);
    gpio_set_function(ADC_MISO_PIN, GPIO_FUNC_SPI);

    volatile uint32_t actualBaudRate = 0;
    actualBaudRate = spi_init(spiADC, 8000000);
    printf("Actual baud rate: %d\n", actualBaudRate);

    g_adc = new MAX11254(spiADC, ADC_CS_PIN, ADC_RDYB_PIN, ADC_RESET_PIN, newDataCallback);
    g_adc->startConversion(false);

    g_adc->setSampleRate(sampleRate);
    
    // Start the sync
    timerSync_init(100, 10, 0, syncCallback);

    //init comInterface
    cominterfaceConfiguration config;
    config.g_i2c = I2C_UNIT;
    config.g_i2cAddr = I2C_ADDR;
    config.g_sdaPin = SDA_PIN;
    config.g_sclPin = SCL_PIN;
    config.HOut_Callback = NULL;
    config.UpdateConfig_Callback = configCallback;
    config.sync_callback = timerSync_tick;
    comInterfaceInit(&config);
}

void configCallback(DeviceSpecificConfiguration_t* config, DeviceSpecificConfiguration_t* oldConfig)
{
    (void) config;
    (void) oldConfig;
}

void newDataCallback(int32_t measurement, uint8_t channel, bool clipped, bool rangeExceeded, bool error)
{
    

    comInterfaceAddSample(&measurement, channel);
}