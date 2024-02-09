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
#include "stateDisplay.h"

//include memcpy
#include <string.h>
#include "math.h"

// Defines

#define ADC_CLK_PIN 10
#define ADC_MOSI_PIN 11
#define ADC_MISO_PIN 8
#define ADC_CS_PIN 9
#define ADC_RDYB_PIN 6
#define ADC_RESET_PIN 7

#define POWER_SAVING_PIN 23

#define USE_ONBOARD_ADC 1

#define I2C_UNIT i2c0
#define I2C_ADDR 0x08
#define SDA_PIN 16
#define SCL_PIN 17

#define LED_R_PIN 22
#define LED_G_PIN 21
#define LED_B_PIN 20

#define JUMPER1_PIN 3
#define JUMPER2_PIN 2
#define JUMPER3_PIN 5
#define JUMPER4_PIN 4
#define JUMPER5_PIN 28
#define JUMPER6_PIN 26
#define JUMPER7_PIN 27

//variables
static uint16_t sampleRate = 12'000; //Hz
static const uint8_t JUMPER_PINS[] = {JUMPER1_PIN, JUMPER2_PIN, JUMPER3_PIN, JUMPER4_PIN, JUMPER5_PIN, JUMPER6_PIN, JUMPER7_PIN};

static MAX11254 *g_adc;
static StateDisplay *g_stateDisplay;


// private function prototypes
void syncCallback();
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
        g_stateDisplay->update();
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

    gpio_init(LED_R_PIN);
    gpio_init(LED_G_PIN);
    gpio_init(LED_B_PIN);
    std::array<uint32_t, 3> ledPins = {LED_R_PIN, LED_G_PIN, LED_B_PIN};
    g_stateDisplay = new StateDisplay(ledPins, 150);

    // Initialise JUMPER pins as inputs
    for (int i = 0; i < 7; i++)
    {
        gpio_init(JUMPER_PINS[i]);
        gpio_set_dir(JUMPER_PINS[i], GPIO_IN);
        gpio_pull_up(JUMPER_PINS[i]);
    }


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
    timerSync_init(100, 15, 0, syncCallback);

    //init comInterface
    cominterfaceConfiguration config;
    config.g_i2c = I2C_UNIT;
    config.g_i2cAddr = I2C_ADDR;
    config.g_sdaPin = SDA_PIN;
    config.g_sclPin = SCL_PIN;
    config.HOut_Callback = NULL;
    config.UpdateConfig_Callback = configCallback;
    config.sync_callback = syncCallback;
    comInterfaceInit(&config);
}

void syncCallback()
{
    timerSync_externalTrigger();
    g_stateDisplay->receivedSync();
}

void configCallback(DeviceSpecificConfiguration_t* config, DeviceSpecificConfiguration_t* oldConfig)
{
    (void) config;
    (void) oldConfig;
}

void newDataCallback(int32_t measurement, uint8_t channel, bool clipped, bool rangeExceeded, bool error)
{
    if(error || clipped || rangeExceeded)
    {
        g_stateDisplay->adcError();
    }
    else
    {
        g_stateDisplay->clearAdcError();
    }
    g_stateDisplay->receivedData();
    comInterfaceAddSample(&measurement, channel);
}