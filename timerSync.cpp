#include "timerSync.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "pico/stdlib.h"
#include "hardware/irq.h"
#include "hardware/clocks.h"
#include "hardware/pwm.h"
#include "assert.h"
#include "debug_pins.h"
#include "math.h"

// Defines
#define ALPHA 0.1 // The alpha value for the low pass filter
#define DEVIATION_FOR_SYNC 2 // The number of clock cycles that the timer is allowed to deviate from the expected value for the sync flag is set

// Global Variables
volatile uint32_t g_cyclesPerSync = 0; // Number of derived clock cycles per sync pulse
volatile uint32_t g_cyclesSinceSync = 0; // Number of derived clock cycles since the last sync pulse

uint32_t g_syncPin = 0; // Pin number of the sync pin

uint32_t g_timer_slice = 0; // Timer number of the timer used to generate the sync signal
volatile uint32_t g_timerPeriod = 0; // Period of the timer used to generate the sync signal
volatile bool g_isSync = false; // Flag that indicates if the synchronisation to the sync signal is done

void (*g_syncCallback)(bool isSync) = NULL;

io_rw_32 *g_pwm_ctr = NULL;


// Private Function Prototypes
void timerSyncIRQ();
void syncIRQ(uint pin, uint32_t eventMask);


// Function Definitions
/**
 * @brief This function will initialize the sync signal generator. It will
 *         configure the timer to generate the sync signal and configure
 *        the gpio pin to be used as the sync pin.
 * 
 * @param inputFreq     The estimated frequency of the input signal
 * @param timesPerSync The timer shall generate timesPerSync evenly spaced sync pulses per cycle
 * @param pin           The pin number of the gpio pin to be used as the sync pin
 * @param timer         The timer number of the timer to be used to generate the sync signal
 */
void initSync(uint32_t inputFreq, uint32_t timesPerSync, uint32_t pin, uint32_t timer, void (*syncCallback)(bool isSync))
{
    assert(inputFreq > 0);
    assert(timesPerSync > 0);
    assert(pin < 32);
    assert(timer < 8);

    g_timer_slice = timer;
    g_pwm_ctr = &pwm_hw->slice[g_timer_slice].ctr;
    g_cyclesPerSync = timesPerSync;
    g_syncCallback = syncCallback;

    // configure the  timer to generate a sync interupt 
    // roughly at inputFreq * timesPerSync Hz. The more precise synchornisation
    // to the sync input will be done while running
    pwm_config pwmConfig =  pwm_get_default_config();

    // Set the prescaler so that the timer runs at 1MHz
    uint32_t clockSpeed = clock_get_hz(clk_sys);
    uint32_t prescaler = clockSpeed / 1000000;
    pwm_config_set_clkdiv(&pwmConfig, prescaler);

    // Set the period of the timer to be 1/outputFreq seconds
    g_timerPeriod = 1'000'000.0 / ((float)inputFreq * (float)timesPerSync);
    pwm_config_set_wrap(&pwmConfig, g_timerPeriod);

    pwm_clear_irq(g_timer_slice);
    pwm_set_irq_enabled(g_timer_slice, true);
    irq_set_exclusive_handler(PWM_IRQ_WRAP, timerSyncIRQ);
    irq_set_enabled(PWM_IRQ_WRAP, true);

    // Init and start the timer
    pwm_init(g_timer_slice, &pwmConfig, true);

    // Configure the gpio pin to be used as the sync pin
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_IN);
    gpio_pull_up(pin);
    gpio_set_irq_enabled_with_callback(pin, GPIO_IRQ_EDGE_RISE, true, syncIRQ);
}

/**
 * @brief This function is to be called from the interrupt handler 
 *          of the gpio pin that is connected to the sync pin of the
 *          host device. This function will reset the timer that is
 *          used to generate the sync signal. And calculate the new
 *          period of the timer based on the new frequency.
 * 
 */
void syncIRQ(uint pin, uint32_t eventMask)
{
    uint32_t counterVal = *g_pwm_ctr;
    uint32_t cyclesSinceSync = g_cyclesSinceSync;

    // Reset the timer
    *g_pwm_ctr = 0;

    // Call the sync callback
    if(g_syncCallback != NULL)
    {
        g_syncCallback(g_isSync);
    }

    // Calculate the new period of the timer
    int32_t totalClockCycles = counterVal + cyclesSinceSync * g_timerPeriod;
    int32_t clocksError = (totalClockCycles - (int32_t)g_cyclesPerSync * (int32_t)g_timerPeriod)/(int32_t)g_cyclesPerSync;
    g_isSync |= abs(clocksError) < DEVIATION_FOR_SYNC; 

    uint32_t newPeriod = (int32_t) g_timerPeriod - (clocksError * ALPHA);
    
    pwm_set_wrap(g_timer_slice, newPeriod);

    // Reset the cycles since sync counter
    g_cyclesSinceSync = 0;
}

void timerSyncIRQ()
{
    // Clear the interrupt flag
    pwm_clear_irq(g_timer_slice);

    // Increment the cycles since sync counter
    g_cyclesSinceSync++;

    // Call the sync callback
    if(g_syncCallback != NULL)
    {
        g_syncCallback(g_isSync);
    }
}