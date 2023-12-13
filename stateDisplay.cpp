#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"
#include "hardware/timer.h"

#include "stateDisplay.h"

// private function prototypes
uint32_t board_millis(void);

StateDisplay::StateDisplay(uint32_t ledPin)
{
    ledPin = ledPin;
    displayState = DisplayState::IDLE;
    lastEvent = board_millis();

    gpio_init(ledPin);
    gpio_set_dir(ledPin, GPIO_OUT);
}

StateDisplay::~StateDisplay()
{
}

void StateDisplay::run()
{
    uint32_t now_ms = board_millis();
    uint32_t timeDiff = now_ms - lastEvent;

    switch (displayState)
    {
    case DisplayState::IDLE:
        animateIdle(timeDiff);
        break;
    case DisplayState::SENDING_DATA:
        if (timeDiff > STATE_DISPLAY_T_IDLE)
        {
            displayState = DisplayState::IDLE;
            lastEvent = now_ms;
        }
        else
        {
            animateSendingData(timeDiff);
        }
        break;
    case DisplayState::ERROR:
        animateError(timeDiff);
        break;
    default:
        break;
    }
}

void StateDisplay::animateIdle(uint32_t timeDiff)
{
    const uint32_t LED_OFF_IDLE = 1000;
    const uint32_t LED_ON_IDLE = 500;

    uint32_t totalTime = LED_OFF_IDLE + LED_ON_IDLE;
    uint32_t cyclePos = timeDiff % totalTime;

    gpio_put(ledPin, cyclePos < LED_ON_IDLE);
}

void StateDisplay::animateError(uint32_t timeDiff)
{
    const uint32_t LED_OFF_ERROR1 = 100;
    const uint32_t LED_OFF_ERROR2 = 1000;
    const uint32_t LED_ON_ERROR = 100;
    const uint32_t LED_ERROR_COUNT = 3;

    uint32_t totalTime = (LED_OFF_ERROR1 + LED_ON_ERROR) * LED_ERROR_COUNT + LED_OFF_ERROR2;
    uint32_t cyclePos = timeDiff % totalTime;

    if (cyclePos > (totalTime - LED_OFF_ERROR2))
    {
        gpio_put(ledPin, 0);
    }
    else
    {
        bool ledOn = (cyclePos) % (LED_OFF_ERROR1 + LED_ON_ERROR) < LED_ON_ERROR;
        gpio_put(ledPin, ledOn);
    }
}

void StateDisplay::animateSendingData(uint32_t timeDiff)
{
    const uint32_t LED_OFF_SENDING = 125;
    const uint32_t LED_ON_SENDING = 125;

    uint32_t totalTime = LED_OFF_SENDING + LED_ON_SENDING;
    uint32_t cyclePos = timeDiff % totalTime;

    gpio_put(ledPin, cyclePos < LED_ON_SENDING);
}

void StateDisplay::displayError()
{
    displayState = DisplayState::ERROR;
    lastEvent = board_millis();
}

void StateDisplay::clearError()
{
    displayState = DisplayState::IDLE;
    lastEvent = board_millis();
}

void StateDisplay::displayIdle()
{
    displayState = DisplayState::IDLE;
    lastEvent = board_millis();
}

void StateDisplay::displaySendingData()
{
    displayState = DisplayState::SENDING_DATA;
    lastEvent = board_millis();
}


uint32_t board_millis(void)
{
    return to_ms_since_boot(get_absolute_time());
}