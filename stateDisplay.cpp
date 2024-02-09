#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"
#include "hardware/timer.h"

#include "stateDisplay.h"

#define ADC_ERROR_COLOR LedColor::MAGENTA
#define SYNC_TIMEOUT_COLOR LedColor::BLUE
#define DATA_TIMEOUT_COLOR LedColor::CYAN

// private function prototypes
uint32_t board_millis(void);

StateDisplay::StateDisplay(std::array<uint32_t, 3>& ledPins, uint32_t timeOut):
    _timeOut(timeOut),
    _connected(false),
    _adcOk(true),
    _dataUpToDate(false),
    _tLastSync(0),
    _tLastData(0)
{
    _ledPins = ledPins;
    _tLastEvent = board_millis();

    for (uint32_t i = 0; i < 3; i++)
    {
        gpio_init(_ledPins[i]);
        #if STATE_DISPLAY_PUSH_PULL
        gpio_set_dir(_ledPins[i], GPIO_OUT);
        #else
        gpio_set_dir(_ledPins[i], GPIO_IN);
        #endif
    }

    update();
}

StateDisplay::~StateDisplay()
{
}

void StateDisplay::setColor(LedColor color)
{
    for (uint32_t i = 0; i < 3; i++)
    {
        bool state = (static_cast<uint8_t>(color) >> i) & 0x01;
        #if STATE_DISPLAY_PUSH_PULL
        gpio_put(_ledPins[i], !state);
        #else
        gpio_set_pulls(_ledPins[i], !state, state);
        #endif
    }
}

void StateDisplay::update()
{
    uint32_t now_ms = board_millis();
    uint32_t timeSinceLastEvent = now_ms - _tLastEvent;

    // Handle timeout
    if((now_ms - _tLastSync) > _timeOut)
    {
        _connected = false;
    }

    if((now_ms - _tLastData) > _timeOut)
    {
        _dataUpToDate = false;
    }

    // ADC error has to be reset 
    if(!_adcOk && (now_ms - _tLastAdcError) > _timeOut)
    {
        clearAdcError();
    }

    // Animate the LED

    if(timeSinceLastEvent > 500)
    {
        _tLastEvent = now_ms;

        if(_connected && _dataUpToDate && _adcOk)
        {
            static bool flashValue = false;
            flashValue = !(flashValue);
            LedColor color = flashValue ? LedColor::GREEN : LedColor::OFF;
            setColor(color);
        }
        else
        {
            uint8_t errors =1 | (!_adcOk << 1) + (!_connected << 2) + (!_dataUpToDate << 3);

            do
            {
                _displayValue++;
                _displayValue = _displayValue % 4;
            } while (((1 << _displayValue) & errors) == 0);

            switch (_displayValue)
            {
            case 0:
                setColor(LedColor::RED);
                break;
            case 1:
                setColor(ADC_ERROR_COLOR);
                break;
            case 2:
                setColor(SYNC_TIMEOUT_COLOR);
                break;
            case 3:
                setColor(DATA_TIMEOUT_COLOR);
                break;
            
            default:
                break;
            }
        }
    }
}

void StateDisplay::adcError()
{
    _adcOk = false;
    update();
}

void StateDisplay::clearAdcError()
{
    _adcOk = true;
}

void StateDisplay::receivedData()
{
    _dataUpToDate = true;
    _tLastData = board_millis();
    update();
}

void StateDisplay::receivedSync()
{
    _connected = true;
    _tLastSync = board_millis();
    update();
}

uint32_t board_millis(void)
{
    return to_ms_since_boot(get_absolute_time());
}