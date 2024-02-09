/**
 * @file stateDisplay.h
 * @author Leon Farchau (leon2225)
 * @brief 
 * @version 0.1
 * @date 11.12.2023
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include <array>

#define STATE_DISPLAY_PUSH_PULL 0

enum class DisplayState
{
    IDLE,
    ERROR,
    SENDING_DATA
};

enum class LedColor
{
    RED     = 0b001,
    GREEN   = 0b010,
    BLUE    = 0b100,
    YELLOW  = 0b011,
    CYAN    = 0b110,
    MAGENTA = 0b101,
    WHITE   = 0b111,
    OFF     = 0b000
};

class StateDisplay
{
private:
    std::array<uint32_t, 3> _ledPins;
    bool _connected;
    bool _adcOk;
    bool _dataUpToDate;
    volatile uint64_t _tLastEvent;
    uint32_t _displayValue; // 0 = red; 1 = ADC error, 2 = sync timeout, 3 = data timeout

    uint64_t _tLastSync;
    uint64_t _tLastData;
    uint64_t _tLastAdcError;

    uint32_t _timeOut;

    void setColor(LedColor color);
public:
    StateDisplay(std::array<uint32_t, 3>& ledPins, uint32_t timeOut = 150);
    ~StateDisplay();

    void update();

    void adcError();
    void clearAdcError();

    void receivedSync();
    void receivedData();
};
