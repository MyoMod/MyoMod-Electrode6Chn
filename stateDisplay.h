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

// Defines
#define STATE_DISPLAY_T_IDLE 1000 // time without event until idle starts


enum class DisplayState
{
    IDLE,
    ERROR,
    SENDING_DATA
};

class StateDisplay
{
private:
    uint32_t ledPin;
    DisplayState displayState;
    uint64_t lastEvent;

    void animateIdle(uint32_t timeDiff );
    void animateError(uint32_t timeDiff );
    void animateSendingData(uint32_t timeDiff );
public:
    StateDisplay(uint32_t ledPin = PICO_DEFAULT_LED_PIN);
    ~StateDisplay();

    void run();
    void displayError();
    void clearError();
    void displayIdle();
    void displaySendingData();
};
