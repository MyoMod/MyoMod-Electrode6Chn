#pragma once
/**
  ******************************************************************************
  * @file    MAX11254.h 
  * @author  Domen Jurkovic
  * @version V1.0
  * @date    18-Nov-2015
  * @brief   Header for MAX11254.c module
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/  

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "max11254_hal.h"
#include "math.h"

/* Defines -------------------------------------------------------------------*/
#define MAX11254_SIMULATED
#ifdef MAX11254_SIMULATED
  #define MAX11254_SIM_FUNC(x)    (sin(2 * 3.14 * ((float)x)) * ((float)(1<<20)))
  //#define MAX11254_SIM_FUNC(x)    (x)
  #define MAX11254_SIM_STEP_SIZE (10)
  #define MAX11254_SIM_CHN_OFFSET (200)
#endif
#define MAX11254_NUM_CHANNELS (6)

/* Exported functions ------------------------------------------------------- */

class MAX11254
{
private:
#ifdef MAX11254_SIMULATED
    uint32_t    _lastIndex;
    uint64_t     _nextUpdate;
#endif


    MAX11254_Rate       _rate;
    MAX11254_Seq_Mode   _mode;
    uint8_t     _pga_gain;
    uint8_t     _channels;
    bool        _singleCycle;
    bool        _is2sComplement;

    uint32_t    _rdybPin;
    uint32_t    _csPin;
    uint32_t    _resetPin;

    spi_inst_t  *_spi;

    void        (*_callback)(int32_t measurement, uint8_t channel, bool clipped, bool rangeExceeded, bool error);

    MAX11254_Rate       sampleRate2Rate(float sample_rate, bool singleCycle, float *actualSampleRate = NULL);
    float               rate2SampleRate(MAX11254_Rate rate, bool singleCycle);
    MAX11254_Gain       interger2PGA(uint8_t integer, uint8_t *actualGain = NULL);
    uint8_t             PGA2Integer(MAX11254_Gain pga);

    void                setMode(MAX11254_Seq_Mode mode);
    MAX11254_Seq_Mode   getMode(void);

    int32_t             readMeasurement(uint32_t channel);

    bool                resetADC(uint32_t timeout);
    bool                setupADC(void);
public:
    MAX11254(){};
    MAX11254(spi_inst_t *spi, uint8_t csPin, uint8_t rdybPin, uint32_t resetPin, void (*callback)(int32_t measurement, uint8_t channel, bool clipped, bool rangeExceeded, bool error));
    ~MAX11254();

    float               setSampleRate(float sample_rate);
    uint8_t             setGain(uint8_t gain);
    void                setChannels(uint8_t channel);

    float               getSampleRate(void);
    uint8_t             getGain(void);
    uint8_t             getChannels(void);

    MAX11254_STAT       getStatus(void);
    bool                dataAvailable(void);

    void                IRQ_handler(void);
    void                startConversion(void);
    bool                stopConversion(uint32_t timeout);
};

/*****	END OF FILE	****/