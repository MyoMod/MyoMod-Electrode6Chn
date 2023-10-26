
#include "max11254.h"
#include "math.h"

// const variables
const float sampleRatesCont[] = {1.9, 3.9, 7.8, 15.6, 31.2, 62.5, 125, 250, 500, 1000, 2000, 4000, 8000, 16000, 32000, 64000};
const float sampleRatesSingle[] = {50, 62.5, 100, 125, 200, 250, 400, 500, 800, 1000, 1600, 2000, 3200, 4000, 6400, 12800};
const uint8_t gainValues[] = {1, 2, 4, 8, 16, 32, 64, 128};

// private function prototypes
uint32_t firstSetBit(uint32_t value);
template <typename T>
uint32_t getNearestIndex(T value, const T *array, uint32_t arrayLength);


MAX11254::MAX11254(spi_inst_t *spi, uint8_t csPin, uint8_t rdybPin, uint32_t resetPin, void (*callback)(int32_t measurement, uint8_t channel, bool clipped, bool rangeExceeded, bool error))
{
    this->_csPin = csPin;
    this->_rdybPin = rdybPin;
    this->_resetPin = resetPin;
    this->_spi = spi;
    this->_callback = callback;

    // Initialize member variables
    this->_rate = MAX11254_Rate::CONT_2000_SPS;
    this->_mode = MAX11254_Seq_Mode::SEQ_MODE_1;
    this->_pga_gain = 1;
    this->_channels = 0b001000;
    this->_singleCycle = false;
    this->_is2sComplement = true;

    // Initialize pins
    gpio_init(this->_csPin);
    gpio_set_dir(this->_csPin, GPIO_OUT);
    gpio_put(this->_csPin, 1);

    gpio_init(this->_rdybPin);
    gpio_set_dir(this->_rdybPin, GPIO_IN);

    gpio_init(this->_resetPin);
    gpio_set_dir(this->_resetPin, GPIO_OUT);
    gpio_put(this->_resetPin, 1);

    // init hal
    max11254_hal_init(this->_spi, this->_csPin);

    // Reset the ADC
    this->resetADC(1'000'000);


    // setup ADC
    this->setupADC();
}

MAX11254::~MAX11254()
{
    // hold the adc in reset
    gpio_put(this->_resetPin, 0);
}

/**
 * @brief Sets the sample rate of the ADC as close as possible to the desired sample rate.
 * 
 * @param sample_rate  Desired sample rate in Hz
 * @return uint32_t    Actual sample rate in Hz
 */
float MAX11254::setSampleRate(float sample_rate)
{
    float selectedSampleRate = 0;
    MAX11254_Rate newRate;

    // get current single-cycle state
    MAX11254_CTRL1 ctrl1_reg;
    max11254_hal_read_reg(MAX11254_CTRL1_OFFSET, &ctrl1_reg);
    bool singleCycle = ctrl1_reg.SCYCLE;

    newRate = this->sampleRate2Rate(sample_rate, singleCycle, &selectedSampleRate);

    bool rateChanged = _rate != newRate;
    _rate = newRate;

    // if sample rate was changed and continuous mode is active, 
        // restart it with the new sample rate
    if(rateChanged && !_singleCycle)
    {
        stopConversion(0);

        //start with new sample rate
        startConversion();
    }
    
    return selectedSampleRate;
}

/**
 * @brief Sets the gain of the PGA as close as possible to the desired gain.
 * 
 * @param gain  Desired gain
 * @return uint8_t  Actual gain
 */
uint8_t MAX11254::setGain(uint8_t gain)
{
    uint8_t selectedGain = 0;
    MAX11254_Gain pgaGain = this->interger2PGA(gain, &selectedGain);

    bool gainChanged = this->_pga_gain != selectedGain;
    this->_pga_gain = selectedGain;

    if(gainChanged)
    {
        stopConversion(0);

        // set new gain
        MAX11254_CTRL2 ctrl2_reg;
        max11254_hal_read_reg(MAX11254_CTRL2_OFFSET, &ctrl2_reg);
        ctrl2_reg.PGAG = pgaGain;
        ctrl2_reg.PGAEN = _pga_gain > 1; // enable PGA if gain is > 1
        max11254_hal_write_reg(MAX11254_CTRL2_OFFSET, &ctrl2_reg);

        startConversion();
    }

    return selectedGain;
}

/**
 * @brief Sets the channels to be sampled.
 * 
 * @param channel   Channels to be sampled, bit 0 = channel 0, bit 1 = channel 1, ...
 */
void MAX11254::setChannels(uint8_t channels)
{
    //for now, only one channel is supported
    assert(0<channels && channels<=0x3F);
    assert((channels & (channels-1)) == 0); // check if only one bit is set

    _channels = channels;
    
    // get MUX
    uint8_t mux = firstSetBit(channels);

    stopConversion(0);

    // Datasheet states that MUX may be changed during conversion
    MAX11254_SEQ seq_ctrl_reg;
    max11254_hal_read_reg(MAX11254_SEQ_OFFSET, &seq_ctrl_reg);
    seq_ctrl_reg.MUX = mux;
    max11254_hal_write_reg(MAX11254_SEQ_OFFSET, &seq_ctrl_reg);

    startConversion();
}

/**
 * @brief Get the samplerate
 * 
 * @return float samplerate in Hz
 */
float MAX11254::getSampleRate()
{
    return rate2SampleRate(_rate, _singleCycle);
}

/**
 * @brief Get the gain
 * 
 * @return uint8_t gain
 */
uint8_t MAX11254::getGain()
{
    return _pga_gain;
}

/**
 * @brief Get the channels
 * 
 * @return uint8_t channels
 */
uint8_t MAX11254::getChannels()
{
    return _channels;
}

/**
 * @brief Returns the STAT-Register of the ADC.
 * 
 * @return MAX11254_STAT 
 */
MAX11254_STAT MAX11254::getStatus()
{
    MAX11254_STAT stat_reg;
    max11254_hal_read_reg(MAX11254_STAT_OFFSET);
    return stat_reg;
}

/**
 * @brief Function that is called when the RDYB pin is pulled low.
 *          It reads the measurement and calls the callback function.
 * 
 */
void MAX11254::IRQ_handler()
{
    // get active channels
    uint8_t channel = firstSetBit(this->_channels);

    // read measurement
    uint32_t measurement = this->readMeasurement(channel);
    MAX11254_STAT stat_reg;
    max11254_hal_read_reg(MAX11254_STAT_OFFSET, &stat_reg);
    bool error = stat_reg.ERROR || stat_reg.GPOERR || stat_reg.ORDERR || stat_reg.SCANERR || !stat_reg.REFDET;

    // restart conversion if in single-cycle mode
    if(_singleCycle)
    {
        startConversion();
    }

    // call callback function
    this->_callback(measurement, channel, stat_reg.DOR, stat_reg.AOR, error);
}

/**
 * @brief Starts a conversion with the current settings.
 * 
 */
void MAX11254::startConversion()
{
    // get STAT register
    MAX11254_STAT stat_reg;
    max11254_hal_read_reg(MAX11254_STAT_OFFSET, (MAX11254_STAT*)&stat_reg);

    // check if ADC is in power-down mode
    if(stat_reg.PDSTAT != MAX11254_PowerDown::CONVERSION)
    {
        // ADC is in power-down mode, start conversion
        max11254_hal_send_command(MAX11254_Command_Mode::SEQUENCER, _rate);
    }
    else
    {
        // ADC is not in power-down mode, stop conversion and wait until ADC is in power-down mode
        stopConversion(0);

        //start with new sample rate
        max11254_hal_send_command(MAX11254_Command_Mode::SEQUENCER, _rate);
    }

}

/**
 * @brief Stops a conversion and waits until the adc is in power-down mode.
 * 
 * @param timeout   Timeout in us after which the function returns even if the adc is not in power-down mode.
 *                  If timeout is 0, the function waits forever.
 * return true if the adc is in power-down mode, false if the timeout was reached.
 */
bool MAX11254::stopConversion(uint32_t timeout)
{
    max11254_hal_send_command(MAX11254_Command_Mode::POWER_DOWN, _rate);

    // wait until adc is in power-down mode
    uint32_t endTime = time_us_32() + timeout;
    bool inPD = false;
    do
    {
        // Read Stat and check if ADC is in power-down mode
        MAX11254_STAT stat_reg;
        max11254_hal_read_reg(MAX11254_STAT_OFFSET, &stat_reg);
        inPD = stat_reg.PDSTAT != MAX11254_PowerDown::CONVERSION;
    } while (!inPD && (timeout == 0 || (time_us_32() < endTime)));
    return inPD;
}

/**
 * @brief Gets the Rate enum that is closest to the given sample rate.
 *          If selectedSampleRate is not NULL, the selected sample rate is written to the given address.
 * 
 * @param sample_rate           Desired sample rate in Hz
 * @param singleCycle           true if single-cycle mode is active, false if continuous mode is active
 * @param selectedSampleRate    Pointer to a float where the selected sample rate is written to or NULL
 * @return MAX11254_Rate        Rate enum that is closest to the given sample rate
 */
MAX11254_Rate MAX11254::sampleRate2Rate(float sample_rate, bool singleCycle, float *selectedSampleRate)
{
    const float* sampleRates = _singleCycle ? sampleRatesSingle : sampleRatesCont;

    uint32_t index = getNearestIndex(sample_rate, sampleRates, sizeof(sampleRatesCont)/sizeof(sampleRatesCont[0]));
    if(selectedSampleRate != NULL)
    {
        *selectedSampleRate = sampleRates[index];
    }

    return (MAX11254_Rate)index;
}


/**
 * @brief Gets the sample rate in Hz for the given Rate enum.
 * 
 * @param rate          Rate enum
 * @param singleCycle   true if single-cycle mode is active, false if continuous mode is active
 * @return float        Sample rate in Hz
 */

float MAX11254::rate2SampleRate(MAX11254_Rate rate, bool singleCycle)
{
    assert((uint)rate < 16); // check if rate is valid (0-15)
    const float* sampleRates = _singleCycle ? sampleRatesSingle : sampleRatesCont;
    return sampleRates[(uint)rate];
}

/**
 * @brief Gets the PGA gain that is closest to the given gain.
 *          If selectedGain is not NULL, the actual gain is written to the given address.
 * 
 * @param integer       Desired gain
 * @param selectedGain    Pointer to a uint8_t where the actual gain is written to or NULL
 * @return MAX11254_Gain    PGA gain that is closest to the given gain
 */
MAX11254_Gain MAX11254::interger2PGA(uint8_t integer, uint8_t *selectedGain)
{
    uint32_t index = getNearestIndex(integer, gainValues, sizeof(gainValues)/sizeof(gainValues[0]));
    if(selectedGain != NULL)
    {
        *selectedGain = gainValues[index];
    }

    return (MAX11254_Gain)index;
}

/**
 * @brief Gets the gain as integer for the given PGA gain.
 * 
 * @param pga_gain      PGA gain
 * @return uint8_t      Gain in integer
 */
uint8_t MAX11254::PGA2Integer(MAX11254_Gain pga_gain)
{
    assert((uint)pga_gain < 8); // check if gain is valid (0-7)
    return gainValues[(uint)pga_gain];
}

/**
 * @brief Reads the measurement from the given channel.
 * 
 * @param channel   Channel to read from
 * @return int32_t  Measurement
 */
int32_t MAX11254::readMeasurement(uint32_t channel)
{
    assert(channel < 6); // check if channel is valid (0-5)
    int32_t measurement = max11254_hal_read_reg(MAX11254_DATA0_OFFSET + channel, NULL);

    // signextend 24bit value to 32bit if 2s complement is used
    if(_is2sComplement)
    {
        measurement = (measurement << 8) >> 8;
    }

    return measurement;
}

/**
 * @brief Resets the ADC and waits until it is ready.
 * 
 * @param timeout   Timeout in us after which the function returns even if the adc is not ready.
 *                  If timeout is 0, the function waits forever.
 * @return true     if the adc is ready, false if the timeout was reached.
 */
bool MAX11254::resetADC(uint32_t timeout)
{
    // reset ADC
    gpio_put(this->_resetPin, 0);
    sleep_ms(1);
    gpio_put(this->_resetPin, 1);

    sleep_ms(10); //datasheet page 29

    // wait until ADC is ready
    uint32_t endTime = time_us_32() + timeout;
    bool ready = false;
    do
    {
        // Read Stat and check if ADC is in reset
        MAX11254_STAT stat_reg;
        max11254_hal_read_reg(MAX11254_STAT_OFFSET, &stat_reg);
        ready = stat_reg.PDSTAT != MAX11254_PowerDown::STANDBY;
    } while (!ready && (timeout == 0 || (time_us_32() < endTime)));
    return ready;
}

/**
 * @brief Sets up the ADC.
 * 
 * @return true if setup was successful, false if not.
 */
bool MAX11254::setupADC()
{
    MAX11254_CTRL1 ctrl1_reg;
	ctrl1_reg.CAL = MAX11254_Calibration_Mode::SELF;
	ctrl1_reg.PD = MAX11254_PowerDown::STANDBY;
	ctrl1_reg.Unipolar = !_is2sComplement;
	ctrl1_reg.FORMAT = 0;
	ctrl1_reg.SCYCLE = _singleCycle;
	ctrl1_reg.CONTSC = 0;
	max11254_hal_write_reg(MAX11254_CTRL1_OFFSET, &ctrl1_reg);

	MAX11254_CTRL2 ctrl2_reg;
	ctrl2_reg.CCSEN = 0;
	ctrl2_reg.EXTCLK = 0;
	ctrl2_reg.LDOEN = 1;
	ctrl2_reg.LPMODE = 0;
	ctrl2_reg.PGAEN = _pga_gain > 1; // enable PGA if gain is > 1
	ctrl2_reg.PGAG = interger2PGA(_pga_gain);
	max11254_hal_write_reg(MAX11254_CTRL2_OFFSET, &ctrl2_reg);

	MAX11254_CTRL3 ctrl3_reg;
	ctrl3_reg.GPO_MODE = 0;
	ctrl3_reg.SYNC_MODE = 0;
	ctrl3_reg.CALREGSEL = 1;
	ctrl3_reg.NOSYSG = 1;
	ctrl3_reg.NOSYSO = 1;
	ctrl3_reg.NOSCG = 1;
	ctrl3_reg.NOSCO = 1;
	max11254_hal_write_reg(MAX11254_CTRL3_OFFSET, &ctrl3_reg);

	// Disable sync Input
	MAX11254_GPIO_CTRL gpio_ctrl_reg;
	gpio_ctrl_reg.DIO0 = 0;
	gpio_ctrl_reg.DIO1 = 0;
	gpio_ctrl_reg.DIR0 = 0;
	gpio_ctrl_reg.DIR1 = 0;
	gpio_ctrl_reg.GPIO0_EN = 1; //disable external clock input
	gpio_ctrl_reg.GPIO1_EN = 1; //disable sync input
	max11254_hal_write_reg(MAX11254_GPIO_CTRL_OFFSET, &gpio_ctrl_reg);

	MAX11254_SEQ seq_reg;
	seq_reg.MODE = _mode;
	seq_reg.MDREN = 0;
	seq_reg.GPODREN = 0;
	seq_reg.RDYBEN = 0;
	seq_reg.MUX = firstSetBit(_channels);
	max11254_hal_write_reg(MAX11254_SEQ_OFFSET, &seq_reg);	

	max11254_hal_send_command(MAX11254_Command_Mode::SEQUENCER, MAX11254_Rate::CONT_32000_SPS);

    return true;
}

/**
 * @brief Returns the position of the first set bit in a 32 bit value.
 * 
 * @param value 
 * @return uint32_t 
 */
uint32_t firstSetBit(uint32_t value)
{
    uint32_t pos = 0;
    while((value & 0x01) == 0)
    {
        value >>= 1;
        pos++;
    }
    return pos;
}

/**
 * @brief Returns the index of the array element that is closest to the given value.
 * 
 * @tparam T 
 * @param value 
 * @param array 
 * @param arrayLength 
 * @return uint32_t 
 */
template <typename T>
uint32_t getNearestIndex(T value, const T *array, uint32_t arrayLength)
{
    uint32_t index = 0;
    T minDiff = abs((float)value - (float)array[0]);
    for(uint32_t i=1; i<arrayLength; i++)
    {
        T diff = abs((float)value - (float)array[i]);
        if(diff < minDiff)
        {
            minDiff = diff;
            index = i;
        }
    }
    return index;
}

