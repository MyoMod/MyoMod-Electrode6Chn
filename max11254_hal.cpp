/**
  ******************************************************************************
  MAX11254
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include "max11254_hal.h"

// Defines
static const char *deviceName = "MAX11254";

// global variables
static spi_inst_t *g_spi;
static uint32_t g_csPin;

/* Private function prototypes -----------------------------------------------*/
static inline uint32_t getRegLength(uint8_t reg);

// public functions

void max11254_hal_init(spi_inst_t *spi, uint32_t csPin)
{
	g_spi = spi;
	g_csPin = csPin;

#if 1
	// Sequence from DATA sheet page 27
	MAX11254_SEQ seq_reg;
	seq_reg.MODE = 0;
	seq_reg.MDREN = 0;
	seq_reg.GPODREN = 0;
	seq_reg.RDYBEN = 0;
	seq_reg.MUX = 3;
	max11254_hal_write_reg(MAX11254_SEQ_OFFSET, *(uint8_t *)&seq_reg);

	// Disable sync Input
	max11254_hal_write_reg(MAX11254_GPIO_CTRL_OFFSET, 0xC0);

	// Disable all calibrations and disable sync
	max11254_hal_write_reg(MAX11254_CTRL3_OFFSET, 0x0F);

	// CTRL2 is ok for now
	MAX11254_CTRL2 ctrl2_reg;
	ctrl2_reg.CCSEN = 0;
	ctrl2_reg.EXTCLK = 0;
	ctrl2_reg.LDOEN = 1;
	ctrl2_reg.LPMODE = 0;
	ctrl2_reg.PGAEN = 1;
	ctrl2_reg.PGAG = PGA_GAIN::GAIN128;
	max11254_hal_write_reg(MAX11254_CTRL2_OFFSET, *(uint8_t *)&ctrl2_reg);

	MAX11254_CTRL1 ctrl1_reg;
	ctrl1_reg.CAL = 0;
	ctrl1_reg.PD = 2;
	ctrl1_reg.Unipolar = 0;
	ctrl1_reg.FORMAT = 0;
	ctrl1_reg.SCYCLE = 0;
	ctrl1_reg.CONTSC = 1;
	max11254_hal_write_reg(MAX11254_CTRL1_OFFSET, *(uint8_t *)&ctrl1_reg);

	max11254_hal_send_command(MAX11254_Command_Mode::MODE_SEQUENCER, MAX11254_SampleRate::RATE_2000_SPS_CONT);

#endif
}

/*
	Read 8 or 24 bit register.
	8bit reg:	STAT1, CTRL1, CTRL2, CTRL3
	24bit reg:	DATA, SOC, SGC, SCOC, SCGC
*/
uint32_t max11254_hal_read_reg(uint8_t reg)
{

	uint32_t read_command = 0xC1;
	uint32_t result = 0;
	uint32_t byteLength = getRegLength(reg);

	read_command |= (reg << 1);

	gpio_put(g_csPin, 0);
	spi_write_read_blocking(g_spi, (uint8_t*)&read_command, (uint8_t *)&result, byteLength);
	gpio_put(g_csPin, 1);
	
	// reverse byte order if 24bit register
	if (byteLength == 4)
	{
		result = __builtin_bswap32(result);
	}

	return result;
}

/*
	Write 8 or 24 bit register.
	8bit reg: 	CTRL1, CTRL2, CTRL3
	24bit reg:	SOC, SGC, SCOC, SCGC
*/
void max11254_hal_write_reg(uint8_t reg, uint32_t value)
{

	uint8_t write_command = 0xC0;
	uint32_t length = getRegLength(reg);

	write_command |= (reg << 1);
	value <<= 8;
	value |= write_command;

	gpio_put(g_csPin, 0);
	spi_write_blocking(g_spi, (uint8_t *)&value, length);
	gpio_put(g_csPin, 1);
}

/*
	Send command to MAX11254.
*/
void max11254_hal_send_command(MAX11254_Command_Mode mode, MAX11254_SampleRate rate)
{
	uint8_t value = 0x80;
	assert((uint8_t)mode <= 0x03 && (uint8_t)mode >= 0x00);
	assert((uint8_t)rate <= 0x0F);
	value |= ((uint8_t)mode << 4);
	value |= (uint8_t)rate;

	gpio_put(g_csPin, 0);
	spi_write_blocking(g_spi, &value, 1);
	gpio_put(g_csPin, 1);
}

#if USE_CALIBRATION
/*
	Calibrate MAX11254
*/
void max11254_hal_calibration()
{
	// clear registers
	max11254_hal_write_reg(CTRL3, 0x00, 0, 0); //	NOSYSG NOSYSO NOSCG NOSCO = 0; GAIN = 1;
	max11254_hal_write_reg(SOC, 0x00, 0x00, 0x00);
	max11254_hal_write_reg(SGC, 0x00, 0x00, 0x00);
	max11254_hal_write_reg(SCOC, 0x00, 0x00, 0x00);
	max11254_hal_write_reg(SCGC, 0x00, 0x00, 0x00);
	/*
	//GAIN = 1
	// Enable self calibration registers
	max11254_hal_write_reg(CTRL3, 0x18, 0, 0);	//	NOSYSG=1, NOSYSO=1, NOSCG=0, NOSCO=0; GAIN = 1;
	max11254_hal_self_calib();	// Self-calibration

	// Enable system offset register
	max11254_hal_write_reg(CTRL3, 0x10, 0, 0);	//	NOSYSG=1, NOSYSO=0, NOSCG=0, NOSCO=0; GAIN = 1;
	max11254_hal_sys_offset_calib();	// System-calibration offset

	// Enable system gain register
	max11254_hal_write_reg(CTRL3, 0x00, 0, 0);	//	NOSYSG=0, NOSYSO=0, NOSCG=0, NOSCO=0; GAIN = 1;
	max11254_hal_sys_gain_calib();	// System-calibration gain
	*/

	// GAIN = 16
	//  Enable self calibration registers
	max11254_hal_write_reg(CTRL3, 0x98, 0, 0); //	NOSYSG=1, NOSYSO=1, NOSCG=0, NOSCO=0; GAIN = 16;
	max11254_hal_self_calib();				   // Self-calibration

	// Enable system offset register
	max11254_hal_write_reg(CTRL3, 0x90, 0, 0); //	NOSYSG=1, NOSYSO=0, NOSCG=0, NOSCO=0; GAIN = 16;
	max11254_hal_sys_offset_calib();		   // System-calibration offset

	// Enable system gain register
	max11254_hal_write_reg(CTRL3, 0x80, 0, 0); //	NOSYSG=0, NOSYSO=0, NOSCG=0, NOSCO=0; GAIN = 16;
	max11254_hal_sys_gain_calib();			   // System-calibration gain
}

/*
	System self calibration
		The first level of calibration is the self-calibration where the part performs the required connections to zero and
	full scale internally.
*/
void max11254_hal_self_calib(void)
{
	max11254_hal_send_command(SELF_CALIB, 1); // SCOC
	vTaskDelay(300 / portTICK_PERIOD_MS);
}

/*
	System offset calibration
		A second level of calibration is available where the user can calibrate a system zero scale and system full scale
	by presenting a zero-scale signal or a full-scale signal to the input pins and initiating a system zero scale or
	system gain calibration command.
*/
void max11254_hal_sys_offset_calib(void)
{
	// max11254_hal_send_command(SYS_OFFSET_CALIB,1);
	vTaskDelay(300 / portTICK_PERIOD_MS);
	//	delay(300);
}

/*
	System gain calibration
		A third level of calibration allows for the user to write to the internal calibration registers through the SPI interface
	to achieve any digital offset or scaling the user requires with the following restrictions. The range of digital offset
	correction is QVREF/4. The range of digital gain correction is from 0.5 to 1.5. The resolution of offset correction is 0.5 LSB.
*/
void max11254_hal_sys_gain_calib(void)
{
	// max11254_hal_send_command(SYS_GAIN_CALIB,1);
	vTaskDelay(300 / portTICK_PERIOD_MS);
	//	delay(300);
}
#endif

/*
	Check measure status
	Return masked status register STAT1
*/
uint8_t max11254_hal_meas_status(void)
{

	return max11254_hal_read_reg(MAX11254_STAT_OFFSET) & 0x000C;
}

/* Private functions ---------------------------------------------------------*/
static inline uint32_t getRegLength(uint8_t reg)
{
	bool is8bit = ((MAX11254_CTRL1_OFFSET <= reg && reg <= MAX11254_CTRL3_OFFSET) || 
					reg == MAX11254_GPO_DIR_OFFSET || reg == MAX11254_SEQ_OFFSET);
	return (is8bit) ? 2 : 4;
}

/*****	END OF FILE	****/