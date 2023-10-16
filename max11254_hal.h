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

#ifndef __MAX11254_H
#define __MAX11254_H
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"


// defines
#define USE_CALIBRATION 0

// register addresses

#define	MAX11254_STAT_OFFSET        0	// reg size = 24 bits
#define	MAX11254_CTRL1_OFFSET       1	// reg size = 8 bits
#define	MAX11254_CTRL2_OFFSET       2	// reg size = 8 bits (FOR GPIO pins)
#define	MAX11254_CTRL3_OFFSET       3	// reg size = 8 bits
#define MAX11254_GPIO_CTRL_OFFSET   4
#define MAX11254_DELAY_OFFSET       5
#define MAX11254_CHMAP1_OFFSET      6
#define MAX11254_CHMAP0_OFFSET      7
#define MAX11254_SEQ_OFFSET         8
#define MAX11254_GPO_DIR_OFFSET     9          
#define	MAX11254_SOC_OFFSET     	10	// reg size = 24 bits		(System Offset Calibration)
#define	MAX11254_SGC_OFFSET     	11  // reg size = 24 bits		(System Gain Calibration)
#define	MAX11254_SCOC_OFFSET        12	// reg size = 24 bits		(Self-calibration Offset)
#define	MAX11254_SCGC_OFFSET        13	// reg size = 24 bits		(Self-calibration Gain)
#define MAX11254_DATA0_OFFSET       14
#define MAX11254_DATA1_OFFSET       15
#define MAX11254_DATA2_OFFSET       16
#define MAX11254_DATA3_OFFSET       17
#define MAX11254_DATA4_OFFSET       18
#define MAX11254_DATA5_OFFSET       19

// COMMAND BYTES  (MODE = 0, LINEF = 0)
//CTRL1 : Control Register 1
//This are all masked.  Need to read then 'AND' with register
//Calibration
#define	SELF_CALIB				0x3f  // [Bit7: 0, Bit6: 0, Bit5: x, Bit4: x, Bit3: x, Bit2: x, Bit1: x, Bit0: x]
#define	SLOC_CALIB				0x7f  // [Bit7: 0, Bit6: 1, Bit5: x, Bit4: x, Bit3: x, Bit2: x, Bit1: x, Bit0: x]
#define	SYSF_CALIB				0xbf  // [Bit7: 1, Bit6: 0, Bit5: x, Bit4: x, Bit3: x, Bit2: x, Bit1: x, Bit0: x]
#define	DNU_CALIB 				0xff  // [Bit7: 1, Bit6: 1, Bit5: x, Bit4: x, Bit3: x, Bit2: x, Bit1: x, Bit0: x] DO NOT USE
//Power state
#define	POWER_NOP 				0xcf  // [Bit7: x, Bit6: x, Bit5: 0, Bit4: 0, Bit3: x, Bit2: x, Bit1: x, Bit0: x]
#define	POWER_SLEEP 		  0xdf  // [Bit7: x, Bit6: x, Bit5: 0, Bit4: 1, Bit3: x, Bit2: x, Bit1: x, Bit0: x]
#define	POWER_STANDBY 		0xef  // [Bit7: x, Bit6: x, Bit5: 1, Bit4: 0, Bit3: x, Bit2: x, Bit1: x, Bit0: x]
#define	POWER_RESET 			0xfF  // [Bit7: x, Bit6: x, Bit5: 1, Bit4: 1, Bit3: x, Bit2: x, Bit1: x, Bit0: x]
//Conversion type
#define CONN_POLAR_TYPE   0xF7  // [Bit7: x, Bit6: x, Bit5: x, Bit4: x, Bit3: 0, Bit2: x, Bit1: x, Bit0: x] 
//Format
#define DATA_FORMAT       0xfb  // [Bit7: x, Bit6: x, Bit5: x, Bit4: x, Bit3: x, Bit2: 0, Bit1: x, Bit0: x]
//Conversion scycle type: Single vs Continuous
#define CONV_SCYCLE       0xfd  // [Bit7: x, Bit6: x, Bit5: x, Bit4: x, Bit3: x, Bit2: x, Bit1: 0, Bit0: x]
//Conversion scycle type: Single vs Continuous
#define CONV_CONTSC       0xfe  // [Bit7: x, Bit6: x, Bit5: x, Bit4: x, Bit3: x, Bit2: x, Bit1: x, Bit0: 0]

struct __attribute__((packed)) MAX11254_CTRL1 {
  uint8_t CONTSC : 1;
  uint8_t SCYCLE : 1;
  uint8_t FORMAT : 1;
  uint8_t Unipolar : 1;
  uint8_t PD : 2;
  uint8_t CAL : 2;
};

//***********************************************************************************************************
//CTRL2 : Control Register 2

enum class PGA_GAIN {
  GAIN1 = 0x00,
  GAIN2 = 0x01,
  GAIN4 = 0x02,
  GAIN8 = 0x03,
  GAIN16 = 0x04,
  GAIN32 = 0x05,
  GAIN64 = 0x06,
  GAIN128 = 0x07
};

struct __attribute__((packed)) MAX11254_CTRL2 {
    PGA_GAIN PGAG : 3;
    uint8_t PGAEN : 1;
    uint8_t LPMODE : 1;
    uint8_t LDOEN : 1;
    uint8_t CCSEN : 1;
    uint8_t EXTCLK : 1;
};

//***********************************************************************************************************
//CTRL3 : Control Register 3
//This are all masked. 
#define GPO_MODE  6
#define SYNC_MODE 5
#define CALREGSEL 4
#define NOSYSG    3
#define NOSYSO    2
#define NOSCG     1
#define NOSCO     0
//***********************************************************************************************************
//GPIO_CTRL: GPIO Control Register
#define GPIO_CTRL_REG 0x40
//  #define	SYS_OFFSET_CALIB	0xA0
//  #define	SYS_GAIN_CALIB		0xB0
#define	MEASURE_1_SPS			0x80
#define	MEASURE_2p5_SPS		0x81
#define	MEASURE_5_SPS			0x82
#define	MEASURE_10_SPS		0x83
#define	MEASURE_15_SPS		0x84
#define	MEASURE_30_SPS		0x85
#define	MEASURE_60_SPS		0x86
#define	MEASURE_120_SPS		0x87
//***********************************************************************************************************

//Command BYTE
// Enum: MAX11254 Sample Rates
enum class MAX11254_SampleRate {
    RATE_1_9_SPS_CONT = 0b0000,
    RATE_3_9_SPS_CONT = 0b0001,
    RATE_7_8_SPS_CONT = 0b0010,
    RATE_15_6_SPS_CONT = 0b0011,
    RATE_31_2_SPS_CONT = 0b0100,
    RATE_62_5_SPS_CONT = 0b0101,
    RATE_125_SPS_CONT = 0b0110,
    RATE_250_SPS_CONT = 0b0111,
    RATE_500_SPS_CONT = 0b1000,
    RATE_1000_SPS_CONT = 0b1001,
    RATE_2000_SPS_CONT = 0b1010,
    RATE_4000_SPS_CONT = 0b1011,
    RATE_8000_SPS_CONT = 0b1100,
    RATE_16000_SPS_CONT = 0b1101,
    RATE_32000_SPS_CONT = 0b1110,
    RATE_64000_SPS_CONT = 0b1111,
    RATE_50_SPS_SINGLE = 0b0000,
    RATE_62_5_SPS_SINGLE = 0b0001,
    RATE_100_SPS_SINGLE = 0b0010,
    RATE_125_SPS_SINGLE = 0b0011,
    RATE_200_SPS_SINGLE = 0b0100,
    RATE_250_SPS_SINGLE = 0b0101,
    RATE_400_SPS_SINGLE = 0b0110,
    RATE_500_SPS_SINGLE = 0b0111,
    RATE_800_SPS_SINGLE = 0b1000,
    RATE_1000_SPS_SINGLE = 0b1001,
    RATE_1600_SPS_SINGLE = 0b1010,
    RATE_2000_SPS_SINGLE = 0b1011,
    RATE_3200_SPS_SINGLE = 0b1100,
    RATE_4000_SPS_SINGLE = 0b1101,
    RATE_6400_SPS_SINGLE = 0b1110,
    RATE_12800_SPS_SINGLE = 0b1111
};
enum class MAX11254_Command_Mode {
    MODE_UNUSED = 0b00,
    MODE_POWER_DOWN = 0b01,
    MODE_CALIBRATION = 0b10,
    MODE_SEQUENCER = 0b11
};


// SEQ Register
struct __attribute__((__packed__)) MAX11254_SEQ {
    uint8_t RDYBEN : 1;
    uint8_t MDREN : 1;
    uint8_t GPODREN : 1;
    uint8_t MODE : 2;
    uint8_t MUX : 3;
};

/* Exported functions ------------------------------------------------------- */
void max11254_hal_init(spi_inst_t *spi, uint32_t csPin);

#if USE_CALIBRATION
void max11254_hal_calibration(void);
void max11254_hal_self_calib(void);
void max11254_hal_sys_offset_calib(void);
void max11254_hal_sys_gain_calib(void);
#endif

uint8_t 	max11254_hal_meas_status(void);

uint32_t   	max11254_hal_read_reg(uint8_t reg);
void max11254_hal_send_command(MAX11254_Command_Mode mode, MAX11254_SampleRate rate);
void        max11254_hal_write_reg(uint8_t reg, uint32_t value);
#endif /* __MAX11254_H */

/*****	END OF FILE	****/