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

// defines
#define USE_CALIBRATION 0

// register addresses

#define MAX11254_STAT_OFFSET 0  // reg size = 24 bits
#define MAX11254_CTRL1_OFFSET 1 // reg size = 8 bits
#define MAX11254_CTRL2_OFFSET 2 // reg size = 8 bits (FOR GPIO pins)
#define MAX11254_CTRL3_OFFSET 3 // reg size = 8 bits
#define MAX11254_GPIO_CTRL_OFFSET 4
#define MAX11254_DELAY_OFFSET 5
#define MAX11254_CHMAP1_OFFSET 6
#define MAX11254_CHMAP0_OFFSET 7
#define MAX11254_SEQ_OFFSET 8
#define MAX11254_GPO_DIR_OFFSET 9
#define MAX11254_SOC_OFFSET 10  // reg size = 24 bits		(System Offset Calibration)
#define MAX11254_SGC_OFFSET 11  // reg size = 24 bits		(System Gain Calibration)
#define MAX11254_SCOC_OFFSET 12 // reg size = 24 bits		(Self-calibration Offset)
#define MAX11254_SCGC_OFFSET 13 // reg size = 24 bits		(Self-calibration Gain)
#define MAX11254_DATA0_OFFSET 14
#define MAX11254_DATA1_OFFSET 15
#define MAX11254_DATA2_OFFSET 16
#define MAX11254_DATA3_OFFSET 17
#define MAX11254_DATA4_OFFSET 18
#define MAX11254_DATA5_OFFSET 19

/***** ENUMs ***************************************/

// Power down mode
enum class MAX11254_PowerDown
{
    CONVERSION = 0x00,
    SLEEP = 0x01,
    STANDBY = 0x02, // default
    RESET = 0x03
};

// Gain of the PGA
enum class MAX11254_Gain
{
    GAIN1 = 0x00,
    GAIN2 = 0x01,
    GAIN4 = 0x02,
    GAIN8 = 0x03,
    GAIN16 = 0x04,
    GAIN32 = 0x05,
    GAIN64 = 0x06,
    GAIN128 = 0x07
};

// Sample rate either single or continuous conversion
enum class MAX11254_Rate
{
    CONT_1_9_SPS = 0b0000,
    CONT_3_9_SPS = 0b0001,
    CONT_7_8_SPS = 0b0010,
    CONT_15_6_SPS = 0b0011,
    CONT_31_2_SPS = 0b0100,
    CONT_62_5_SPS = 0b0101,
    CONT_125_SPS = 0b0110,
    CONT_250_SPS = 0b0111,
    CONT_500_SPS = 0b1000,
    CONT_1000_SPS = 0b1001,
    CONT_2000_SPS = 0b1010,
    CONT_4000_SPS = 0b1011,
    CONT_8000_SPS = 0b1100,
    CONT_16000_SPS = 0b1101,
    CONT_32000_SPS = 0b1110,
    CONT_64000_SPS = 0b1111,
    SINGLE_50_SPS = 0b0000,
    SINGLE_62_5_SPS = 0b0001,
    SINGLE_100_SPS = 0b0010,
    SINGLE_125_SPS = 0b0011,
    SINGLE_200_SPS = 0b0100,
    SINGLE_250_SPS = 0b0101,
    SINGLE_400_SPS = 0b0110,
    SINGLE_500_SPS = 0b0111,
    SINGLE_800_SPS = 0b1000,
    SINGLE_1000_SPS = 0b1001,
    SINGLE_1600_SPS = 0b1010,
    SINGLE_2000_SPS = 0b1011,
    SINGLE_3200_SPS = 0b1100,
    SINGLE_4000_SPS = 0b1101,
    SINGLE_6400_SPS = 0b1110,
    SINGLE_12800_SPS = 0b1111
};

// Calibration mode
enum class MAX11254_Calibration_Mode
{
    SELF = 0b00,
    SYS_OFFSET = 0b01,
    SYS_FULL_SCALE = 0b10,
    RESEREVED = 0b11
};

// Command modes
enum class MAX11254_Command_Mode
{
    UNUSED = 0b00,
    POWER_DOWN = 0b01,
    CALIBRATION = 0b10,
    SEQUENCER = 0b11
};

// Sequencer mode
enum class MAX11254_Seq_Mode
{
    SEQ_MODE_1 = 0b00,
    SEQ_MODE_2 = 0b01,
    SEQ_MODE_3 = 0b10,
    RESERVED = 0b11
};

/***** STAT Register    ***********************************/
// STAT : Status Register
struct __attribute__((packed)) MAX11254_STAT
{
    uint RDY : 1;
    uint MSTAT : 1;
    MAX11254_PowerDown PDSTAT : 2;
    MAX11254_Rate RATE : 4;
    uint AOR : 1;
    uint DOR : 1;
    uint SYSGOR : 1;
    uint ERROR : 1;
    uint GPOERR : 1;
    uint ORDERR : 1;
    uint REFDET : 1;
    uint SCANERR : 1;
    uint SRDY : 6;
    uint INRESET : 1;
    uint padding : 1;

    // default value
    MAX11254_STAT() : RDY(0), MSTAT(0), PDSTAT(MAX11254_PowerDown::STANDBY), 
                        RATE(MAX11254_Rate::CONT_32000_SPS), AOR(0), DOR(0), 
                        SYSGOR(0), ERROR(0), GPOERR(0), ORDERR(0), REFDET(0), 
                        SCANERR(0), SRDY(0), INRESET(0), padding(0) {}
};

/***** CTRL1 Register   ***********************************/
// CTRL1 : Control  Register 1
struct __attribute__((packed)) MAX11254_CTRL1
{
    uint8_t CONTSC : 1;
    uint8_t SCYCLE : 1;
    uint8_t FORMAT : 1;
    uint8_t Unipolar : 1;
    MAX11254_PowerDown PD : 2;
    MAX11254_Calibration_Mode CAL : 2;

    // default value
    MAX11254_CTRL1() : CONTSC(1), SCYCLE(0), FORMAT(0), Unipolar(0), 
    PD(MAX11254_PowerDown::STANDBY), CAL(MAX11254_Calibration_Mode::SELF) {}
};

/***** CTRL2 Register   ***********************************/
// CTRL2 : Control   Register 2
struct __attribute__((packed)) MAX11254_CTRL2
{
    MAX11254_Gain PGAG : 3;
    uint8_t PGAEN : 1;
    uint8_t LPMODE : 1;
    uint8_t LDOEN : 1;
    uint8_t CCSEN : 1;
    uint8_t EXTCLK : 1;

    // default value
    MAX11254_CTRL2() : PGAG(MAX11254_Gain::GAIN128), PGAEN(0), LPMODE(0),
                        LDOEN(1), CCSEN(0), EXTCLK(0) {}
};

/***** CTRL3 Register   ***********************************/
// CTRL3 : Control   Register 3
struct __attribute__((packed)) MAX11254_CTRL3
{
    uint8_t NOSCO : 1;
    uint8_t NOSCG : 1;
    uint8_t NOSYSO : 1;
    uint8_t NOSYSG : 1;
    uint8_t CALREGSEL : 1;
    uint8_t SYNC_MODE : 1;
    uint8_t GPO_MODE : 1;

    // default value
    MAX11254_CTRL3() : NOSCO(1), NOSCG(1), NOSYSO(1), NOSYSG(1), 
                        CALREGSEL(1), SYNC_MODE(0), GPO_MODE(0) {}
};

/***** GPIO_CTRL Register   ********************************/
// GPIO_CTRL: GPIO Control   Register
struct __attribute__((packed)) MAX11254_GPIO_CTRL
{
    uint8_t DIO0 : 1;
    uint8_t DIO1 : 1;
    uint8_t _reserved0 : 1;
    uint8_t DIR0 : 1;
    uint8_t DIR1 : 1;
    uint8_t _reserved1 : 1;
    uint8_t GPIO0_EN : 1;
    uint8_t GPIO1_EN : 1;

    // default value
    MAX11254_GPIO_CTRL() : DIO0(0), DIO1(0), _reserved0(0), DIR0(0), DIR1(0), 
                            _reserved1(0), GPIO0_EN(1), GPIO1_EN(1) {}
};

/***** DELAY Register   ***********************************/
// DELAY: Delay Register
struct __attribute__((packed)) MAX11254_DELAY
{
    uint8_t GPO : 8;
    uint8_t MUX : 8;

    // default value
    MAX11254_DELAY() : GPO(0), MUX(0) {}
};

/***** CHMAP1 Register  **********************************/
// CHMAP1: Channel Map   Register 1
struct __attribute__((packed)) MAX11254_CHMAP1
{
    uint8_t CH3_GPOEN : 1;
    uint8_t CH3_EN : 1;
    uint8_t CH3_ORD : 3;
    uint8_t CH3_GPO0 : 1;
    uint8_t CH3_GPO1 : 1;
    uint8_t reserved1 : 1;
    uint8_t CH4_GPOEN : 1;
    uint8_t CH4_EN : 1;
    uint8_t CH4_ORD : 3;
    uint8_t CH4_GPO0 : 1;
    uint8_t CH4_GPO1 : 1;
    uint8_t reserved2 : 1;
    uint8_t CH5_GPOEN : 1;
    uint8_t CH5_EN : 1;
    uint8_t CH5_ORD : 3;
    uint8_t CH5_GPO0 : 1;
    uint8_t CH5_GPO1 : 1;
    uint8_t reserved3 : 1;

    // default value
    MAX11254_CHMAP1() : CH3_GPOEN(0), CH3_EN(0), CH3_ORD(0), CH3_GPO0(0), 
                        CH3_GPO1(0), reserved1(0), CH4_GPOEN(0), CH4_EN(0), 
                        CH4_ORD(0), CH4_GPO0(0), CH4_GPO1(0), reserved2(0), 
                        CH5_GPOEN(0), CH5_EN(0), CH5_ORD(0), CH5_GPO0(0), 
                        CH5_GPO1(0), reserved3(0) {}
};

/***** CHMAP0 Register  **********************************/
// CHMAP0: Channel Map   Register 0
struct __attribute__((packed)) MAX11254_CHMAP0
{
    uint8_t CH0_GPOEN : 1;
    uint8_t CH0_EN : 1;
    uint8_t CH0_ORD: 3;
    uint8_t CH0_GPO0 : 1;
    uint8_t CH0_GPO1 : 1;
    uint8_t reserved1 : 1;
    uint8_t CH1_GPOEN : 1;
    uint8_t CH1_EN : 1;
    uint8_t CH1_ORD : 3;
    uint8_t CH1_GPO0 : 1;
    uint8_t CH1_GPO1 : 1;
    uint8_t reserved2 : 1;
    uint8_t CH2_GPOEN : 1;
    uint8_t CH2_EN : 1;
    uint8_t CH2_ORD : 3;
    uint8_t CH2_GPO0 : 1;
    uint8_t CH2_GPO1 : 1;
    uint8_t reserved3 : 1;

    // default value
    MAX11254_CHMAP0() : CH0_GPOEN(0), CH0_EN(0), CH0_ORD(0), CH0_GPO0(0), 
                        CH0_GPO1(0), reserved1(0), CH1_GPOEN(0), CH1_EN(0), 
                        CH1_ORD(0), CH1_GPO0(0), CH1_GPO1(0), reserved2(0), 
                        CH2_GPOEN(0), CH2_EN(0), CH2_ORD(0), CH2_GPO0(0), 
                        CH2_GPO1(0), reserved3(0) {}
};

/***** SEQ Register     *************************************/
// SEQ: Sequence Register
struct __attribute__((__packed__)) MAX11254_SEQ
{
    uint8_t RDYBEN : 1;
    uint8_t MDREN : 1;
    uint8_t GPODREN : 1;
    MAX11254_Seq_Mode MODE : 2;
    uint8_t MUX : 3;

    // default value
    MAX11254_SEQ() : RDYBEN(0), MDREN(0), GPODREN(0), 
                        MODE(MAX11254_Seq_Mode::SEQ_MODE_1), MUX(3) {}
};

/***** GPO_DIR Register     **********************************/
// GPO_DIR: GPO Direction    Register
struct __attribute__((packed)) MAX11254_GPO_DIR
{
    uint8_t GPO0_DIR : 1;
    uint8_t GPO1_DIR : 1;
    uint8_t reserved : 6;

    // default value
    MAX11254_GPO_DIR() : GPO0_DIR(0), GPO1_DIR(0), reserved(0) {}
};

/* Exported functions   ------------------------------------------------------- */
void max11254_hal_init(spi_inst_t *spi, uint32_t csPin);

#if USE_CALIBRATION
void max11254_hal_calibration(void);
void max11254_hal_self_calib(void);
void max11254_hal_sys_offset_calib(void);
void max11254_hal_sys_gain_calib(void);
#endif

uint8_t max11254_hal_meas_status(void);

uint32_t max11254_hal_read_reg(uint8_t reg, void *data = NULL);
void max11254_hal_send_command(MAX11254_Command_Mode mode, MAX11254_Rate rate);
void max11254_hal_write_reg(uint8_t reg, void *value);

/*****	END OF FILE	****/