#include "i2c.h"

#include <stdlib.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/dma.h"
#include "hardware/irq.h"

#include "pico/multicore.h" // <- for multicore save malloc
#include "pico/malloc.h"
#include "pico/mem_ops.h"

#include "debug_pins.h"

// Type defines
enum TransactionDirection {
    TRANSDIR_NONE = 0,
    TRANSDIR_HIDO = 1,
    TRANSDIR_HODI = 2
};

enum TransactionType {
    TRANSTYPE_NONE = 0,
    TRANSTYPE_REGISTER = 1,
    TRANSTYPE_PDO = 2,
    TRANSTYPE_ERROR = 3
};

enum TransactionPhase {
    TRANSPHASE_NONE = 0,
    TRANSPHASE_CMD_RECEIVED = 1,
    TRANSPHASE_FIRST_BYTE_SEND = 2,
    TRANSPHASE_DONE = 3
};

enum FIFO_DIRECTION {
    FIFO_DIR_TX = 0,
    FIFO_DIR_RX = 1
};


// global variables
static TransactionDirection g_transactionDir = TRANSDIR_NONE;
static TransactionType g_transactionType = TRANSTYPE_NONE;
static TransactionPhase g_transactionPhase = TRANSPHASE_NONE;
static uint32_t g_transactionAddr = 0xFF; // 0xFF = none, 0x00 - 0x3F = address

//hardware
static i2c_inst_t *g_i2c;
static uint32_t g_i2cAddr;
static uint32_t g_sdaPin;
static uint32_t g_sclPin;

// buffer related variables
static volatile uint16_t *g_HInStreamBuffer[2] = {0}; // Buffer that contains the pds data and the one byte status.
static volatile uint16_t *g_HInPdsData[2] = {0};
static volatile uint32_t g_pdsDataLen = 0;
static volatile uint8_t *g_HOutStreamBuffer[2] = {0}; // Buffer that contains the HOut pds data
static volatile uint32_t g_HOutStreamBufferLen = 0;
static volatile uint32_t g_HOutStreamBufferIndex = 0;
static volatile uint32_t g_activePdsRxChannel = 0;
static volatile uint32_t g_activePdsTxChannel = 0;
static volatile uint32_t g_PdsChannelFull = 0;
static volatile bool g_pdsOverflow = false;
static volatile bool g_pdsUnderflow = false;
static volatile uint16_t *g_registerData[2] = {0};

//dma
static dma_channel_config g_i2cTxDmaConfig;
static uint32_t g_i2cTxDmaChan;
static dma_channel_config g_i2cRxDmaConfig;
static uint32_t g_i2cRxDmaChan;

// callbacks
static void (*H_Out_NotifyPdsBufferFull)(uint32_t bufferIndex);
static bool (*H_In_GetRegisterCallback)(void* buffer, uint32_t *length, uint32_t registerAddr);
static bool (*H_In_GetStatusCallback)(uint8_t *status);
static bool (*H_Out_RegisterCallback)(void* buffer, uint32_t length, uint32_t registerAddr);

static void (*sync_callback)(void);

// private prototype functions
static void __isr __not_in_flash_func(i2c_slave_irq_handler)(void);
void __isr __not_in_flash_func(i2c_dma_irq_handler)(void);

static void __always_inline H_IN_PdsData(void);
static void __always_inline H_In_RegisterTransfer(uint32_t addr);
static void H_Out_PdsData(void);
static void __always_inline H_Out_RegisterTransfer(uint32_t addr);

void I2C_Init(i2cInitConfiguration_t *initConfiguration)
{
    // Set hardware configuration variables
    g_i2c = initConfiguration->i2c;
    g_sdaPin = initConfiguration->sdaPin;
    g_sclPin = initConfiguration->sclPin;
    g_i2cAddr = initConfiguration->i2cAddr;

    // Set callback functions
    H_Out_NotifyPdsBufferFull = initConfiguration->H_Out_NotifyPdsBufferFull;
    H_In_GetRegisterCallback = initConfiguration->H_In_GetRegisterCallback;
    H_In_GetStatusCallback = initConfiguration->H_In_GetStatusCallback;
    H_Out_RegisterCallback = initConfiguration->H_Out_RegisterCallback;
    sync_callback = initConfiguration->sync_callback;

    // Set HOut buffer pointers
    g_HOutStreamBuffer[0] = (uint8_t*) initConfiguration->HOut_pdsBuffer;
    g_HOutStreamBuffer[1] = g_HOutStreamBuffer[0] + initConfiguration->HOut_pdsLength;
    g_HOutStreamBufferLen = initConfiguration->HOut_pdsLength;

    // I2C Initialisation. Using it at 1 MHz.
    i2c_init(g_i2c, 1000*1000);
    i2c_set_slave_mode(g_i2c, true, g_i2cAddr);
    
    gpio_set_function(g_sdaPin, GPIO_FUNC_I2C);
    gpio_set_function(g_sclPin, GPIO_FUNC_I2C);
    gpio_pull_up(g_sdaPin);
    gpio_pull_up(g_sclPin);

    // Malloc memory for the HIn PDS and register data buffers
    g_pdsDataLen = initConfiguration->HIn_pdsLength;
    g_HInStreamBuffer[0] = (uint16_t*)calloc((g_pdsDataLen + 1) * 2, 1);
    g_HInStreamBuffer[1] = (uint16_t*)calloc((g_pdsDataLen + 1) * 2, 1);
    g_HInPdsData[0] = g_HInStreamBuffer[0]+1;
    g_HInPdsData[1] = g_HInStreamBuffer[1]+1;
    g_registerData[0] = (uint16_t*)calloc(initConfiguration->longestRegisterLength * 2, 1);
    g_registerData[1] = (uint16_t*)calloc(initConfiguration->longestRegisterLength * 2, 1);

    // Fill buffers with debug data, as they will be overwritten anyway.
    for (uint32_t i = 0; i < g_pdsDataLen; i++)
    {
        g_HInPdsData[0][i] = i & 0xFF;
        g_HInPdsData[1][i] = i & 0xFF;
    }


    // init i2c dma
    // tx
    g_i2cTxDmaChan = dma_claim_unused_channel(true);
    g_i2cTxDmaConfig = dma_channel_get_default_config(g_i2cTxDmaChan);
    channel_config_set_transfer_data_size(&g_i2cTxDmaConfig, DMA_SIZE_16);
    channel_config_set_read_increment(&g_i2cTxDmaConfig, true);
    channel_config_set_write_increment(&g_i2cTxDmaConfig, false);
    channel_config_set_dreq(&g_i2cTxDmaConfig, i2c_get_dreq(g_i2c, true));
    dma_channel_configure(
        g_i2cTxDmaChan,              // Channel to be configured
        &g_i2cTxDmaConfig,             // The configuration we just created
        &g_i2c->hw->data_cmd,   // The write address
        0,           // The read address
        0,              // Number of transfers; 
        false           // Don't start yet
    );

    // rx
    g_i2cRxDmaChan = dma_claim_unused_channel(true);
    g_i2cRxDmaConfig = dma_channel_get_default_config(g_i2cRxDmaChan);
    channel_config_set_transfer_data_size(&g_i2cRxDmaConfig, DMA_SIZE_8);
    channel_config_set_read_increment(&g_i2cRxDmaConfig, false);
    channel_config_set_write_increment(&g_i2cRxDmaConfig, true);
    channel_config_set_dreq(&g_i2cRxDmaConfig, i2c_get_dreq(g_i2c, false));
    dma_channel_configure(
        g_i2cRxDmaChan,              // Channel to be configured
        &g_i2cRxDmaConfig,             // The configuration we just created
        0,   // The write address
        &g_i2c->hw->data_cmd,           // The read address
        0,              // Number of transfers; 
        false           // Don't start yet
    );

    // Enable dma interrupts
    dma_channel_set_irq0_enabled(g_i2cRxDmaChan, true);
    dma_channel_set_irq0_enabled(g_i2cTxDmaChan, true);
    irq_set_exclusive_handler(DMA_IRQ_0, i2c_dma_irq_handler);
    irq_set_enabled(DMA_IRQ_0, true);
    
    // Only enable interrupts we want to use
    g_i2c->hw->intr_mask =
            I2C_IC_INTR_MASK_M_RX_FULL_BITS | I2C_IC_INTR_MASK_M_RD_REQ_BITS | I2C_IC_RAW_INTR_STAT_TX_ABRT_BITS |
            I2C_IC_INTR_MASK_M_STOP_DET_BITS | I2C_IC_INTR_MASK_M_RX_DONE_BITS | I2C_IC_INTR_MASK_M_GEN_CALL_BITS;

    // Enable interrupts
    uint32_t irq = g_i2c == &i2c0_inst ? I2C0_IRQ : I2C1_IRQ;
    irq_set_exclusive_handler(irq, i2c_slave_irq_handler);
    irq_set_enabled(irq, true);
}

/**
 * @brief Send a pds buffer to the i2c-unit to send it to the host.
 * 
 * @param data  The data to send.
 * @param len   The length of the data in bytes.
 */
void I2C_send_H_In_PDSData(uint8_t *data, uint32_t len)
{
    // Check if the data fits into the buffer.
    assert(len == g_pdsDataLen);
    g_activePdsRxChannel = !g_activePdsRxChannel;
    if(g_PdsChannelFull & (1 << g_activePdsRxChannel))
    {
        // Channel full -> overflow
        g_pdsOverflow = true;
    }
    
    // Transform the data from byte array to 16 bit array.
    volatile uint16_t *buffer16 = g_HInPdsData[g_activePdsRxChannel];
    for (uint32_t i = 0; i < len; i++)
    {
        buffer16[i] = data[i] & 0xFF;
    }

    // Write status byte to the beginnign of the buffer.
    uint16_t status = 0;
    H_In_GetStatusCallback((uint8_t*)&status);
    g_HInStreamBuffer[g_activePdsRxChannel][0] = status & 0xFF;

    g_PdsChannelFull |= 1 << g_activePdsRxChannel;
}

// ---- private functions ----

/**
 * @brief Prepares the tx dma to send the pdo to host.
 * 
 */
static void __always_inline H_IN_PdsData(void) {
    // Write data to the tx fifo.
    auto hw = g_i2c->hw;
    g_activePdsTxChannel = !g_activePdsTxChannel;
    if(!(g_PdsChannelFull & (1 << g_activePdsTxChannel)))
    {
        // Channel not full -> underflow
        g_pdsUnderflow = true;
    }
    dma_channel_set_trans_count(g_i2cTxDmaChan, g_pdsDataLen + 1, false);
    dma_channel_set_read_addr(g_i2cTxDmaChan, g_HInStreamBuffer[g_activePdsTxChannel], true);
}


/**
 * @brief Prepares the tx dma to send value of the register with address addr to host.
 * 
 */
static void __always_inline H_In_RegisterTransfer(uint32_t addr) {
    auto hw = g_i2c->hw;
    volatile uint16_t *regData = g_registerData[FIFO_DIR_TX];
    uint32_t regLength = 0;

    // load the data from the register into the tx fifo.
    bool valid = H_In_GetRegisterCallback((void*)regData, &regLength, addr);

    // Transform the data from byte array to 16 bit array.
    // Loop backwards to avoid overwriting data.
    for (int32_t i = regLength-1; i >= 0; i--)
    {
        regData[i] = ((uint8_t*) regData)[i];
    }
    
    // Setup the dma channel to send the data.
    dma_channel_set_trans_count(g_i2cTxDmaChan, regLength, false);
    dma_channel_set_read_addr(g_i2cTxDmaChan, g_registerData[FIFO_DIR_TX], true);
}

/**
 * @brief Send the status byte to host.
 * 
 */
static void __always_inline H_In_Status(void) {
    auto hw = g_i2c->hw;

    uint16_t status = 0;
    H_In_GetStatusCallback((uint8_t*)&status);

    hw->data_cmd = status;
}

/**
 * @brief Initializes the rx dma to receive the pds from host.
 * 
 */
static void H_Out_PdsData(void)
{
    // Write data to the tx fifo.
    auto hw = g_i2c->hw;
    g_activePdsRxChannel = !g_activePdsRxChannel;
    if(!(g_PdsChannelFull & (1 << g_activePdsRxChannel)))
    {
        // Channel not full -> underflow
        g_pdsUnderflow = true;
    }

    dma_channel_set_trans_count(g_i2cRxDmaChan, g_HOutStreamBufferLen, false);
    dma_channel_set_write_addr(g_i2cRxDmaChan, g_HOutStreamBuffer[g_activePdsRxChannel], true);
}

/**
 * @brief Sets the value of the register with address addr to data.
 * 
 */
static void __always_inline H_Out_RegisterTransfer(uint32_t addr) {
    auto hw = g_i2c->hw;
    volatile uint32_t length = hw->rxflr+1;
    volatile uint8_t *regData = (uint8_t*)g_registerData[FIFO_DIR_RX];

    for (uint32_t i = 0; i < length; i++)
    {
        regData[i] = hw->data_cmd;
    }

    H_Out_RegisterCallback((void*)regData, length, addr);
}


// i2c slave handler
static void __isr __not_in_flash_func(i2c_slave_irq_handler)(void) {
    gpio_put(DEBUG_PIN3, 1);
    // Read the interrupt status register to see what caused this interrupt.
    auto hw = g_i2c->hw;
    volatile uint32_t intr_stat = hw->intr_stat;
    if (intr_stat == 0) {
        return;
    }

    // There was a general call.
    if (intr_stat & I2C_IC_INTR_STAT_R_GEN_CALL_BITS) {
        volatile uint32_t dummy = hw->clr_gen_call;
        dummy = hw->clr_gen_call;
        volatile uint32_t intr_stat_dbg = hw->intr_stat;
        dummy = hw->clr_gen_call;
        intr_stat_dbg = hw->intr_stat;
        gpio_put(DEBUG_PIN1, 0);
        gpio_put(DEBUG_PIN1, 1);

        sync_callback();

        // TODO: Check if this is the right place to send the status.
        // Write status byte to the beginnign of the output buffer.
        uint16_t status = 0;
        H_In_GetStatusCallback((uint8_t*)&status);
        g_HInStreamBuffer[g_activePdsRxChannel][0] = status & 0xFF;
    }    

    // There was data left in the tx-fifo that is now cleared.
    if (intr_stat & I2C_IC_INTR_STAT_R_TX_ABRT_BITS) {
        // Clear the abort state before attempting to write to the tx fifo again.
        volatile uint32_t tx_abrt_source = hw->tx_abrt_source;
        hw->clr_tx_abrt;
    }

    // There was a addr match and we are being asked to send data.
    if (intr_stat & I2C_IC_INTR_STAT_R_RD_REQ_BITS) {
        hw->clr_rd_req;
        
        if (g_transactionPhase == TRANSPHASE_CMD_RECEIVED && g_transactionDir != TRANSDIR_HODI)
        {
            // We did already receive the cmd, so we now what to send
            if (g_transactionType == TRANSTYPE_PDO)
            {
                H_IN_PdsData();
            }
            else if (g_transactionType == TRANSTYPE_REGISTER)
            {
                H_In_RegisterTransfer(g_transactionAddr);
            }
        }
        else
        {
            // if this is the first byte, we shall answer with a PDO.
            if(g_transactionPhase == TRANSPHASE_NONE)
            {
                g_transactionPhase = TRANSPHASE_FIRST_BYTE_SEND;
                g_transactionType = TRANSTYPE_PDO;

                H_IN_PdsData();
            }
            else
            {
                // We are not supposed to send data, so we just send 0x00
                hw->data_cmd = 0x00;
                g_transactionType = TRANSTYPE_ERROR;
            }
            
        }
    }

    // The tx fifo is empty.
    if (intr_stat & I2C_IC_INTR_STAT_R_TX_EMPTY_BITS) {
        // We have to write something into the fifo to clear the interrupt.
        
        hw->data_cmd = 0x55;
        g_transactionType = TRANSTYPE_ERROR;
    }

    if (intr_stat & I2C_IC_INTR_STAT_R_STOP_DET_BITS) {

        hw->clr_stop_det;

                // Read is terminated, so we can process the data.
        if (g_transactionDir == TRANSDIR_HODI)
        {
            // This is HoDi, so we need to set the register.
            H_Out_RegisterTransfer(g_transactionAddr);
        }

        g_transactionAddr = 0xFF;
        g_transactionDir = TRANSDIR_NONE;
        g_transactionType = TRANSTYPE_NONE;
        g_transactionPhase = TRANSPHASE_NONE;
        hw->rx_tl = 0;
    }

    if (intr_stat & I2C_IC_INTR_STAT_R_RX_FULL_BITS) {

        // Read data from the rx fifo - also clears the interrupt.
        volatile uint32_t rxReg = hw->data_cmd;   
        volatile uint32_t data = rxReg & 0xFF;

        // If this is the first byte, it may contain the cmd.
        if (rxReg & (1 << 11) && g_transactionPhase == TRANSPHASE_NONE)
        {  
            g_transactionPhase = TRANSPHASE_CMD_RECEIVED;       
            if (data & 0x80)
            {
                // this is a pds transaction.
                g_transactionAddr = 0xFF;
                g_transactionType = TRANSTYPE_PDO;

                bool HiDo = data & 0x40;
                g_transactionDir = HiDo? TRANSDIR_HIDO : TRANSDIR_HODI;
                
                if(!HiDo)
                {
                    // This is HoDi -> config the dma to receive the pds.
                    H_Out_PdsData();
                }
            }
            else
            {
                // if the msb is cleared, this is a register access.
                g_transactionAddr = data & 0x3F;
                g_transactionType = TRANSTYPE_REGISTER;

                // This is a cmd byte.
                if (data & 0x40)
                {
                    // This is HiDo.
                    g_transactionDir = TRANSDIR_HIDO;
                }
                else
                {
                    // This is HoDi.
                    g_transactionDir = TRANSDIR_HODI;

                    hw->rx_tl = 15;
                }
            }
        }
    }

    if (intr_stat & I2C_IC_INTR_STAT_R_RX_DONE_BITS) {
        hw->clr_rx_done;
        
    }

    // There shouldn't be any interrupts that were there at IRQ entry we didn't handle.
    if ((hw->intr_stat & intr_stat ) & ~0x800) {
        volatile uint32_t intr_stat_dbg = hw->intr_stat;
        volatile uint32_t tx_abrt_source = hw->tx_abrt_source;
        gpio_put(DEBUG_PIN1, 0);
        __breakpoint();
    }

    gpio_put(DEBUG_PIN3, 0);
}


void __isr __not_in_flash_func(i2c_dma_irq_handler)(void)
{
    if(dma_channel_get_irq0_status(g_i2cTxDmaChan))
    {
        dma_channel_acknowledge_irq0(g_i2cTxDmaChan);
        // The pdo transfer is done.
        g_PdsChannelFull &= ~(1 << g_activePdsTxChannel);
    }
    if(dma_channel_get_irq0_status(g_i2cRxDmaChan))
    {
        dma_channel_acknowledge_irq0(g_i2cRxDmaChan);
        // The HOut transfer is done.
        
        // Check if pds or register data was received.
        if (g_transactionType == TRANSTYPE_PDO)
        {
            // Swap the active pds channel as fast as possible.
            g_activePdsRxChannel = !g_activePdsRxChannel;

            // Notify the application that the pds buffer is full, but use the old buffer index.
            H_Out_NotifyPdsBufferFull(!g_activePdsRxChannel);

        }
        else if (g_transactionType == TRANSTYPE_REGISTER)
        {
            // Notify the application that the register data was received.

            // TODO: Get the right length.
            panic_unsupported();
            //H_Out_RegisterCallback((void*)g_registerData[FIFO_DIR_RX], g_i2c->hw->rxflr+1, g_transactionAddr);
        }
        
        
    }
}