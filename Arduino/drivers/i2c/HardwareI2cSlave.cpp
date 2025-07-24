#include <Arduino.h>
#include <drivers/gpio/gpio.h>
#include <drivers/irqn/irqn.h>
#include <lwmem/lwmem.h>
#include <lwrb/lwrb.h>

#include "HardwareI2cSlave.h"
#include "core_debug.h"

#define ENABLE_I2C_DEBUG 1
#if defined(ENABLE_I2C_DEBUG) && (ENABLE_I2C_DEBUG == 1)
#define I2C_DEBUG_PRINTF(fmt, ...) printf("[I2C1]" fmt, ##__VA_ARGS__)
#else
#define I2C_DEBUG_PRINTF(fmt, ...)
#endif

#if defined(I2C_SLAVE_USE_IRQN) && (I2C_SLAVE_USE_IRQN != 0)
static lwrb_t rx_rb_t;     /*!< Receive ring buffer */
static lwrb_t tx_rb_t;     /*!< Transmit ring buffer */
uint8_t *rxbuff = nullptr; /*!< pointer to i2c transfer buffer  */
uint8_t *txbuff = nullptr; /*!< pointer to i2c transfer buffer  */
i2c_handle_type i2c_handle_t;
//
// IRQ register / unregister helper
//
inline void i2c_irq_register(i2c_interrupt_config_t &irq, const char *name, uint32_t priority = DDL_IRQ_PRIO_05)
{
    // get auto-assigned irqn and set in irq struct
    IRQn_Type irqn;
    irqn_aa_get(irqn, name);
    irq.interrupt_number = irqn;

    // create irq registration struct
    stc_irq_signin_config_t irqConf = {
        .enIntSrc    = irq.interrupt_source,
        .enIRQn      = irq.interrupt_number,
        .pfnCallback = irq.interrupt_handler,
    };

    // register and enable irq
    INTC_IrqSignIn(&irqConf);
    NVIC_SetPriority(irqConf.enIRQn, priority);
    NVIC_ClearPendingIRQ(irqConf.enIRQn);
    NVIC_EnableIRQ(irqConf.enIRQn);
}

inline void i2c_irq_resign(i2c_interrupt_config_t &irq, const char *name)
{
    // disable interrupt and clear pending
    NVIC_DisableIRQ(irq.interrupt_number);
    NVIC_ClearPendingIRQ(irq.interrupt_number);
    INTC_IrqSignOut(irq.interrupt_number);

    // resign auto-assigned irqn
    irqn_aa_resign(irq.interrupt_number, name);
}

i2c_status_type i2c_wait_flag(i2c_handle_type *hi2c, uint32_t flag, uint32_t timeout)
{
    if (flag == I2C_FLAG_BUSY) {
        while (I2C_GetStatus(I2C_UNIT, flag) != RESET) {
            /* check timeout */
            if ((timeout--) == 0) {
                hi2c->error_code = I2C_ERR_TIMEOUT;

                return I2C_ERR_TIMEOUT;
            }
        }
    }
    return I2C_OK;
}

i2c_status_type i2c_wait_end(i2c_handle_type *hi2c, uint32_t timeout)
{
    while (hi2c->status != I2C_END) {
        /* check timeout */
        if ((timeout--) == 0) {
            return I2C_ERR_TIMEOUT;
        }
    }

    if (hi2c->error_code != I2C_OK) {
        return hi2c->error_code;
    } else {
        return I2C_OK;
    }
}

i2c_status_type i2c_slave_receive_int(i2c_handle_type *hi2c, uint32_t timeout)
{
    /* initialization parameters */
    hi2c->mode   = I2C_INT_SLA_RX;
    hi2c->status = I2C_START;

    hi2c->timeout    = timeout;
    hi2c->error_code = I2C_OK;

    /* wait for the busy flag to be reset */
    if (i2c_wait_flag(hi2c, I2C_FLAG_BUSY, timeout) != I2C_OK) {
        return I2C_ERR_STEP_1;
    }
    I2C_Cmd(I2C_UNIT, ENABLE);
    /* Config slave address match and receive full interrupt function*/
    I2C_IntCmd(I2C_UNIT, I2C_INT_MATCH_ADDR0 | I2C_INT_RX_FULL, ENABLE);
    return I2C_OK;
}

i2c_status_type i2c_slave_transmit_int(i2c_handle_type *hi2c, uint8_t *buff, uint32_t size, uint32_t timeout)
{
    /* initialization parameters */
    hi2c->mode   = I2C_INT_SLA_TX;
    hi2c->status = I2C_START;

    hi2c->tx_buffer = buff;
    hi2c->tx_size   = size;

    hi2c->timeout    = timeout;
    hi2c->error_code = I2C_OK;

    /* wait for the busy flag to be reset */
    if (i2c_wait_flag(hi2c, I2C_FLAG_BUSY, timeout) != I2C_OK) {
        return I2C_ERR_STEP_1;
    }
    I2C_Cmd(I2C_UNIT, ENABLE);
    /* Config slave address match interrupt function*/
    I2C_IntCmd(I2C_UNIT, I2C_INT_MATCH_ADDR0, ENABLE);
    return I2C_OK;
}

void i2c_evt_irq_handler(i2c_handle_type *hi2c)
{
    if (SET == I2C_GetStatus(I2C_UNIT, I2C_FLAG_MATCH_ADDR0)) {
        I2C_ClearStatus(I2C_UNIT, I2C_CLR_SLADDR0FCLR | I2C_CLR_NACKFCLR | I2C_CLR_STOPFCLR);
        if (RESET == I2C_GetStatus(I2C_UNIT, I2C_FLAG_TRA) && hi2c->mode == I2C_INT_SLA_RX) {
            hi2c->status = I2C_BUSY;
            /* Enable stop and NACK interrupt */
            I2C_IntCmd(I2C_UNIT, I2C_INT_STOP | I2C_INT_NACK, ENABLE);
        } else if (SET == I2C_GetStatus(I2C_UNIT, I2C_FLAG_TRA) && hi2c->mode == I2C_INT_SLA_TX) {
            /* Enable tx end interrupt function*/
            I2C_IntCmd(I2C_UNIT, I2C_INT_TX_CPLT, ENABLE);
            /* Write the first data to DTR immediately */
            if (hi2c->tx_size > 0) {
                I2C_WriteData(I2C_UNIT, *hi2c->tx_buffer++);
                hi2c->tx_size--;
            }
            // uint8_t ch;
            // if(lwrb_read(&tx_rb_t, &ch, 1) == 1){
            //     I2C_WriteData(I2C_UNIT, ch);
            // }

            /* Enable stop and NACK interrupt */
            I2C_IntCmd(I2C_UNIT, I2C_INT_STOP | I2C_INT_NACK, ENABLE);
        } else {
        }
    } else if (SET == I2C_GetStatus(I2C_UNIT, I2C_FLAG_NACKF)) {
        /* Clear STOPF flag */
        I2C_ClearStatus(I2C_UNIT, I2C_CLR_NACKFCLR);
        /* Config rx buffer full interrupt function disable */
        if (RESET == I2C_GetStatus(I2C_UNIT, I2C_FLAG_TRA)) {
            I2C_IntCmd(I2C_UNIT, I2C_INT_RX_FULL, DISABLE);
        } else {
            I2C_ClearStatus(I2C_UNIT, I2C_CLR_TENDFCLR);
            /* Config tx end interrupt function disable*/
            I2C_IntCmd(I2C_UNIT, I2C_INT_TX_CPLT, DISABLE);
            /* Read DRR register to release */
            (void)I2C_ReadData(I2C_UNIT);
        }
    } else if (SET == I2C_GetStatus(I2C_UNIT, I2C_FLAG_STOP)) {
        /* If stop interrupt occurred */
        /* Clear STOPF flag */
        I2C_ClearStatus(I2C_UNIT, I2C_CLR_STOPFCLR);
        /* Disable all interrupt enable flag except SLADDR0IE*/
        I2C_IntCmd(I2C_UNIT, I2C_INT_RX_FULL | I2C_INT_STOP | I2C_INT_NACK, DISABLE);
        I2C_Cmd(I2C_UNIT, DISABLE);
        hi2c->status = I2C_END; // Communication finished
    }
}
/**
 * @brief   I2C EEI(communication error or event) interrupt callback function
 * @param   None
 * @retval  None
 */
static void I2C_communication_error_event_callback(void)
{
    i2c_evt_irq_handler(&i2c_handle_t);
}

void i2c_slave_tx_end_callback(i2c_handle_type *hi2c)
{
    if ((SET == I2C_GetStatus(I2C_UNIT, I2C_FLAG_TX_CPLT)) &&
        (RESET == I2C_GetStatus(I2C_UNIT, I2C_FLAG_NACKF))) {
        if (hi2c->tx_size > 0) {
            I2C_WriteData(I2C_UNIT, *hi2c->tx_buffer++);
            hi2c->tx_size--;
        }
        // uint8_t ch;
        // if(lwrb_read(&tx_rb_t, &ch, 1) == 1){
        //     I2C_WriteData(I2C_UNIT, ch);
        // }
    }
}
/**
 * @brief   I2C TEI(transfer end) interrupt callback function
 * @param   None
 * @retval  None
 */
static void I2C_transfer_end_callback(void)
{
    i2c_slave_tx_end_callback(&i2c_handle_t);
}

/**
 * @brief   I2C RXI(receive buffer full) interrupt callback function
 * @param   None
 * @retval  None
 */
static void I2C_receive_buffer_full_Callback(void)
{
    if (SET == I2C_GetStatus(I2C_UNIT, I2C_FLAG_RX_FULL)) {
        uint8_t ch = I2C_ReadData(I2C_UNIT);
        lwrb_write(&rx_rb_t, &ch, 1);
    }
}

i2c_interrupt_config_t event_error_irq = {
    .interrupt_source  = INT_SRC_I2C1_EEI,
    .interrupt_handler = &I2C_communication_error_event_callback,
};

i2c_interrupt_config_t transfer_end_irq = {
    .interrupt_source  = INT_SRC_I2C1_TEI,
    .interrupt_handler = &I2C_transfer_end_callback,
};

i2c_interrupt_config_t receive_buffer_full_irq = {
    .interrupt_source  = INT_SRC_I2C1_RXI,
    .interrupt_handler = &I2C_receive_buffer_full_Callback,
};

void i2c_buffer_init()
{
    /* Allocate memory for the receive and transmit buffers */
    rxbuff = (uint8_t *)lwmem_malloc(I2C_SLAVE_RX_BUFFER_SIZE);
    // txbuff = (uint8_t *)lwmem_malloc(I2C_SLAVE_TX_BUFFER_SIZE);
    CORE_ASSERT(rxbuff != nullptr, "Failed to allocate memory for receive buffer");
    // CORE_ASSERT(txbuff != nullptr, "Failed to allocate memory for transmit buffer");
    /* Init buffer */
    lwrb_init(&rx_rb_t, rxbuff, I2C_SLAVE_RX_BUFFER_SIZE);
    lwrb_reset(&rx_rb_t);
    // lwrb_init(&tx_rb_t, txbuff, I2C_SLAVE_TX_BUFFER_SIZE);
    // lwrb_reset(&tx_rb_t);
}

size_t i2c_getcount_rxbuffer()
{
    return lwrb_get_full(&rx_rb_t);
}
size_t i2c_read_rxbuffer(uint8_t *data, uint32_t size)
{
    if (data == nullptr || size == 0) {
        return 0; // Invalid parameters
    }

    size_t read_len = lwrb_read(&rx_rb_t, data, size);
    return read_len;
}

// size_t i2c_write_txbuffer(const uint8_t *data, uint32_t size)
// {
//     if (data == nullptr || size == 0) {
//         return 0; // Invalid parameters
//     }
//     size_t write_len = lwrb_write(&tx_rb_t, data, size);
//     return write_len;
// }
#endif
bool i2c_slave_get_addrMatched()
{
    I2C_Cmd(I2C_UNIT, ENABLE);
    /* Clear status */
    I2C_ClearStatus(I2C_UNIT, I2C_CLR_STOPFCLR | I2C_CLR_NACKFCLR);
    /* Wait slave address matched */
    if (SET == I2C_GetStatus(I2C_UNIT, I2C_FLAG_MATCH_ADDR0)) {
        return true;
    }
    return false;
}

int32_t I2C_Slave_Receive(uint8_t *au8Data, uint32_t u32Size, uint32_t u32Timeout)
{
    int32_t i32Ret;
    I2C_ClearStatus(I2C_UNIT, I2C_CLR_SLADDR0FCLR);

    if (RESET == I2C_GetStatus(I2C_UNIT, I2C_FLAG_TRA)) {
        /* Slave receive data*/
        i32Ret = I2C_ReceiveData(I2C_UNIT, au8Data, u32Size, u32Timeout);

        if ((LL_OK == i32Ret) || (LL_ERR_TIMEOUT == i32Ret)) {
            /* Wait stop condition */
            i32Ret = I2C_WaitStatus(I2C_UNIT, I2C_FLAG_STOP, SET, u32Timeout);
        }
    } else {
        i32Ret = LL_ERR;
    }

    I2C_Cmd(I2C_UNIT, DISABLE);
    CORE_DEBUG_PRINTF("I2C_Slave_Receive return %d\n", i32Ret);
    return i32Ret;
}

int32_t I2C_Slave_Transmit(uint8_t *au8Data, uint32_t u32Size, uint32_t u32Timeout)
{
    int32_t i32Ret;
    I2C_ClearStatus(I2C_UNIT, I2C_CLR_SLADDR0FCLR);
    if (RESET == I2C_GetStatus(I2C_UNIT, I2C_FLAG_TRA)) {
        i32Ret = LL_ERR;
    } else {
        i32Ret = I2C_TransData(I2C_UNIT, au8Data, u32Size, u32Timeout);

        if ((LL_OK == i32Ret) || (LL_ERR_TIMEOUT == i32Ret)) {
            /* Release SCL pin */
            (void)I2C_ReadData(I2C_UNIT);

            /* Wait stop condition */
            i32Ret = I2C_WaitStatus(I2C_UNIT, I2C_FLAG_STOP, SET, u32Timeout);
        }
    }

    I2C_Cmd(I2C_UNIT, DISABLE);
    return i32Ret;
}
/**
 * @brief   Initialize the I2C peripheral for slave
 * @param   None
 * @retval int32_t:
 *            - LL_OK:              Success
 *            - LL_ERR_INVD_PARAM:  Invalid parameter
 */
int32_t i2cSlaveConfig_Initialize()
{
    int32_t i32Ret;
    stc_i2c_init_t stcI2cInit;
    stc_irq_signin_config_t stcIrqRegCfg;
    float32_t fErr;
#if defined(I2C_SLAVE_USE_IRQN) && (I2C_SLAVE_USE_IRQN != 0)
    i2c_buffer_init();
#endif
    (void)I2C_DeInit(I2C_UNIT);

    (void)I2C_StructInit(&stcI2cInit);
    stcI2cInit.u32ClockDiv = I2C_CLK_DIV2;
    stcI2cInit.u32Baudrate = 400 * 1000U; // 400kHz
    stcI2cInit.u32SclTime  = 5U;
    i32Ret                 = I2C_Init(I2C_UNIT, &stcI2cInit, &fErr);

    if (LL_OK == i32Ret) {
        /* Set slave address*/
        I2C_SlaveAddrConfig(I2C_UNIT, I2C_ADDR0, I2C_ADDR_7BIT, I2C_SLAVE_ADDRESS);

#if defined(I2C_SLAVE_USE_IRQN) && (I2C_SLAVE_USE_IRQN != 0)
        /* Enable I2C peripheral */
        i2c_irq_register(event_error_irq, "i2c1_event_error_irq", DDL_IRQ_PRIO_05);

        i2c_irq_register(transfer_end_irq, "i2c1_transfer_end_irq", DDL_IRQ_PRIO_08);

        i2c_irq_register(receive_buffer_full_irq, "i2c1_receive_buffer_full_irq", DDL_IRQ_PRIO_08);
        I2C_Cmd(I2C_UNIT, ENABLE);
#endif

        I2C_DEBUG_PRINTF("I2C slave initialized with address 0x%02X, %s\n", I2C_SLAVE_ADDRESS, I2C_SLAVE_USE_IRQN ? "interrupt mode" : "polling mode");

        return LL_OK;
    } else {
        I2C_DEBUG_PRINTF("error :%f and return %d\n", fErr, i32Ret);
    }

    I2C_DEBUG_PRINTF("Failed to initialize I2C slave, error code: %d\n", i32Ret);
    return i32Ret;
}

void i2cSlaveConfig_Deinitialize()
{
    /* Disable I2C peripheral */
    I2C_Cmd(I2C_UNIT, DISABLE);

    I2C_DeInit(I2C_UNIT);
#if defined(I2C_SLAVE_USE_IRQN) && (I2C_SLAVE_USE_IRQN != 0)
    /* Unregister IRQ handlers */
    i2c_irq_resign(event_error_irq, "I2C1_EEI_IRQ");
    i2c_irq_resign(transfer_end_irq, "I2C1_TEI_IRQ");
    i2c_irq_resign(receive_buffer_full_irq, "I2C1_RXI_IRQ");

    /* Free allocated buffers */
    lwmem_free(rxbuff);
    lwrb_free(&rx_rb_t);
    lwmem_free(txbuff);
    lwrb_free(&tx_rb_t);
#endif
}

int32_t i2cSlave_init()
{
    /* Enable I2C Peripheral*/
    FCG_Fcg1PeriphClockCmd(I2C_FCG_USE, ENABLE);
    /* Initialize I2C port*/
    GPIO_SetFunction(I2C_SLAVE_SCL_PIN, I2C_GPIO_SCL_FUNC);
    GPIO_SetFunction(I2C_SLAVE_SDA_PIN, I2C_GPIO_SDA_FUNC);
    /* Initialize I2C peripheral */
    int32_t i32Ret = i2cSlaveConfig_Initialize();
    CORE_ASSERT(i32Ret == LL_OK, "Failed to initialize I2C slave");
    return LL_OK;
}