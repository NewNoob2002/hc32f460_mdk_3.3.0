#include <Arduino.h>
#include <drivers/gpio/gpio.h>
#include <drivers/irqn/irqn.h>
#include <lwmem/lwmem.h>
#include <lwrb/lwrb.h>

#include "HardwareI2cSlave.h"

#define ENABLE_I2C_DEBUG 1
#if defined(ENABLE_I2C_DEBUG) && (ENABLE_I2C_DEBUG == 1)
#define I2C_DEBUG_PRINTF(fmt, ...) printf("[I2C1]" fmt, ##__VA_ARGS__)
#else
#define I2C_DEBUG_PRINTF(fmt, ...)
#endif

static lwrb_t rx_rb_t; /*!< Receive ring buffer */
static lwrb_t tx_rb_t; /*!< Transmit ring buffer */
uint8_t *rxbuff = nullptr; /*!< pointer to i2c transfer buffer  */
uint8_t *txbuff = nullptr; /*!< pointer to i2c transfer buffer  */
static i2c_handle_type i2c_handle_t;
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

i2c_status_type i2c_slave_receive_int(i2c_handle_type* hi2c, uint8_t* pdata, uint16_t size, uint32_t timeout)
{
  /* initialization parameters */
  hi2c->mode   = I2C_INT_SLA_RX;
  hi2c->status = I2C_START;
  
  hi2c->pcount = size;

  hi2c->timeout = timeout;
  hi2c->error_code = I2C_OK;

  /* wait for the busy flag to be reset */
  if(i2c_wait_flag(hi2c, I2C_BUSYF_FLAG, I2C_EVENT_CHECK_NONE, timeout) != I2C_OK)
  {
    return I2C_ERR_STEP_1;
  }

  /* ack acts on the current byte */
  i2c_master_receive_ack_set(hi2c->i2cx, I2C_MASTER_ACK_CURRENT);

  /* enable ack */
  i2c_ack_enable(hi2c->i2cx, TRUE);

  /* enable interrupt */
  i2c_interrupt_enable(hi2c->i2cx, I2C_EVT_INT | I2C_DATA_INT | I2C_ERR_INT, TRUE);

  return I2C_OK;
}
/**
 * @brief   I2C EEI(communication error or event) interrupt callback function
 * @param   None
 * @retval  None
 */
static void I2C_communication_error_event_callback(void)
{
    /* If address interrupt occurred */
    if (SET == I2C_GetStatus(I2C_UNIT, I2C_FLAG_MATCH_ADDR0)) {
        I2C_ClearStatus(I2C_UNIT, I2C_CLR_SLADDR0FCLR | I2C_CLR_NACKFCLR | I2C_CLR_STOPFCLR);

        if ((MD_TX == stcI2cCom.enMode) && (SET == I2C_GetStatus(I2C_UNIT, I2C_FLAG_TRA))) {
            /* Enable tx end interrupt function*/
            I2C_IntCmd(I2C_UNIT, I2C_INT_TX_CPLT, ENABLE);
            /* Write the first data to DTR immediately */
            I2C_WriteData(I2C_UNIT, BufRead());

            /* Enable stop and NACK interrupt */
            I2C_IntCmd(I2C_UNIT, I2C_INT_STOP | I2C_INT_NACK, ENABLE);
        } else if ((MD_RX == stcI2cCom.enMode) && (RESET == I2C_GetStatus(I2C_UNIT, I2C_FLAG_TRA))) {
            /* Enable stop and NACK interrupt */
            I2C_IntCmd(I2C_UNIT, I2C_INT_STOP | I2C_INT_NACK, ENABLE);
        } else {
        }
    } else if (SET == I2C_GetStatus(I2C_UNIT, I2C_FLAG_NACKF)) {
        /* If NACK interrupt occurred */
        /* clear NACK flag*/
        I2C_ClearStatus(I2C_UNIT, I2C_CLR_NACKFCLR);
        /* Stop tx or rx process*/
        if (SET == I2C_GetStatus(I2C_UNIT, I2C_FLAG_TRA)) {
            /* Config tx end interrupt function disable*/
            I2C_IntCmd(I2C_UNIT, I2C_INT_TX_CPLT, DISABLE);
            I2C_ClearStatus(I2C_UNIT, I2C_CLR_TENDFCLR);

            /* Read DRR register to release */
            (void)I2C_ReadData(I2C_UNIT);
        } else {
            /* Config rx buffer full interrupt function disable */
            I2C_IntCmd(I2C_UNIT, I2C_INT_RX_FULL, DISABLE);
        }
    } else if (SET == I2C_GetStatus(I2C_UNIT, I2C_FLAG_STOP)) {
        /* If stop interrupt occurred */
        /* Disable all interrupt enable flag except SLADDR0IE*/
        I2C_IntCmd(I2C_UNIT, I2C_INT_TX_CPLT | I2C_INT_RX_FULL | I2C_INT_STOP | I2C_INT_NACK, DISABLE);
        /* Clear STOPF flag */
        I2C_ClearStatus(I2C_UNIT, I2C_CLR_STOPFCLR);
        I2C_Cmd(I2C_UNIT, DISABLE);
        /* Communication finished */
        stcI2cCom.enComStatus = I2C_COM_IDLE;
    } else {
    }
}

/**
 * @brief   I2C TEI(transfer end) interrupt callback function
 * @param   None
 * @retval  None
 */
static void I2C_transfer_end_callback(void)
{
    if ((SET == I2C_GetStatus(I2C_UNIT, I2C_FLAG_TX_CPLT)) &&
        (RESET == I2C_GetStatus(I2C_UNIT, I2C_FLAG_NACKF))) {
        // I2C_WriteData(I2C_UNIT, BufRead());
    }
}

/**
 * @brief   I2C RXI(receive buffer full) interrupt callback function
 * @param   None
 * @retval  None
 */
static void I2C_receive_buffer_full_Callback(void)
{
    if (SET == I2C_GetStatus(I2C_UNIT, I2C_FLAG_RX_FULL)) {
        // BufWrite(I2C_ReadData(I2C_UNIT));
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
    txbuff = (uint8_t *)lwmem_malloc(I2C_SLAVE_TX_BUFFER_SIZE);
    CORE_ASSERT(rxbuff != nullptr, "Failed to allocate memory for receive buffer");
    CORE_ASSERT(txbuff != nullptr, "Failed to allocate memory for transmit buffer");
    /* Init buffer */
    lwrb_init(&rx_rb_t, rxbuff, I2C_SLAVE_RX_BUFFER_SIZE);
    lwrb_init(&tx_rb_t, txbuff, I2C_SLAVE_TX_BUFFER_SIZE);
    lwrb_reset(&rx_rb_t);
    lwrb_reset(&tx_rb_t);
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

    FCG_Fcg1PeriphClockCmd(I2C_FCG_USE, ENABLE);
    i2c_buffer_init();
    (void)I2C_DeInit(I2C_UNIT);

    (void)I2C_StructInit(&stcI2cInit);
    stcI2cInit.u32ClockDiv = I2C_CLK_DIV2;
    stcI2cInit.u32Baudrate = 400 * 1000U; // 400kHz
    stcI2cInit.u32SclTime  = 5U;
    i32Ret                 = I2C_Init(I2C_UNIT, &stcI2cInit, &fErr);

    if (LL_OK == i32Ret) {
        /* Set slave address*/
        I2C_SlaveAddrConfig(I2C_UNIT, I2C_ADDR0, I2C_ADDR_7BIT, I2C_SLAVE_ADDRESS);
        /* Enable I2C peripheral */
        i2c_irq_register(event_error_irq, "I2C1_EEI_IRQ");

        i2c_irq_register(transfer_end_irq, "I2C1_TEI_IRQ");

        i2c_irq_register(receive_buffer_full_irq, "I2C1_RXI_IRQ");

        I2C_Cmd(I2C_UNIT, ENABLE);
        I2C_DEBUG_PRINTF("I2C slave initialized with address 0x%02X\n", I2C_SLAVE_ADDRESS);
        
        return LL_OK;
    }

    I2C_DEBUG_PRINTF("Failed to initialize I2C slave, error code: %d\n", i32Ret);
    return i32Ret;
}

void i2cSlaveConfig_Deinitialize()
{
    /* Disable I2C peripheral */
    I2C_Cmd(I2C_UNIT, DISABLE);

    I2C_DeInit(I2C_UNIT);
    /* Unregister IRQ handlers */
    i2c_irq_resign(event_error_irq, "I2C1_EEI_IRQ");
    i2c_irq_resign(transfer_end_irq, "I2C1_TEI_IRQ");
    i2c_irq_resign(receive_buffer_full_irq, "I2C1_RXI_IRQ");

    /* Free allocated buffers */
    lwmem_free(rxbuff);
    lwmem_free(txbuff);
    lwrb_free(&rx_rb_t);
    lwrb_free(&rx_rb_t);
}

int32_t i2cSlave_init()
{
    /* Enable I2C Peripheral*/
    FCG_Fcg1PeriphClockCmd(I2C_FCG_USE, ENABLE);
    i2c_buffer_init();
    /* Initialize I2C port*/
    GPIO_SetFunction(I2C_SLAVE_SCL_PIN, I2C_SLAVE_SDA_PIN);
    GPIO_SetFunction(I2C_SLAVE_SDA_PIN, I2C_GPIO_SDA_FUNC);
    /* Initialize I2C peripheral */
    int32_t i32Ret = i2cSlaveConfig_Initialize();
    CORE_ASSERT(i32Ret == LL_OK, "Failed to initialize I2C slave");
}