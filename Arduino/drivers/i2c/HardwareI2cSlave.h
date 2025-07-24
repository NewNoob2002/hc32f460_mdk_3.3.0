#ifndef _HardWAREI2C_H
#define _HardWAREI2C_H

#include <hc32_ll.h>

#define I2C_SLAVE_ADDRESS        0x11 /*!< I2C slave address */
#define I2C_UNIT                 (CM_I2C1)
#define I2C_FCG_USE              (FCG1_PERIPH_I2C1)

#define I2C_SLAVE_USE_IRQN         0

#define I2C_SLAVE_SCL_PIN        (PA3)
#define I2C_SLAVE_SDA_PIN        (PA4)
#define I2C_GPIO_SCL_FUNC        (GPIO_FUNC_49)
#define I2C_GPIO_SDA_FUNC        (GPIO_FUNC_48)

#define I2C_SLAVE_TX_BUFFER_SIZE 256 /*!< I2C slave buffer size */
#define I2C_SLAVE_RX_BUFFER_SIZE 512 /*!< I2C slave buffer size */

#define I2C_EVENT_CHECK_NONE             ((uint32_t)0x00000000)    /*!< check flag none */
#define I2C_EVENT_CHECK_ACKFAIL          ((uint32_t)0x00000001)    /*!< check flag ackfail */
#define I2C_EVENT_CHECK_STOP             ((uint32_t)0x00000002)    /*!< check flag stop */

typedef enum {
    I2C_OK = 0,        /*!< no error */
    I2C_ERR_STEP_1,
    I2C_ERR_LINE_BUSY,
    I2C_ERR_START,     /*!< start error */
    I2C_ERR_ADDR10,    /*!< addr10 error */
    I2C_ERR_ADDR,      /*!< addr error */
    I2C_ERR_STOP,      /*!< stop error */
    I2C_ERR_ACKFAIL,   /*!< ackfail error */
    I2C_ERR_TIMEOUT,   /*!< timeout error */
    I2C_ERR_INTERRUPT, /*!< interrupt error */
} i2c_status_type;

typedef enum
{
  I2C_INT_SLA_TX,
  I2C_INT_SLA_RX,
} i2c_mode_type;

typedef enum
{
    I2C_START = 0, /*!< I2C start condition */
    I2C_BUSY,
    I2C_END,   /*!< I2C end condition */
} i2c_communicate_status_type;

typedef struct
{
    uint8_t* tx_buffer;
    __IO uint32_t tx_size;
    __IO uint32_t mode;              /*!< i2c communication mode          */
    __IO uint32_t timeout;           /*!< i2c wait time                   */
    __IO uint32_t status;            /*!< i2c communication status        */
    __IO i2c_status_type error_code; /*!< i2c error code                  */
} i2c_handle_type;

struct i2c_interrupt_config_t {
    /**
     * @brief IRQn assigned to this interrupt handler
     * @note auto-assigned in Usart implementation
     */
    IRQn_Type interrupt_number;

    /**
     * @brief Interrupt source to set for this interrupt handler
     */
    en_int_src_t interrupt_source;

    /**
     * @brief Interrupt handler function pointer
     */
    func_ptr_t interrupt_handler;
};

int32_t i2cSlave_init();
#if defined (I2C_SLAVE_USE_IRQN) && (I2C_SLAVE_USE_IRQN != 0)
i2c_status_type i2c_wait_end(i2c_handle_type* hi2c, uint32_t timeout);
i2c_status_type i2c_slave_receive_int(i2c_handle_type *hi2c, uint32_t timeout);
i2c_status_type i2c_slave_transmit_int(i2c_handle_type *hi2c, uint8_t* buff, uint32_t size, uint32_t timeout);
size_t i2c_getcount_rxbuffer();
size_t i2c_read_rxbuffer(uint8_t *data, uint32_t size);
// size_t i2c_write_txbuffer(const uint8_t *data, uint32_t size);
#endif
bool i2c_slave_get_addrMatched();
int32_t I2C_Slave_Receive(uint8_t *au8Data, uint32_t u32Size, uint32_t u32Timeout);
int32_t I2C_Slave_Transmit(uint8_t *au8Data, uint32_t u32Size, uint32_t u32Timeout);
extern i2c_handle_type i2c_handle_t;
#endif
