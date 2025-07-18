#ifndef _HardWAREI2C_H
#define _HardWAREI2C_H

#include <hc32_ll.h>

#define I2C_SLAVE_ADDRESS        0x11 /*!< I2C slave address */
#define I2C_UNIT                 (CM_I2C1)
#define I2C_FCG_USE              (FCG1_PERIPH_I2C1)

#define I2C_SLAVE_SCL_PIN        (PA3)
#define I2C_SLAVE_SDA_PIN        (PA2)
#define I2C_GPIO_SCL_FUNC        (GPIO_FUNC_49)
#define I2C_GPIO_SDA_FUNC        (GPIO_FUNC_48)

#define I2C_SLAVE_TX_BUFFER_SIZE 256 /*!< I2C slave buffer size */
#define I2C_SLAVE_RX_BUFFER_SIZE 512 /*!< I2C slave buffer size */

typedef enum {
    I2C_OK = 0,        /*!< no error */
    I2C_ERR_STEP_1,    /*!< step 1 error */
    I2C_ERR_STEP_2,    /*!< step 2 error */
    I2C_ERR_STEP_3,    /*!< step 3 error */
    I2C_ERR_STEP_4,    /*!< step 4 error */
    I2C_ERR_STEP_5,    /*!< step 5 error */
    I2C_ERR_STEP_6,    /*!< step 6 error */
    I2C_ERR_STEP_7,    /*!< step 7 error */
    I2C_ERR_STEP_8,    /*!< step 8 error */
    I2C_ERR_STEP_9,    /*!< step 9 error */
    I2C_ERR_STEP_10,   /*!< step 10 error */
    I2C_ERR_STEP_11,   /*!< step 11 error */
    I2C_ERR_STEP_12,   /*!< step 12 error */
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
    I2C_END,   /*!< I2C end condition */
} i2c_communicate_status_type;

typedef struct
{
    __IO uint16_t pcount;            /*!< i2c transfer counter            */
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

#endif
