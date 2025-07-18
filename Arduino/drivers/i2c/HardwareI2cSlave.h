#ifndef _HardWAREI2C_H
#define _HardWAREI2C_H

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

#endif
