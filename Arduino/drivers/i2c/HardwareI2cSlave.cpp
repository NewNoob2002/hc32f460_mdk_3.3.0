#include <Arduino.h>
#include <lwmem/lwmem.h>

#include "HardwareI2cSlave.h"

uint8_t *rxbuff;                     /*!< pointer to i2c transfer buffer  */
uint8_t *txbuff;                     /*!< pointer to i2c transfer buffer  */
volatile uint16_t pcount;            /*!< i2c transfer counter            */
volatile uint32_t mode;              /*!< i2c communication mode          */
volatile uint32_t timeout;           /*!< i2c wait time                   */
volatile uint32_t status;            /*!< i2c communication status        */
volatile i2c_status_type error_code; /*!< i2c error code                  */