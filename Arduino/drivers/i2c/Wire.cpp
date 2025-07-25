#include <Arduino.h>
#include <lwmem/lwmem.h>
#include <Wire.h>

i2c_peripheral_config_t I2C1_config = {
    .register_base    = CM_I2C1,
    .clock_id         = PWC_FCG1_I2C1,
    .scl_pin_function = Func_I2c1_Scl,
    .sda_pin_function = Func_I2c1_Sda,
};

i2c_peripheral_config_t I2C2_config = {
    .register_base    = CM_I2C2,
    .clock_id         = PWC_FCG1_I2C2,
    .scl_pin_function = Func_I2c2_Scl,
    .sda_pin_function = Func_I2c2_Sda,
};

TwoWire Wire(&I2C2_config, PA9, PA8);
TwoWire Wire_Slave(&I2C1_config, PA3, PA2);

#define REG_TO_I2Cx(reg) ((reg == CM_I2C1) ? "I2C1" : (reg == CM_I2C2) ? "I2C2" \
                                                  : (reg == CM_I2C3)   ? "I2C3" \
                                                                       : "Unknown")

#define WIRE_DEBUG_PRINTF(fmt, ...) CORE_DEBUG_PRINTF("[%s] " fmt, REG_TO_I2Cx(this->_config->register_base), ##__VA_ARGS__)

TwoWire::TwoWire(i2c_peripheral_config_t *config, gpio_pin_t scl_pin, gpio_pin_t sda_pin)
{
    this->_config  = config;
    this->_scl_pin = scl_pin;
    this->_sda_pin = sda_pin;
#ifdef _WIRE_USE_RINGBUFFER
    this->rxbuff = (uint8_t *)lwmem_malloc(WIRE_BUFF_SIZE);
    this->txbuff = (uint8_t *)lwmem_malloc(WIRE_BUFF_SIZE);

    lwrb_init(&this->rx_rb_t, this->rxbuff, WIRE_BUFF_SIZE);
    lwrb_init(&this->tx_rb_t, this->txbuff, WIRE_BUFF_SIZE);
#endif
}

void TwoWire::begin(uint32_t clockFreq)
{
    this->_clock_frequency = clockFreq;

    if (this->enableSlave) {
        CORE_ASSERT(this->slave_address != 0 && this->slave_address < 0x7F, "Slave address must be set and less than 0x7F");
        this->rxbuff = (uint8_t *)lwmem_malloc(WIRE_RX_BUFF_SIZE);

        lwrb_init(&this->rx_rb_t, this->rxbuff, WIRE_RX_BUFF_SIZE);
    }

    GPIO_SetFunction(this->_scl_pin, this->_config->scl_pin_function);
    GPIO_SetFunction(this->_sda_pin, this->_config->sda_pin_function);

    FCG_Fcg1PeriphClockCmd(this->_config->clock_id, ENABLE);

    int32_t i32Ret;
    stc_i2c_init_t stcI2cInit;
    float32_t fErr;

    (void)I2C_DeInit(this->_config->register_base);
    (void)I2C_StructInit(&stcI2cInit);
    if (this->_clock_frequency <= 100 * 1000) {
        stcI2cInit.u32ClockDiv = I2C_CLK_DIV8;
        stcI2cInit.u32Baudrate = this->_clock_frequency;
        stcI2cInit.u32SclTime  = 3UL;
    } else if (this->_clock_frequency == 400 * 1000) {
        stcI2cInit.u32ClockDiv = I2C_CLK_DIV2;
        stcI2cInit.u32Baudrate = this->_clock_frequency;
        stcI2cInit.u32SclTime  = 5UL;
    }
    i32Ret = I2C_Init(this->_config->register_base, &stcI2cInit, &fErr);

    if (i32Ret != LL_OK) {
        WIRE_DEBUG_PRINTF("Failed to initialize I2C, error:%f, ret = %d", fErr, i32Ret);
        return;
    }

    if (this->enableSlave) {
        I2C_SlaveAddrConfig(this->_config->register_base, I2C_ADDR0, I2C_ADDR_7BIT, this->slave_address);
        WIRE_DEBUG_PRINTF("I2c init success, in mode: slave, scl pin: %d, sda pin: %d, slave address: 0x%02x\n",
                          this->_scl_pin, this->_sda_pin, this->slave_address);
    } else {
        I2C_BusWaitCmd(this->_config->register_base, ENABLE);
        WIRE_DEBUG_PRINTF("I2c init success, in mode: master, scl pin: %d, sda pin: %d\n", this->_scl_pin, this->_sda_pin);
    }
    this->isInitliased = true;
}

void TwoWire::end()
{
    I2C_DeInit(this->_config->register_base);
}

bool TwoWire::beginTransmission(uint8_t address)
{
    if (this->enableSlave) {
        WIRE_DEBUG_PRINTF("Slave mode not supported BeginTransmission\n");
        return false;
    }
    uint32_t i32Ret = LL_ERR;
    bool result     = false;

    I2C_Cmd(this->_config->register_base, ENABLE);

    I2C_SWResetCmd(this->_config->register_base, ENABLE);
    I2C_SWResetCmd(this->_config->register_base, DISABLE);
    if (I2C_Start(this->_config->register_base, WIRE_TIMEOUT) == LL_OK) {
        if (LL_OK == I2C_TransAddr(this->_config->register_base, address, I2C_DIR_TX, WIRE_TIMEOUT)) {
            result = true;
        }
    }
    return result;
}

uint8_t TwoWire::endTransmission(bool stopBit)
{
    if (this->enableSlave) {
        WIRE_DEBUG_PRINTF("Slave mode not supported endTransmission\n");
        return 0;
    }
    if (stopBit) {
        // Stop by software
        I2C_Stop(this->_config->register_base, WIRE_TIMEOUT);
        // Disable I2C
        I2C_Cmd(this->_config->register_base, DISABLE);
        return 1;
    }

    return 0;
}

bool TwoWire::slave_address_match()
{
    I2C_Cmd(this->_config->register_base, ENABLE);
    /* Clear status */
    I2C_ClearStatus(this->_config->register_base, I2C_CLR_STOPFCLR | I2C_CLR_NACKFCLR);
    /* Wait slave address matched */
    if (SET == I2C_GetStatus(this->_config->register_base, I2C_FLAG_MATCH_ADDR0)) {
        return true;
    }
    return false;
}

size_t TwoWire::slave_receive(uint32_t timeout)
{
    int32_t i32Ret;
    uint16_t rx_data_count = 0;
    // Clear address match flag
    I2C_ClearStatus(this->_config->register_base, I2C_CLR_SLADDR0FCLR);
    // get receive data flag
    if (RESET == I2C_GetStatus(this->_config->register_base, I2C_FLAG_TRA)) {
        while (I2C_GetStatus(this->_config->register_base, I2C_FLAG_STOP) == RESET) {
            i32Ret = I2C_WaitStatus(this->_config->register_base, I2C_FLAG_RX_FULL, SET, timeout);
            I2C_AckConfig(this->_config->register_base, I2C_ACK);
            if (i32Ret == LL_OK) {
                /* read data from register */
                uint8_t ch = I2C_ReadData(this->_config->register_base);
                rx_data_count += lwrb_write(&this->rx_rb_t, &ch, 1);
            } 
            // else {
            //     I2C_AckConfig(this->_config->register_base, I2C_ACK);
            //     break;
            // }
        };
        I2C_Cmd(this->_config->register_base, DISABLE);

    }
    return rx_data_count;
}

size_t TwoWire::slave_transmit(uint8_t *tx_buffer, uint32_t timeout)
{
    int32_t i32Ret = LL_ERR;
    uint16_t tx_data_count = 0;
    I2C_ClearStatus(this->_config->register_base, I2C_CLR_SLADDR0FCLR);
    if(SET == I2C_GetStatus(this->_config->register_base, I2C_FLAG_TRA)){ 
        while(I2C_GetStatus(this->_config->register_base, I2C_FLAG_STOP) == RESET){
            i32Ret = I2C_WaitStatus(this->_config->register_base, I2C_FLAG_TX_EMPTY, SET, timeout);
            if (i32Ret == LL_OK) {
                /* write data to register */
                uint8_t ch = *tx_buffer++;
                I2C_WriteData(this->_config->register_base, ch);
                if (I2C_GetStatus(this->_config->register_base, I2C_FLAG_NACKF) == SET) {
                    break;
                }
                tx_data_count++;
            }
        }
    }
    else
    {
        return 0;
    }
    return tx_data_count;
}

size_t TwoWire::slave_communicate(uint8_t *tx_buffer, uint32_t timeout)
{
    int32_t i32Ret         = LL_ERR;
    uint16_t tx_data_count = 0;
    uint16_t rx_data_count = 0;
    I2C_ClearStatus(this->_config->register_base, I2C_CLR_SLADDR0FCLR);
    if (SET == I2C_GetStatus(this->_config->register_base, I2C_FLAG_TRA)) {
        slave_workMode = SLAVE_MODE_TX;
        while (I2C_GetStatus(this->_config->register_base, I2C_FLAG_STOP) == RESET) {
            i32Ret = I2C_WaitStatus(this->_config->register_base, I2C_FLAG_TX_EMPTY, SET, timeout);
            if (i32Ret == LL_OK) {
                /* write data to register */
                uint8_t ch = *tx_buffer++;
                I2C_WriteData(this->_config->register_base, ch);
                if (I2C_GetStatus(this->_config->register_base, I2C_FLAG_NACKF) == SET) {
                    break;
                }
                tx_data_count++;
            }
        }
        I2C_Cmd(this->_config->register_base, DISABLE);
        return tx_data_count;
    } else {
        slave_workMode = SLAVE_MODE_RX;
        while (I2C_GetStatus(this->_config->register_base, I2C_FLAG_STOP) == RESET) {
            i32Ret = I2C_WaitStatus(this->_config->register_base, I2C_FLAG_RX_FULL, SET, timeout);
            I2C_AckConfig(this->_config->register_base, I2C_ACK);
            if (i32Ret == LL_OK) {
                /* read data from register */
                uint8_t ch = I2C_ReadData(this->_config->register_base);
                rx_data_count += lwrb_write(&this->rx_rb_t, &ch, 1);
            }
            // else {
            //     I2C_AckConfig(this->_config->register_base, I2C_ACK);
            //     break;
            // }
        };
        I2C_Cmd(this->_config->register_base, DISABLE);
        return rx_data_count;
    }
    return 0;
}
size_t TwoWire::available()
{
    return lwrb_get_full(&this->rx_rb_t);
}
size_t TwoWire::read(uint8_t *buffer, size_t quantity)
{
    return lwrb_read(&this->rx_rb_t, buffer, quantity);
}

size_t TwoWire::write(uint8_t data)
{
    if (I2C_TransData(this->_config->register_base, &data, 1, WIRE_TIMEOUT) == LL_OK) {
        return 1;
    }
    return 0;
}

size_t TwoWire::write(const uint8_t *data, size_t quantity)
{
    if (I2C_TransData(this->_config->register_base, data, quantity, WIRE_TIMEOUT) == LL_OK) {
        return quantity;
    }
    return 0;
}

size_t TwoWire::requestFrom(uint8_t address, uint8_t register_address, uint8_t *buffer, uint8_t quantity, bool sendStop)
{
    if (this->enableSlave) {
        WIRE_DEBUG_PRINTF("Slave mode not supported requestFrom\n");
        return 0;
    }
    int32_t ret = LL_ERR;
    if (write(register_address) == 0) {
        WIRE_DEBUG_PRINTF("I2c write register address failed\n");
        return 0;
    }
    ret = I2C_Restart(this->_config->register_base, WIRE_TIMEOUT);
    if (ret != LL_OK) {
        WIRE_DEBUG_PRINTF("I2c restart failed, ret = %d\n", ret);
        return 0;
    }
    ret = I2C_TransAddr(this->_config->register_base, address, I2C_DIR_RX, WIRE_TIMEOUT);
    if (ret != LL_OK) {
        WIRE_DEBUG_PRINTF("I2c restart trans addr failed, ret = %d\n", ret);
        return 0;
    }
    if (sendStop) {
        if (LL_OK == I2C_MasterReceiveDataAndStop(this->_config->register_base, buffer, quantity, WIRE_TIMEOUT)) {
            WIRE_DEBUG_PRINTF("I2c receive data success\n");
            return quantity;
        }
    }

    WIRE_DEBUG_PRINTF("I2c receive data failed\n");
    return 0;
}

bool TwoWire::isDeviceOnline(uint8_t address)
{
    if (this->enableSlave) {
        WIRE_DEBUG_PRINTF("Slave mode not supported Scan slave\n");
        return false;
    }
    if (beginTransmission(address)) {
        endTransmission();
    } else {
        endTransmission();
        return false;
    }
    return true;
}

void TwoWire::scanDeivces(voidFuncPtrWithArg callback)
{
    if (this->enableSlave) {
        WIRE_DEBUG_PRINTF("Slave mode not supported Scan slave\n");
        return;
    }
    for (uint8_t i = 0x01; i < 0x7F; i++) {
        if (isDeviceOnline(i)) {
            if (callback) {
                callback(&i);
            }
        }
    }
}
