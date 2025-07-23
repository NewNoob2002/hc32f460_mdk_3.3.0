#include <Arduino.h>
#include <lwmem/lwmem.h>
#include <Wire.h>

i2c_peripheral_config_t I2C1_config = {
    .register_base    = CM_I2C1,
    .clock_id         = PWC_FCG1_I2C1,
    .scl_pin_function = Func_I2c1_Scl,
    .sda_pin_function = Func_I2c1_Sda,
};

TwoWire Wire(&I2C1_config, PA3, PA4);

TwoWire::TwoWire(i2c_peripheral_config_t *config, gpio_pin_t scl_pin, gpio_pin_t sda_pin)
{
    this->_config  = config;
    this->_scl_pin = scl_pin;
    this->_sda_pin = sda_pin;

    this->isInitliased = false;
#ifdef USE_RINGBUFFER
    this->rxbuff = (uint8_t *)lwmem_malloc(WIRE_BUFF_SIZE);
    this->txbuff = (uint8_t *)lwmem_malloc(WIRE_BUFF_SIZE);

    lwrb_init(&this->rx_rb_t, this->rxbuff, WIRE_BUFF_SIZE);
    lwrb_init(&this->tx_rb_t, this->txbuff, WIRE_BUFF_SIZE);
#endif
}

void TwoWire::begin(uint32_t clockFreq)
{
    this->_clock_frequency = clockFreq;
    GPIO_SetFunction(this->_scl_pin, this->_config->scl_pin_function);
    GPIO_SetFunction(this->_sda_pin, this->_config->sda_pin_function);

    FCG_Fcg1PeriphClockCmd(this->_config->clock_id, ENABLE);

    int32_t i32Ret;
    stc_i2c_init_t stcI2cInit;
    float32_t fErr;

    (void)I2C_DeInit(this->_config->register_base);

    (void)I2C_StructInit(&stcI2cInit);
    stcI2cInit.u32ClockDiv = I2C_CLK_DIV8;
    stcI2cInit.u32Baudrate = this->_clock_frequency;
    stcI2cInit.u32SclTime  = 3UL;
    i32Ret                 = I2C_Init(this->_config->register_base, &stcI2cInit, &fErr);

    if (i32Ret != LL_OK) {
        CORE_DEBUG_PRINTF("Failed to initialize I2C, error:%f, ret = %d", fErr, i32Ret);
        return;
    }
    this->isInitliased = true;
	CORE_DEBUG_PRINTF("I2c init success\n");
    I2C_BusWaitCmd(this->_config->register_base, ENABLE);
}

void TwoWire::end()
{
    I2C_DeInit(this->_config->register_base);
}

void TwoWire::setClock(uint32_t clockFreq)
{
    this->_clock_frequency = clockFreq;
}

bool TwoWire::beginTransmission(uint8_t address)
{
    uint32_t i32Ret  = LL_ERR;
    bool result = false;

    I2C_Cmd(this->_config->register_base, ENABLE);

    I2C_SWResetCmd(this->_config->register_base, ENABLE);
    I2C_SWResetCmd(this->_config->register_base, DISABLE);
    if (I2C_Start(this->_config->register_base, 0x4000) == LL_OK) {
      if(LL_OK == I2C_TransAddr(this->_config->register_base, address, I2C_DIR_TX, 0x4000))
			{
				result = true;
			}
    }
    return result;
}

uint8_t TwoWire::endTransmission(bool stopBit)
{
    if (stopBit) {
        // Stop by software
        I2C_Stop(this->_config->register_base, 0x4000);
        // Disable I2C
        I2C_Cmd(this->_config->register_base, DISABLE);
        return 1;
    }

    return 0;
}

size_t TwoWire::write(uint8_t data)
{
    if (I2C_TransData(this->_config->register_base, &data, 1, 0x4000) == LL_OK) {
        return 1;
    }
    return 0;
}

size_t TwoWire::write(const uint8_t *data, size_t quantity)
{
    if (I2C_TransData(this->_config->register_base, data, quantity, 0x4000) == LL_OK) {
        return quantity;
    }
    return 0;
}

bool TwoWire::isDeviceOnline(uint8_t address)
{
		if(beginTransmission(address))
		{
			endTransmission();
		}
		else
		{
			return false;
		}
    return true;
}
