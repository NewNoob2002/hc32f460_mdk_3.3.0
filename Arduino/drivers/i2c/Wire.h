#pragma once

#include <hc32_ll.h>
#include <core_types.h>

#include <lwrb/lwrb.h>

#define WIRE_TIMEOUT 0x40000UL

#ifndef WIRE_BUFF_SIZE
#define WIRE_BUFF_SIZE 128
#endif

/* I2C address mode */
#define I2C_ADDR_MD_7BIT                (1U)
#define I2C_ADDR_MD_10BIT               (0U)

#define I2C_ADDR_MD                     (I2C_ADDR_MD_7BIT)
/**
 * @brief I2C peripheral configuration
 */
struct i2c_peripheral_config_t
{
    /**
     * @brief I2C peripheral register base address
     */
    CM_I2C_TypeDef *register_base;

    /**
     * @brief I2C peripheral clock id
     * @note in FCG1
     */
    uint32_t clock_id;

    /**
     * @brief pin function for i2c scl pin
     */
    uint16_t scl_pin_function;

    /**
     * @brief pin function for i2c sda pin
     */
    uint16_t sda_pin_function;
};

class TwoWire{
public:
	TwoWire(i2c_peripheral_config_t *config, gpio_pin_t scl_pin, gpio_pin_t sda_pin);

	void begin(uint32_t clockFreq = 100 * 1000);
	void end();

	void setClock(uint32_t clockFreq){
        this->_clock_frequency = clockFreq;
    }
    void setSlaveAddress(uint8_t address)
    {
        this->enableSlave = true;
        this->slave_address = address;
    }

	bool beginTransmission(uint8_t address);
	uint8_t endTransmission(bool stopBit = true);

    bool slave_address_match();

	size_t write(uint8_t data);
	size_t write(const uint8_t * data, size_t quantity);
    size_t requestFrom(uint8_t address, uint8_t register_address, uint8_t *buffer, uint8_t quantity , bool sendStop = true);

    bool isDeviceOnline(uint8_t address);
    void scanDeivces(voidFuncPtrWithArg callback = nullptr);
private:
    i2c_peripheral_config_t *_config = nullptr;
    bool isInitliased = false;
    bool enableSlave = false;
    uint8_t slave_address = 0;

    gpio_pin_t _scl_pin;
    gpio_pin_t _sda_pin;

    uint32_t _clock_frequency;
    uint8_t *rxbuff = nullptr;
    uint8_t *txbuff = nullptr;

    lwrb_t rx_rb_t;
    lwrb_t tx_rb_t;
};

extern TwoWire Wire;