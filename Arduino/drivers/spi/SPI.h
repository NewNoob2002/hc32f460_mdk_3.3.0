#pragma once
#include "drivers/sysclock/sysclock.h"
#include "spi_config.h"
#include <core_types.h>
// SPI_HAS_TRANSACTION means SPI has
//   - beginTransaction()
//   - endTransaction()
//   - usingInterrupt()
//   - SPISetting(clock, bitOrder, dataMode)
#define SPI_HAS_TRANSACTION 1

#define SPI_MODE0           0x02
#define SPI_MODE1           0x00
#define SPI_MODE2           0x03
#define SPI_MODE3           0x01

class SPIClass
{
public:
    SPIClass(spi_config_t *config)
    {
        this->config = config;
    }
    inline void set_mosi_pin(const gpio_pin_t pin)
    {
        this->mosi_pin = pin;
    }
    inline void set_miso_pin(const gpio_pin_t pin)
    {
        this->miso_pin = pin;
    }
    inline void set_clock_pin(const gpio_pin_t pin)
    {
        this->clock_pin = pin;
    }
    inline void set_pins(const gpio_pin_t mosi, const gpio_pin_t miso, const gpio_pin_t clock)
    {
        this->set_mosi_pin(mosi);
        this->set_miso_pin(miso);
        this->set_clock_pin(clock);
    }

    void calculate_FrequencyToDivider(const uint32_t frequency, uint16_t &cal_divider)
    {
        uint16_t divider = SYSTEM_CLOCK_FREQUENCIES.pclk0 / frequency;
        if (divider <= 2) {
            divider = SPI_BR_CLK_DIV2; // Minimum divider value
        } else if (divider > 2 && divider <= 4) {
            divider = SPI_BR_CLK_DIV4; // Adjust to next valid divider
        } else if (divider > 4 && divider <= 8) {
            divider = SPI_BR_CLK_DIV8; // Adjust to next valid divider
        } else if (divider > 8 && divider <= 16) {
            divider = SPI_BR_CLK_DIV16; // Adjust to next valid divider
        } else if (divider > 16 && divider <= 32) {
            divider = SPI_BR_CLK_DIV32; // Adjust to next valid divider
        } else if (divider > 32 && divider <= 64) {
            divider = SPI_BR_CLK_DIV64; // Adjust to next valid divider
        } else if (divider > 64 && divider <= 128) {
            divider = SPI_BR_CLK_DIV128; // Adjust to next valid divider
        } else if (divider > 128 && divider <= 256) {
            divider = SPI_BR_CLK_DIV256; // Adjust to next valid divider
        }
    }

    inline void setClockFrequency(const uint32_t frequency)
    {
        this->frequency = frequency;
        calculate_FrequencyToDivider(frequency, this->divider);
    }

    inline uint32_t getClockFrequency() const
    {
        return this->frequency;
    }
    inline uint16_t getClockDivider() const
    {
        return this->divider;
    }

    void begin(const uint32_t frequency = 12500000, const bool enable_DMA = false);

    void end();

    inline uint8_t transfer(const uint8_t data)
    {
        send(static_cast<uint32_t>(data));
        return static_cast<uint8_t>(receive());
    }

    inline uint16_t transfer16(const uint16_t data)
    {
        send(static_cast<uint32_t>(data));
        return static_cast<uint16_t>(receive());
    }

    inline uint32_t transfer32(const uint32_t data)
    {
        send(data);
        return receive();
    }

    inline void transfer(uint8_t *buffer, const size_t count)
    {
        for (size_t i = 0; i < count; i++) {
            *buffer = transfer(*buffer);
            buffer++;
        }
    }
    void beginTransaction();

    void endTransaction();

private:
    spi_config_t *config = NULL; // Pointer to SPI configuration
    gpio_pin_t mosi_pin;
    gpio_pin_t miso_pin;
    gpio_pin_t clock_pin;

    uint32_t frequency;
    uint16_t divider;

    bool enableDMA = false;

    /**
     * @brief synchronously send data_len bits of data
     * @param data_len number of bits to send. reconfigures the SPI peripheral to this data length before sending.
     * @param data data to send
     */
    void send(const uint32_t data);

    /**
     * @brief synchronously receive data
     * @return received data
     * @note data lenght must be set before to-be received data is sent by the other side, e.g. by using send()
     */
    uint32_t receive();
};
extern SPIClass SPI1;