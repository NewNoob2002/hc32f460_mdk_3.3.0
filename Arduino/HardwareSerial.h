/*
  Copyright (c) 2015 Arduino LLC.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef HardwareSerial_h
#define HardwareSerial_h

#include "RingBuffer.h"
#include "Stream.h"
#include "core_types.h"
#include "Print.h"
#include <drivers/usart/usart_config.h>

#define HARDSER_PARITY_EVEN (0x1ul)
#define HARDSER_PARITY_ODD  (0x2ul)
#define HARDSER_PARITY_NONE (0x3ul)
#define HARDSER_PARITY_MASK (0xFul)

#define HARDSER_STOP_BIT_1  (0x10ul)
// #define HARDSER_STOP_BIT_1_5 (0x20ul)
#define HARDSER_STOP_BIT_2    (0x30ul)
#define HARDSER_STOP_BIT_MASK (0xF0ul)

// #define HARDSER_DATA_5 (0x100ul)
// #define HARDSER_DATA_6 (0x200ul)
// #define HARDSER_DATA_7 (0x300ul)
#define HARDSER_DATA_8    (0x400ul)
#define HARDSER_DATA_MASK (0xF00ul)

#define SERIAL_5N1        (HARDSER_STOP_BIT_1 | HARDSER_PARITY_NONE | HARDSER_DATA_5)
#define SERIAL_6N1        (HARDSER_STOP_BIT_1 | HARDSER_PARITY_NONE | HARDSER_DATA_6)
#define SERIAL_7N1        (HARDSER_STOP_BIT_1 | HARDSER_PARITY_NONE | HARDSER_DATA_7)
#define SERIAL_8N1        (HARDSER_STOP_BIT_1 | HARDSER_PARITY_NONE | HARDSER_DATA_8)
#define SERIAL_5N2        (HARDSER_STOP_BIT_2 | HARDSER_PARITY_NONE | HARDSER_DATA_5)
#define SERIAL_6N2        (HARDSER_STOP_BIT_2 | HARDSER_PARITY_NONE | HARDSER_DATA_6)
#define SERIAL_7N2        (HARDSER_STOP_BIT_2 | HARDSER_PARITY_NONE | HARDSER_DATA_7)
#define SERIAL_8N2        (HARDSER_STOP_BIT_2 | HARDSER_PARITY_NONE | HARDSER_DATA_8)
#define SERIAL_5E1        (HARDSER_STOP_BIT_1 | HARDSER_PARITY_EVEN | HARDSER_DATA_5)
#define SERIAL_6E1        (HARDSER_STOP_BIT_1 | HARDSER_PARITY_EVEN | HARDSER_DATA_6)
#define SERIAL_7E1        (HARDSER_STOP_BIT_1 | HARDSER_PARITY_EVEN | HARDSER_DATA_7)
#define SERIAL_8E1        (HARDSER_STOP_BIT_1 | HARDSER_PARITY_EVEN | HARDSER_DATA_8)
#define SERIAL_5E2        (HARDSER_STOP_BIT_2 | HARDSER_PARITY_EVEN | HARDSER_DATA_5)
#define SERIAL_6E2        (HARDSER_STOP_BIT_2 | HARDSER_PARITY_EVEN | HARDSER_DATA_6)
#define SERIAL_7E2        (HARDSER_STOP_BIT_2 | HARDSER_PARITY_EVEN | HARDSER_DATA_7)
#define SERIAL_8E2        (HARDSER_STOP_BIT_2 | HARDSER_PARITY_EVEN | HARDSER_DATA_8)
#define SERIAL_5O1        (HARDSER_STOP_BIT_1 | HARDSER_PARITY_ODD | HARDSER_DATA_5)
#define SERIAL_6O1        (HARDSER_STOP_BIT_1 | HARDSER_PARITY_ODD | HARDSER_DATA_6)
#define SERIAL_7O1        (HARDSER_STOP_BIT_1 | HARDSER_PARITY_ODD | HARDSER_DATA_7)
#define SERIAL_8O1        (HARDSER_STOP_BIT_1 | HARDSER_PARITY_ODD | HARDSER_DATA_8)
#define SERIAL_5O2        (HARDSER_STOP_BIT_2 | HARDSER_PARITY_ODD | HARDSER_DATA_5)
#define SERIAL_6O2        (HARDSER_STOP_BIT_2 | HARDSER_PARITY_ODD | HARDSER_DATA_6)
#define SERIAL_7O2        (HARDSER_STOP_BIT_2 | HARDSER_PARITY_ODD | HARDSER_DATA_7)
#define SERIAL_8O2        (HARDSER_STOP_BIT_2 | HARDSER_PARITY_ODD | HARDSER_DATA_8)

#define SERIAL_RX_BUFFER_SIZE 512
#define SERIAL_TX_BUFFER_SIZE 128

class HardwareSerial : public Stream
{
public:
    HardwareSerial(struct usart_config_t *usart_config,
                  gpio_pin_t tx_pin,
                  gpio_pin_t rx_pin);
    ~HardwareSerial();
    
    const usart_config_t *get_config() const { return this->usart_config; }
    const usart_receive_error_t get_rx_error() const { return this->usart_config->state.rx_error; }
    void set_tx_timeout(uint32_t timeout) { this->_tx_timeout = timeout; }
    void begin(uint32_t baud);
    void begin(uint32_t baud, uint16_t config);
    void begin(const stc_usart_uart_init_t *config, const bool rxNoiseFilter = true);
    void end();
    virtual int available(void);
    int availableForWrite(void);
    virtual int peek(void);
    virtual int read(void);
    virtual void flush(void);

    virtual size_t write(uint8_t n);
    inline size_t write(unsigned long n)
    {
        return write((uint8_t)n);
    }
    inline size_t write(long n)
    {
        return write((uint8_t)n);
    }
    inline size_t write(unsigned int n)
    {
        return write((uint8_t)n);
    }
    inline size_t write(int n)
    {
        return write((uint8_t)n);
    }
    using Print::write; // pull in write(str) and write(buf, size) from Print
    operator bool()
    {
        return true;
    }

    inline bool is_initialized() const { return _is_initialized; }
  private:
    usart_config_t *usart_config;

    gpio_pin_t tx_pin;
    gpio_pin_t rx_pin;

    RingBuffer<uint8_t> *_rx_buffer;
    RingBuffer<uint8_t> *_tx_buffer;

    uint8_t _rxBuffer[SERIAL_RX_BUFFER_SIZE];
    uint32_t _tx_timeout;
    bool _is_initialized;
};

#if SERIAL_1_ENABLE
extern HardwareSerial Serial;
#endif

#if SERIAL_2_ENABLE
extern HardwareSerial Serial2;
#endif

#if SERIAL_3_ENABLE
extern HardwareSerial Serial3;
#endif

#endif
