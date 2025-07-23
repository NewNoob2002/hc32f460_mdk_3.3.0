#ifndef __SOFT_SLAVE_I2C_H__
#define __SOFT_SLAVE_I2C_H__

#include <stdint.h>
#include <stddef.h>

#define I2C_BUFFER_SIZE 32

class SoftSlaveI2C {
public:
    SoftSlaveI2C();

    void begin(uint8_t address, int sda_pin, int scl_pin);
    void onReceive(void (*)(int));
    void onRequest(void (*)(void));
    size_t write(uint8_t data);
    size_t write(const uint8_t *data, size_t quantity);
    int available(void);
    int read(void);

private:
    static void i2c_isr();
    void handle_state();

    static SoftSlaveI2C* instance;

    int _sda_pin;
    int _scl_pin;
    uint8_t _slave_address;

    enum I2CState {
        I2C_IDLE,
        I2C_ADDR,
        I2C_READ,
        I2C_WRITE,
        I2C_ACK
    };

    volatile I2CState _state;
    volatile uint8_t _bit_count;
    volatile uint8_t _current_byte;

    uint8_t _rx_buffer[I2C_BUFFER_SIZE];
    volatile uint8_t _rx_buffer_index;
    volatile uint8_t _rx_buffer_head;

    uint8_t _tx_buffer[I2C_BUFFER_SIZE];
    volatile uint8_t _tx_buffer_index;
    volatile uint8_t _tx_buffer_size;

};

extern SoftSlaveI2C SlaveI2C;

#endif // __SOFT_SLAVE_I2C_H__
