#include "soft_slave_i2c.h"
#include <Arduino.h>

SoftSlaveI2C* SoftSlaveI2C::instance = nullptr;

// Helper functions for pin operations to keep ISRs fast
static void sda_mode(gpio_pin_t pin, PinMode_TypeDef mode) {
    pinMode(pin, mode);
}

static void sda_write(gpio_pin_t pin, uint8_t val) {
    digitalWrite(pin, val);
}

static int sda_read(gpio_pin_t pin) {
    return digitalRead(pin);
}

SoftSlaveI2C::SoftSlaveI2C() {
    instance = this;
    _state = I2C_IDLE;
    _on_receive_callback = nullptr;
    _on_request_callback = nullptr;
}

void SoftSlaveI2C::begin(uint8_t address, int sda_pin, int scl_pin) {
    _slave_address = address;
    _sda_pin = sda_pin;
    _scl_pin = scl_pin;

    pinMode(_sda_pin, INPUT_PULLUP);
    pinMode(_scl_pin, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(_scl_pin), i2c_isr, CHANGE);
    attachInterrupt(digitalPinToInterrupt(_sda_pin), i2c_isr, CHANGE);
}

void SoftSlaveI2C::onReceive(void (*callback)(int)) {
    _on_receive_callback = callback;
}

void SoftSlaveI2C::onRequest(void (*callback)(void)) {
    _on_request_callback = callback;
}

void SoftSlaveI2C::i2c_isr() {
    instance->handle_state();
}

void SoftSlaveI2C::handle_state() {
    // Detect START condition: SDA falling edge while SCL is high
    if (_state == I2C_IDLE) {
        if (!digitalRead(_sda_pin) && digitalRead(_scl_pin)) {
            _state = I2C_ADDR;
            _bit_count = 0;
            _current_byte = 0;
            _rx_buffer_index = 0;
            _rx_buffer_head = 0;
            _tx_buffer_index = 0;
            _tx_buffer_size = 0;
            // Wait for SCL to go low to start clocking bits
            while(digitalRead(_scl_pin));
        }
        return;
    }

    // Detect STOP condition: SDA rising edge while SCL is high
    if (digitalRead(_sda_pin) && digitalRead(_scl_pin)) {
        if (_state == I2C_READ && _on_receive_callback) {
            _on_receive_callback(_rx_buffer_index);
        }
        _state = I2C_IDLE;
        return;
    }

    // We are in a transaction, handle SCL edges
    if (!digitalRead(_scl_pin)) { // SCL is low, slave can change SDA
        if (_state == I2C_ACK) {
            if (_current_byte & 0x01) { // Master is reading from us
                // Master ACK'd our data, get ready for next byte
                _state = I2C_WRITE;
                _bit_count = 0;
            } else { // Master is writing to us
                // We ACK'd master's data, get ready for next byte
                _state = I2C_READ;
            }
            sda_mode(_sda_pin, INPUT_PULLUP); // Release SDA for master
        } else if (_state == I2C_WRITE && _bit_count == 0) {
            // Prepare to send a byte
            sda_mode(_sda_pin, OUTPUT);
            if (_tx_buffer_index >= _tx_buffer_size) {
                // No more data, send 0xFF
                 _current_byte = 0xFF;
            } else {
                 _current_byte = _tx_buffer[_tx_buffer_index++];
            }
        }
        
        if (_state == I2C_WRITE) {
            // Clock out the bit
            digitalWrite(_sda_pin, (_current_byte >> (7 - _bit_count)) & 0x01);
        }

    } else { // SCL is high, slave must read SDA
        if (_bit_count < 8) {
            if (_state == I2C_READ || _state == I2C_ADDR) {
                 _current_byte = (_current_byte << 1) | digitalRead(_sda_pin);
            }
            _bit_count++;
        } else { // 8 bits have been clocked, time for ACK/NACK
            if (_state == I2C_ADDR) {
                if ((_current_byte >> 1) == _slave_address) {
                    _state = I2C_ACK;
                    sda_mode(_sda_pin, OUTPUT);
                    digitalWrite(_sda_pin, LOW); // Send ACK
                } else {
                    _state = I2C_IDLE; // Not for us
                }
            } else if (_state == I2C_READ) {
                if (_rx_buffer_index < I2C_BUFFER_SIZE) {
                    _rx_buffer[_rx_buffer_index++] = _current_byte;
                }
                _state = I2C_ACK;
                sda_mode(_sda_pin, OUTPUT);
                digitalWrite(_sda_pin, LOW); // Send ACK
            } else if (_state == I2C_WRITE) {
                // Read ACK/NACK from master
                bool nack = digitalRead(_sda_pin);
                if (nack) {
                    _state = I2C_IDLE; // Master NACK'd, end of transfer
                } else {
                    _state = I2C_ACK; // Master ACK'd, continue
                }
            }
            _bit_count = 0; // Reset for next byte
        }
    }
}


size_t SoftSlaveI2C::write(uint8_t data) {
    if (_tx_buffer_size < I2C_BUFFER_SIZE) {
        _tx_buffer[_tx_buffer_size++] = data;
        return 1;
    }
    return 0;
}

size_t SoftSlaveI2C::write(const uint8_t *data, size_t quantity) {
    size_t written = 0;
    for (size_t i = 0; i < quantity; i++) {
        if (write(data[i])) {
            written++;
        } else {
            break;
        }
    }
    return written;
}

int SoftSlaveI2C::available() {
    return _rx_buffer_index - _rx_buffer_head;
}

int SoftSlaveI2C::read() {
    if (_rx_buffer_head == _rx_buffer_index) {
        return -1;
    }
    return _rx_buffer[_rx_buffer_head++];
}

SoftSlaveI2C SlaveI2C;