#include "soft_i2c.h"
#include "drivers/gpio/gpio.h"
#include <Arduino.h>

// HC32F460 GPIO寄存器基地址定义
#define HC32_GPIO_BASE      (0x40053800ul)
#define HC32_PODR_OFFSET    (0x0004ul)    // 输出数据寄存器偏移
#define HC32_POER_OFFSET    (0x0006ul)    // 输出使能寄存器偏移
#define HC32_PIDR_OFFSET    (0x0000ul)    // 输入数据寄存器偏移

Wire wire(SOFT_I2C_SDA_PIN, SOFT_I2C_SCL_PIN);
// 构造函数
Wire::Wire(gpio_pin_t sda_pin, gpio_pin_t scl_pin, uint32_t delay_us) 
    : _sda_pin(sda_pin), _scl_pin(scl_pin), _delay_us(delay_us), _initialized(false),
      _sda_podr(nullptr), _sda_poer(nullptr), _sda_pidr(nullptr),
      _scl_podr(nullptr), _scl_poer(nullptr), _scl_pidr(nullptr),
      _sda_pin_mask(0), _scl_pin_mask(0),
      _tx_address(0), _tx_buffer_length(0), _rx_buffer_length(0), _rx_buffer_index(0) {
}

// ==================== 寄存器指针初始化 ====================

void Wire::init_register_pointers(void) {
    // 获取SDA引脚信息
    const pin_info_t& sda_info = PIN_MAP[_sda_pin];
    _sda_pin_mask = sda_info.bit_mask();
    uint32_t sda_port_base = HC32_GPIO_BASE + ((uint32_t)sda_info.port * 0x10ul);
    _sda_podr = (volatile uint16_t*)(sda_port_base + HC32_PODR_OFFSET);
    _sda_poer = (volatile uint16_t*)(sda_port_base + HC32_POER_OFFSET);
    _sda_pidr = (volatile uint16_t*)(sda_port_base + HC32_PIDR_OFFSET);
    
    // 获取SCL引脚信息
    const pin_info_t& scl_info = PIN_MAP[_scl_pin];
    _scl_pin_mask = scl_info.bit_mask();
    uint32_t scl_port_base = HC32_GPIO_BASE + ((uint32_t)scl_info.port * 0x10ul);
    _scl_podr = (volatile uint16_t*)(scl_port_base + HC32_PODR_OFFSET);
    _scl_poer = (volatile uint16_t*)(scl_port_base + HC32_POER_OFFSET);
    _scl_pidr = (volatile uint16_t*)(scl_port_base + HC32_PIDR_OFFSET);
    
    CORE_DEBUG_PRINTF("SoftWire: SDA P%c%d mask=0x%04X, SCL P%c%d mask=0x%04X\n",
                     'A' + sda_info.port, sda_info.bit_pos, _sda_pin_mask,
                     'A' + scl_info.port, scl_info.bit_pos, _scl_pin_mask);
}

// ==================== 私有函数：基础时序控制 - 寄存器优化版本 ====================

inline void Wire::i2c_delay(void) {
    if (_delay_us > 0) {
        delay_us(_delay_us);
    }
}

inline void Wire::sda_high(void) {
    // 设置为输入模式（高阻态），依靠外部上拉电阻
    *_sda_poer &= ~_sda_pin_mask;  // 禁用输出
}

inline void Wire::sda_low(void) {
    // 设置为输出低电平
    *_sda_podr &= ~_sda_pin_mask;  // 输出数据设为0
    *_sda_poer |= _sda_pin_mask;   // 使能输出
}

inline void Wire::scl_high(void) {
    // 设置为输入模式（高阻态），依靠外部上拉电阻
    *_scl_poer &= ~_scl_pin_mask;  // 禁用输出
}

inline void Wire::scl_low(void) {
    // 设置为输出低电平
    *_scl_podr &= ~_scl_pin_mask;  // 输出数据设为0
    *_scl_poer |= _scl_pin_mask;   // 使能输出
}

inline bool Wire::read_sda(void) {
    return (*_sda_pidr & _sda_pin_mask) != 0;
}

inline bool Wire::read_scl(void) {
    return (*_scl_pidr & _scl_pin_mask) != 0;
}

bool Wire::wait_scl_high(uint32_t timeout_us) {
    uint32_t start_time = micros();
    while (!read_scl()) {
        if (micros() - start_time > timeout_us) {
            return false;  // 超时
        }
        delay_us(1);
    }
    return true;
}

// ==================== 私有函数：I2C协议实现 ====================

void Wire::i2c_start(void) {
    sda_high();
    scl_high();
    i2c_delay();
    sda_low();
    i2c_delay();
    scl_low();
    i2c_delay();
}

void Wire::i2c_stop(void) {
    sda_low();
    scl_low();
    i2c_delay();
    scl_high();
    wait_scl_high();
    i2c_delay();
    sda_high();
    i2c_delay();
}

bool Wire::i2c_write_bit(bool bit) {
    // 设置数据线
    if (bit) {
        sda_high();
    } else {
        sda_low();
    }
    i2c_delay();
    
    // 时钟高脉冲
    scl_high();
    if (!wait_scl_high()) {
        return false;  // 时钟拉伸超时
    }
    i2c_delay();
    scl_low();
    i2c_delay();
    
    return true;
}

bool Wire::i2c_read_bit(void) {
    bool bit;
    
    sda_high();  // 释放数据线
    i2c_delay();
    scl_high();
    if (!wait_scl_high()) {
        return false;  // 时钟拉伸超时
    }
    i2c_delay();
    bit = read_sda();
    scl_low();
    i2c_delay();
    
    return bit;
}

bool Wire::i2c_write_byte(uint8_t data) {
    bool ack;
    
    // 发送8位数据（MSB先发送）
    for (int i = 7; i >= 0; i--) {
        if (!i2c_write_bit((data >> i) & 0x01)) {
            return false;
        }
    }
    
    // 读取ACK位
    ack = !i2c_read_bit();  // ACK为低电平
    
    return ack;
}

uint8_t Wire::i2c_read_byte(bool ack) {
    uint8_t data = 0;
    
    // 读取8位数据（MSB先接收）
    for (int i = 7; i >= 0; i--) {
        if (i2c_read_bit()) {
            data |= (1 << i);
        }
    }
    
    // 发送ACK/NACK
    i2c_write_bit(!ack);  // ACK为低电平，NACK为高电平
    
    return data;
}

// ==================== 公共函数：基础操作 ====================

bool Wire::begin(void) {
    if (_initialized) {
        return true;
    }
    
    // 初始化寄存器指针
    init_register_pointers();
    
    // 配置引脚为GPIO功能
    stc_port_init_t port_init;
    port_init.enPinMode = Pin_Mode_In;        // 初始为输入模式
    port_init.enLatch = Disable;
    port_init.enExInt = Disable;
    port_init.enInvert = Disable;
    port_init.enPullUp = Enable;              // 使能内部上拉
    port_init.enPinDrv = Pin_Drv_H;           // 高驱动能力
    port_init.enPinOType = Pin_OType_Cmos;    // CMOS输出
    port_init.enPinSubFunc = Disable;
    
    // 初始化SDA和SCL引脚
    GPIO_Init(_sda_pin, &port_init);
    GPIO_Init(_scl_pin, &port_init);
    
    // 初始化引脚为高阻态
    sda_high();
    scl_high();
    
    // 等待总线稳定
    delay_us(10);
    
    // 检查总线是否空闲
    if (!read_sda() || !read_scl()) {
        CORE_DEBUG_PRINTF("SoftWire: Bus not free during init\n");
        return false;
    }
    
    _initialized = true;
    CORE_DEBUG_PRINTF("SoftWire: Register-optimized init complete, SDA=%d, SCL=%d, delay=%luus\n", 
                     _sda_pin, _scl_pin, _delay_us);
    return true;
}

void Wire::end(void) {
    if (!_initialized) {
        return;
    }
    
    // 确保总线处于空闲状态
    sda_high();
    scl_high();
    
    _initialized = false;
}

void Wire::setClock(uint32_t frequency) {
    if (frequency > 800000) {
        frequency = 800000;  // 寄存器优化后最大800kHz
    } else if (frequency < 10000) {
        frequency = 10000;   // 最小10kHz
    }
    
    // 寄存器操作优化的延时计算
    // 寄存器操作比Arduino函数快约10-20倍，大幅减少所需延时
    uint32_t half_period_us = 500000 / frequency;
    
    if (frequency > 400000) {
        // 高频模式：寄存器操作本身提供足够延时
        _delay_us = (half_period_us > 1) ? (half_period_us - 1) : 0;
    } else if (frequency > 200000) {
        // 中高频模式：适度减少延时
        _delay_us = (half_period_us > 2) ? (half_period_us - 2) : 1;
    } else {
        // 标准模式：保守的延时设置
        _delay_us = half_period_us;
    }
    
    CORE_DEBUG_PRINTF("SoftWire: Clock set to %uHz, optimized delay=%luus\n", frequency, _delay_us);
}

bool Wire::isDeviceOnline(uint8_t address, uint8_t retries) {
    if (!_initialized) {
        return false;
    }
    
    for (uint8_t i = 0; i < retries; i++) {
        i2c_start();
        bool ack = i2c_write_byte((address << 1) | 0);  // 写命令
        i2c_stop();
        
        if (ack) {
            return true;
        }
        
        delay_us(1000);  // 重试间隔
    }
    
    return false;
}

// ==================== 8位数据读写 ====================

SoftI2CStatus Wire::writeRegister8(uint8_t device_addr, uint8_t reg_addr, uint8_t data) {
    if (!_initialized) {
        return SOFT_I2C_BUS_ERROR;
    }
    
    i2c_start();
    
    // 发送设备地址（写）
    if (!i2c_write_byte((device_addr << 1) | 0)) {
        i2c_stop();
        return SOFT_I2C_NACK_ADDR;
    }
    
    // 发送寄存器地址
    if (!i2c_write_byte(reg_addr)) {
        i2c_stop();
        return SOFT_I2C_NACK_DATA;
    }
    
    // 发送数据
    if (!i2c_write_byte(data)) {
        i2c_stop();
        return SOFT_I2C_NACK_DATA;
    }
    
    i2c_stop();
    return SOFT_I2C_SUCCESS;
}

SoftI2CStatus Wire::readRegister8(uint8_t device_addr, uint8_t reg_addr, uint8_t* data) {
    if (!_initialized || !data) {
        return SOFT_I2C_BUS_ERROR;
    }
    
    // 写寄存器地址
    i2c_start();
    if (!i2c_write_byte((device_addr << 1) | 0)) {
        i2c_stop();
        return SOFT_I2C_NACK_ADDR;
    }
    if (!i2c_write_byte(reg_addr)) {
        i2c_stop();
        return SOFT_I2C_NACK_DATA;
    }
    
    // 重新开始读取
    i2c_start();
    if (!i2c_write_byte((device_addr << 1) | 1)) {
        i2c_stop();
        return SOFT_I2C_NACK_ADDR;
    }
    
    // 读取数据
    *data = i2c_read_byte(false);  // NACK（最后一个字节）
    i2c_stop();
    
    return SOFT_I2C_SUCCESS;
}

SoftI2CStatus Wire::writeBytes(uint8_t device_addr, uint8_t reg_addr, const uint8_t* data, uint8_t length) {
    if (!_initialized || !data || length == 0) {
        return SOFT_I2C_BUS_ERROR;
    }
    
    i2c_start();
    
    // 发送设备地址（写）
    if (!i2c_write_byte((device_addr << 1) | 0)) {
        i2c_stop();
        return SOFT_I2C_NACK_ADDR;
    }
    
    // 发送寄存器地址
    if (!i2c_write_byte(reg_addr)) {
        i2c_stop();
        return SOFT_I2C_NACK_DATA;
    }
    
    // 发送数据
    for (uint8_t i = 0; i < length; i++) {
        if (!i2c_write_byte(data[i])) {
            i2c_stop();
            return SOFT_I2C_NACK_DATA;
        }
    }
    
    i2c_stop();
    return SOFT_I2C_SUCCESS;
}

SoftI2CStatus Wire::readBytes(uint8_t device_addr, uint8_t reg_addr, uint8_t* data, uint8_t length) {
    if (!_initialized || !data || length == 0) {
        return SOFT_I2C_BUS_ERROR;
    }
    
    // 写寄存器地址
    i2c_start();
    if (!i2c_write_byte((device_addr << 1) | 0)) {
        i2c_stop();
        return SOFT_I2C_NACK_ADDR;
    }
    if (!i2c_write_byte(reg_addr)) {
        i2c_stop();
        return SOFT_I2C_NACK_DATA;
    }
    
    // 重新开始读取
    i2c_start();
    if (!i2c_write_byte((device_addr << 1) | 1)) {
        i2c_stop();
        return SOFT_I2C_NACK_ADDR;
    }
    
    // 读取数据
    for (uint8_t i = 0; i < length; i++) {
        data[i] = i2c_read_byte(i < (length - 1));  // 最后一个字节发送NACK
    }
    
    i2c_stop();
    return SOFT_I2C_SUCCESS;
}

// ==================== 16位数据读写 ====================

SoftI2CStatus Wire::writeRegister16BE(uint8_t device_addr, uint8_t reg_addr, uint16_t data) {
    uint8_t bytes[2];
    bytes[0] = (data >> 8) & 0xFF;   // 高字节
    bytes[1] = data & 0xFF;          // 低字节
    return writeBytes(device_addr, reg_addr, bytes, 2);
}

SoftI2CStatus Wire::readRegister16BE(uint8_t device_addr, uint8_t reg_addr, uint16_t* data) {
    uint8_t bytes[2];
    SoftI2CStatus status = readBytes(device_addr, reg_addr, bytes, 2);
    if (status == SOFT_I2C_SUCCESS && data) {
        *data = (bytes[0] << 8) | bytes[1];  // 大端序
    }
    return status;
}

SoftI2CStatus Wire::writeRegister16LE(uint8_t device_addr, uint8_t reg_addr, uint16_t data) {
    uint8_t bytes[2];
    bytes[0] = data & 0xFF;          // 低字节
    bytes[1] = (data >> 8) & 0xFF;   // 高字节
    return writeBytes(device_addr, reg_addr, bytes, 2);
}

SoftI2CStatus Wire::readRegister16LE(uint8_t device_addr, uint8_t reg_addr, uint16_t* data) {
    uint8_t bytes[2];
    SoftI2CStatus status = readBytes(device_addr, reg_addr, bytes, 2);
    if (status == SOFT_I2C_SUCCESS && data) {
        *data = (bytes[1] << 8) | bytes[0];  // 小端序
    }
    return status;
}

// ==================== Wire兼容接口 ====================

void Wire::beginTransmission(uint8_t address) {
    _tx_address = address;
    _tx_buffer_length = 0;
}

size_t Wire::write(uint8_t data) {
    if (_tx_buffer_length >= sizeof(_tx_buffer)) {
        return 0;  // 缓冲区满
    }
    _tx_buffer[_tx_buffer_length++] = data;
    return 1;
}

uint8_t Wire::endTransmission(bool sendStop) {
    if (!_initialized) {
        return 4;  // 其他错误
    }
    
    if (_tx_buffer_length == 0) {
        return 0;  // 没有数据要发送
    }
    
    i2c_start();
    
    // 发送设备地址（写）
    if (!i2c_write_byte((_tx_address << 1) | 0)) {
        if (sendStop) i2c_stop();
        return 2;  // 地址NACK
    }
    
    // 发送数据
    for (uint8_t i = 0; i < _tx_buffer_length; i++) {
        if (!i2c_write_byte(_tx_buffer[i])) {
            if (sendStop) i2c_stop();
            return 3;  // 数据NACK
        }
    }
    
    if (sendStop) {
        i2c_stop();
    }
    
    _tx_buffer_length = 0;  // 清空缓冲区
    return 0;  // 成功
}

uint8_t Wire::requestFrom(uint8_t address, uint8_t quantity, bool sendStop) {
    if (!_initialized || quantity == 0 || quantity > sizeof(_rx_buffer)) {
        return 0;
    }
    
    i2c_start();
    
    // 发送设备地址（读）
    if (!i2c_write_byte((address << 1) | 1)) {
        if (sendStop) i2c_stop();
        return 0;
    }
    
    // 读取数据
    _rx_buffer_length = 0;
    _rx_buffer_index = 0;
    
    for (uint8_t i = 0; i < quantity; i++) {
        _rx_buffer[i] = i2c_read_byte(i < (quantity - 1));  // 最后一个字节发送NACK
        _rx_buffer_length++;
    }
    
    if (sendStop) {
        i2c_stop();
    }
    
    return _rx_buffer_length;
}

int Wire::available(void) {
    return _rx_buffer_length - _rx_buffer_index;
}

int Wire::read(void) {
    if (_rx_buffer_index >= _rx_buffer_length) {
        return -1;
    }
    return _rx_buffer[_rx_buffer_index++];
}

// ==================== 工具函数 ====================

const char* Wire::getStatusString(SoftI2CStatus status) {
    switch (status) {
        case SOFT_I2C_SUCCESS:        return "Success";
        case SOFT_I2C_NACK_ADDR:      return "NACK on address";
        case SOFT_I2C_NACK_DATA:      return "NACK on data";
        case SOFT_I2C_TIMEOUT_ERROR:  return "Timeout error";
        case SOFT_I2C_BUS_ERROR:      return "Bus error";
        default:                      return "Unknown error";
    }
}

uint8_t Wire::scanBus(uint8_t* found_devices, uint8_t max_devices) {
    if (!_initialized || !found_devices || max_devices == 0) {
        return 0;
    }
    
    uint8_t count = 0;
    
    CORE_DEBUG_PRINTF("SoftWire: Scanning I2C bus...\n");
    
    for (uint8_t addr = 1; addr < 127 && count < max_devices; addr++) {
        i2c_start();
        bool ack = i2c_write_byte((addr << 1) | 0);  // 尝试写命令
        i2c_stop();
        
        if (ack) {
            found_devices[count++] = addr;
            CORE_DEBUG_PRINTF("SoftWire: Found device at 0x%02X\n", addr);
        }
        
        delay_us(100);  // 短暂延时
    }
    
    CORE_DEBUG_PRINTF("SoftWire: Scan complete, found %d devices\n", count);
    return count;
}