#ifndef SOFT_I2C_H_
#define SOFT_I2C_H_

#include <core_types.h>

// 定义I2C引脚 - 根据你的硬件连接修改
#define SOFT_I2C_SDA_PIN    PA8   // SDA引脚
#define SOFT_I2C_SCL_PIN    PA9   // SCL引脚
// 软件I2C配置 - 寄存器优化版本
#define SOFT_I2C_DELAY_US      1     // I2C时序延时(微秒) - 寄存器优化，可用更小延时
#define SOFT_I2C_TIMEOUT       1000  // 超时时间(毫秒)
#define SOFT_I2C_MAX_RETRIES   3     // 最大重试次数

// I2C操作结果枚举
enum SoftI2CStatus : uint8_t {
    SOFT_I2C_SUCCESS = 0,        // 成功
    SOFT_I2C_NACK_ADDR = 1,      // 地址NACK
    SOFT_I2C_NACK_DATA = 2,      // 数据NACK
    SOFT_I2C_TIMEOUT_ERROR = 3,  // 超时错误
    SOFT_I2C_BUS_ERROR = 4       // 总线错误
};

/**
 * 软件I2C类 - 兼容Wire接口
 */
class Wire {
private:
    gpio_pin_t _sda_pin;    // SDA引脚
    gpio_pin_t _scl_pin;    // SCL引脚
    uint32_t _delay_us;     // 时序延时
    bool _initialized;      // 初始化标志
    
    // 寄存器直接操作优化 - 高速GPIO控制
    volatile uint16_t *_sda_podr;     // SDA输出数据寄存器
    volatile uint16_t *_sda_poer;     // SDA输出使能寄存器
    volatile uint16_t *_sda_pidr;     // SDA输入数据寄存器
    volatile uint16_t *_scl_podr;     // SCL输出数据寄存器
    volatile uint16_t *_scl_poer;     // SCL输出使能寄存器
    volatile uint16_t *_scl_pidr;     // SCL输入数据寄存器
    
    uint16_t _sda_pin_mask;           // SDA引脚位掩码
    uint16_t _scl_pin_mask;           // SCL引脚位掩码
    
    // 低级I2C时序控制函数 - 寄存器优化版本
    inline void i2c_delay(void);
    inline void sda_high(void);
    inline void sda_low(void);
    inline void scl_high(void);
    inline void scl_low(void);
    inline bool read_sda(void);
    inline bool read_scl(void);
    
    // 寄存器地址计算和初始化
    void init_register_pointers(void);
    
    // I2C协议函数
    void i2c_start(void);
    void i2c_stop(void);
    bool i2c_write_bit(bool bit);
    bool i2c_read_bit(void);
    bool i2c_write_byte(uint8_t data);
    uint8_t i2c_read_byte(bool ack);
    
    // 等待SCL高电平（时钟拉伸处理）
    bool wait_scl_high(uint32_t timeout_us = 1000);

public:
    /**
     * 构造函数 - 寄存器优化版本
     * @param sda_pin SDA引脚
     * @param scl_pin SCL引脚
     * @param delay_us 时序延时(微秒)，默认1us支持高达800kHz传输
     */
    Wire(gpio_pin_t sda_pin, gpio_pin_t scl_pin, uint32_t delay_us = SOFT_I2C_DELAY_US);
    
    /**
     * 初始化软件I2C
     * @return true:成功, false:失败
     */
    bool begin(void);
    
    /**
     * 结束软件I2C，释放引脚
     */
    void end(void);
    
    /**
     * 设置时钟频率
     * @param frequency 频率(Hz)，支持10kHz-400kHz
     */
    void setClock(uint32_t frequency);
    
    /**
     * 检测设备是否在线
     * @param address 7位设备地址
     * @param retries 重试次数
     * @return true:设备在线, false:设备离线
     */
    bool isDeviceOnline(uint8_t address, uint8_t retries = SOFT_I2C_MAX_RETRIES);
    
    // ==================== 8位数据读写 ====================
    
    /**
     * 写8位寄存器
     * @param device_addr 7位设备地址
     * @param reg_addr 寄存器地址
     * @param data 要写入的数据
     * @return 操作状态
     */
    SoftI2CStatus writeRegister8(uint8_t device_addr, uint8_t reg_addr, uint8_t data);
    
    /**
     * 读8位寄存器
     * @param device_addr 7位设备地址  
     * @param reg_addr 寄存器地址
     * @param data 读取数据的指针
     * @return 操作状态
     */
    SoftI2CStatus readRegister8(uint8_t device_addr, uint8_t reg_addr, uint8_t* data);
    
    /**
     * 写多个8位数据
     * @param device_addr 7位设备地址
     * @param reg_addr 起始寄存器地址
     * @param data 数据缓冲区
     * @param length 数据长度
     * @return 操作状态
     */
    SoftI2CStatus writeBytes(uint8_t device_addr, uint8_t reg_addr, const uint8_t* data, uint8_t length);
    
    /**
     * 读多个8位数据
     * @param device_addr 7位设备地址
     * @param reg_addr 起始寄存器地址  
     * @param data 数据缓冲区
     * @param length 数据长度
     * @return 操作状态
     */
    SoftI2CStatus readBytes(uint8_t device_addr, uint8_t reg_addr, uint8_t* data, uint8_t length);
    
    // ==================== 16位数据读写 ====================
    
    /**
     * 写16位寄存器 (大端序)
     * @param device_addr 7位设备地址
     * @param reg_addr 寄存器地址
     * @param data 要写入的16位数据
     * @return 操作状态
     */
    SoftI2CStatus writeRegister16BE(uint8_t device_addr, uint8_t reg_addr, uint16_t data);
    
    /**
     * 读16位寄存器 (大端序)
     * @param device_addr 7位设备地址
     * @param reg_addr 寄存器地址
     * @param data 读取数据的指针
     * @return 操作状态
     */
    SoftI2CStatus readRegister16BE(uint8_t device_addr, uint8_t reg_addr, uint16_t* data);
    
    /**
     * 写16位寄存器 (小端序)
     * @param device_addr 7位设备地址
     * @param reg_addr 寄存器地址
     * @param data 要写入的16位数据
     * @return 操作状态
     */
    SoftI2CStatus writeRegister16LE(uint8_t device_addr, uint8_t reg_addr, uint16_t data);
    
    /**
     * 读16位寄存器 (小端序)
     * @param device_addr 7位设备地址
     * @param reg_addr 寄存器地址
     * @param data 读取数据的指针
     * @return 操作状态
     */
    SoftI2CStatus readRegister16LE(uint8_t device_addr, uint8_t reg_addr, uint16_t* data);
    
    // ==================== 兼容Wire接口 ====================
    
    /**
     * 开始传输
     * @param address 7位设备地址
     */
    void beginTransmission(uint8_t address);
    
    /**
     * 写数据到传输缓冲区
     * @param data 要写入的数据
     * @return 写入的字节数
     */
    size_t write(uint8_t data);
    
    /**
     * 结束传输
     * @param sendStop 是否发送停止位
     * @return 传输状态
     */
    uint8_t endTransmission(bool sendStop = true);
    
    /**
     * 从设备请求数据
     * @param address 7位设备地址
     * @param quantity 请求的字节数
     * @param sendStop 是否发送停止位
     * @return 实际接收的字节数
     */
    uint8_t requestFrom(uint8_t address, uint8_t quantity, bool sendStop = true);
    
    /**
     * 检查可用数据
     * @return 可用字节数
     */
    int available(void);
    
    /**
     * 读取一个字节
     * @return 读取的数据，-1表示无数据
     */
    int read(void);
    
    // ==================== 工具函数 ====================
    
    /**
     * 获取状态描述字符串
     * @param status 状态码
     * @return 状态描述
     */
    const char* getStatusString(SoftI2CStatus status);
    
    /**
     * 扫描I2C总线上的设备
     * @param found_devices 发现的设备地址数组
     * @param max_devices 最大设备数量
     * @return 发现的设备数量
     */
    uint8_t scanBus(uint8_t* found_devices, uint8_t max_devices = 16);

private:
    // Wire兼容接口的内部变量
    uint8_t _tx_address;
    uint8_t _tx_buffer[32];
    uint8_t _tx_buffer_length;
    uint8_t _rx_buffer[32];
    uint8_t _rx_buffer_length;
    uint8_t _rx_buffer_index;
};


extern Wire wire;
#endif // SOFT_I2C_H_