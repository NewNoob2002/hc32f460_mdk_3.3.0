/**
 * SoftWire Demo - 软件I2C库使用示例
 * 
 * 本示例展示如何使用SoftWire库进行I2C通信，包括：
 * 1. 基础初始化
 * 2. 设备检测
 * 3. 8位数据读写
 * 4. 16位数据读写（大端序和小端序）
 * 5. 总线扫描
 * 6. Wire兼容接口使用
 */

#include "soft_i2c.h"

// 定义I2C引脚 - 根据你的硬件连接修改
#define SOFT_I2C_SDA_PIN    PA8   // SDA引脚
#define SOFT_I2C_SCL_PIN    PA9   // SCL引脚

// 创建软件I2C对象
SoftWire softWire(SOFT_I2C_SDA_PIN, SOFT_I2C_SCL_PIN);

// 示例设备地址 (7位地址)
#define DEVICE_ADDR     0x48  // 示例：温度传感器地址
#define EEPROM_ADDR     0x50  // 示例：EEPROM地址

void setup() {
    Serial.begin(115200);
    Serial.println("SoftWire Demo Starting...");
    
    // 初始化软件I2C
    if (softWire.begin()) {
        Serial.println("SoftWire initialized successfully!");
    } else {
        Serial.println("SoftWire initialization failed!");
        return;
    }
    
    // 设置I2C时钟频率为100kHz
    softWire.setClock(100000);
    Serial.println("I2C clock set to 100kHz");
    
    // 演示总线扫描
    demonstrateBusScan();
    
    // 演示设备检测
    demonstrateDeviceDetection();
    
    // 演示8位数据读写
    demonstrate8BitReadWrite();
    
    // 演示16位数据读写
    demonstrate16BitReadWrite();
    
    // 演示Wire兼容接口
    demonstrateWireInterface();
}

void loop() {
    // 主循环可以放置持续的I2C操作
    delay(5000);
    
    // 示例：定期读取温度传感器数据
    uint8_t temp_data;
    SoftI2CStatus status = softWire.readRegister8(DEVICE_ADDR, 0x00, &temp_data);
    if (status == SOFT_I2C_SUCCESS) {
        Serial.printf("Temperature register: 0x%02X\n", temp_data);
    }
}

/**
 * 演示总线扫描功能
 */
void demonstrateBusScan() {
    Serial.println("\n=== I2C Bus Scan Demo ===");
    
    uint8_t found_devices[16];
    uint8_t device_count = softWire.scanBus(found_devices, 16);
    
    Serial.printf("Found %d I2C devices:\n", device_count);
    for (uint8_t i = 0; i < device_count; i++) {
        Serial.printf("  Device %d: 0x%02X\n", i + 1, found_devices[i]);
    }
}

/**
 * 演示设备检测功能
 */
void demonstrateDeviceDetection() {
    Serial.println("\n=== Device Detection Demo ===");
    
    // 检测示例设备是否在线
    if (softWire.isDeviceOnline(DEVICE_ADDR)) {
        Serial.printf("Device 0x%02X is online\n", DEVICE_ADDR);
    } else {
        Serial.printf("Device 0x%02X is offline\n", DEVICE_ADDR);
    }
    
    if (softWire.isDeviceOnline(EEPROM_ADDR)) {
        Serial.printf("Device 0x%02X is online\n", EEPROM_ADDR);
    } else {
        Serial.printf("Device 0x%02X is offline\n", EEPROM_ADDR);
    }
}

/**
 * 演示8位数据读写
 */
void demonstrate8BitReadWrite() {
    Serial.println("\n=== 8-bit Read/Write Demo ===");
    
    // 写单个8位寄存器
    uint8_t write_data = 0xAB;
    SoftI2CStatus status = softWire.writeRegister8(DEVICE_ADDR, 0x01, write_data);
    Serial.printf("Write 8-bit register: %s\n", softWire.getStatusString(status));
    
    // 读单个8位寄存器
    uint8_t read_data;
    status = softWire.readRegister8(DEVICE_ADDR, 0x01, &read_data);
    if (status == SOFT_I2C_SUCCESS) {
        Serial.printf("Read 8-bit register: 0x%02X\n", read_data);
    } else {
        Serial.printf("Read 8-bit register failed: %s\n", softWire.getStatusString(status));
    }
    
    // 写多个8位数据
    uint8_t write_buffer[] = {0x12, 0x34, 0x56, 0x78};
    status = softWire.writeBytes(DEVICE_ADDR, 0x10, write_buffer, sizeof(write_buffer));
    Serial.printf("Write multiple bytes: %s\n", softWire.getStatusString(status));
    
    // 读多个8位数据
    uint8_t read_buffer[4];
    status = softWire.readBytes(DEVICE_ADDR, 0x10, read_buffer, sizeof(read_buffer));
    if (status == SOFT_I2C_SUCCESS) {
        Serial.print("Read multiple bytes: ");
        for (int i = 0; i < sizeof(read_buffer); i++) {
            Serial.printf("0x%02X ", read_buffer[i]);
        }
        Serial.println();
    } else {
        Serial.printf("Read multiple bytes failed: %s\n", softWire.getStatusString(status));
    }
}

/**
 * 演示16位数据读写
 */
void demonstrate16BitReadWrite() {
    Serial.println("\n=== 16-bit Read/Write Demo ===");
    
    // 16位大端序写入
    uint16_t write_data_be = 0x1234;
    SoftI2CStatus status = softWire.writeRegister16BE(DEVICE_ADDR, 0x20, write_data_be);
    Serial.printf("Write 16-bit BE register: %s\n", softWire.getStatusString(status));
    
    // 16位大端序读取
    uint16_t read_data_be;
    status = softWire.readRegister16BE(DEVICE_ADDR, 0x20, &read_data_be);
    if (status == SOFT_I2C_SUCCESS) {
        Serial.printf("Read 16-bit BE register: 0x%04X\n", read_data_be);
    } else {
        Serial.printf("Read 16-bit BE register failed: %s\n", softWire.getStatusString(status));
    }
    
    // 16位小端序写入
    uint16_t write_data_le = 0x5678;
    status = softWire.writeRegister16LE(DEVICE_ADDR, 0x22, write_data_le);
    Serial.printf("Write 16-bit LE register: %s\n", softWire.getStatusString(status));
    
    // 16位小端序读取
    uint16_t read_data_le;
    status = softWire.readRegister16LE(DEVICE_ADDR, 0x22, &read_data_le);
    if (status == SOFT_I2C_SUCCESS) {
        Serial.printf("Read 16-bit LE register: 0x%04X\n", read_data_le);
    } else {
        Serial.printf("Read 16-bit LE register failed: %s\n", softWire.getStatusString(status));
    }
}

/**
 * 演示Wire兼容接口
 */
void demonstrateWireInterface() {
    Serial.println("\n=== Wire Compatible Interface Demo ===");
    
    // 使用Wire风格的API写数据
    softWire.beginTransmission(DEVICE_ADDR);
    softWire.write(0x30);  // 寄存器地址
    softWire.write(0xDE);  // 数据1
    softWire.write(0xAD);  // 数据2
    uint8_t result = softWire.endTransmission();
    Serial.printf("Wire-style write result: %d\n", result);
    
    // 使用Wire风格的API读数据
    uint8_t bytes_received = softWire.requestFrom(DEVICE_ADDR, 2);
    Serial.printf("Wire-style read received %d bytes: ", bytes_received);
    
    while (softWire.available()) {
        uint8_t data = softWire.read();
        Serial.printf("0x%02X ", data);
    }
    Serial.println();
}

/**
 * 高级应用示例：读取温度传感器
 */
void readTemperatureSensor() {
    const uint8_t TEMP_REG = 0x00;
    uint16_t temp_raw;
    
    // 假设使用16位大端序读取温度
    SoftI2CStatus status = softWire.readRegister16BE(DEVICE_ADDR, TEMP_REG, &temp_raw);
    
    if (status == SOFT_I2C_SUCCESS) {
        // 转换为实际温度值（具体公式取决于传感器）
        float temperature = (float)temp_raw * 0.0625;  // 示例转换公式
        Serial.printf("Temperature: %.2f°C\n", temperature);
    } else {
        Serial.printf("Temperature read failed: %s\n", softWire.getStatusString(status));
    }
}

/**
 * 高级应用示例：EEPROM读写
 */
void eepromReadWrite() {
    const uint8_t EEPROM_PAGE_SIZE = 16;
    uint8_t write_data[EEPROM_PAGE_SIZE];
    uint8_t read_data[EEPROM_PAGE_SIZE];
    
    // 准备测试数据
    for (int i = 0; i < EEPROM_PAGE_SIZE; i++) {
        write_data[i] = i + 0xA0;
    }
    
    // 写入EEPROM页
    SoftI2CStatus status = softWire.writeBytes(EEPROM_ADDR, 0x00, write_data, EEPROM_PAGE_SIZE);
    if (status == SOFT_I2C_SUCCESS) {
        Serial.println("EEPROM write successful");
        
        // 等待写入完成
        delay(10);
        
        // 读取EEPROM页
        status = softWire.readBytes(EEPROM_ADDR, 0x00, read_data, EEPROM_PAGE_SIZE);
        if (status == SOFT_I2C_SUCCESS) {
            Serial.print("EEPROM read data: ");
            for (int i = 0; i < EEPROM_PAGE_SIZE; i++) {
                Serial.printf("0x%02X ", read_data[i]);
            }
            Serial.println();
            
            // 验证数据
            bool data_match = true;
            for (int i = 0; i < EEPROM_PAGE_SIZE; i++) {
                if (write_data[i] != read_data[i]) {
                    data_match = false;
                    break;
                }
            }
            Serial.printf("Data verification: %s\n", data_match ? "PASSED" : "FAILED");
        }
    }
} 