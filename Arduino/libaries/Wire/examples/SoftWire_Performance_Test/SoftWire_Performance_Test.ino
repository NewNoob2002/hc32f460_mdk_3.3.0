/**
 * SoftWire Performance Test - 软件I2C性能测试
 * 
 * 本示例用于测试和验证SoftWire库的实际性能：
 * 1. 不同频率下的实际传输速度测试
 * 2. 与目标频率的对比分析
 * 3. 连续传输的稳定性测试
 * 4. 不同数据长度的性能对比
 */

#include "soft_i2c.h"

// 定义I2C引脚 - 根据你的硬件连接修改
#define SOFT_I2C_SDA_PIN    PA8   // SDA引脚
#define SOFT_I2C_SCL_PIN    PA9   // SCL引脚

// 创建软件I2C对象
SoftWire softWire(SOFT_I2C_SDA_PIN, SOFT_I2C_SCL_PIN);

// 测试设备地址 (确保有真实设备响应以获得准确测试结果)
#define TEST_DEVICE_ADDR    0x48

void setup() {
    Serial.begin(115200);
    Serial.println("SoftWire Performance Test Starting...");
    Serial.println("========================================");
    
    // 初始化软件I2C
    if (!softWire.begin()) {
        Serial.println("SoftWire initialization failed!");
        return;
    }
    
    delay(1000);
    
    // 测试不同的时钟频率
    testFrequencies();
    
    delay(1000);
    
    // 测试数据传输性能
    testDataTransferPerformance();
    
    delay(1000);
    
    // 测试连续传输稳定性
    testContinuousTransfer();
    
    Serial.println("\nPerformance test completed!");
}

void loop() {
    // 可以在这里添加持续的性能监控
    delay(5000);
}

/**
 * 测试不同频率下的实际性能
 */
void testFrequencies() {
    Serial.println("\n=== Frequency Performance Test ===");
    
    uint32_t test_frequencies[] = {10000, 20000, 50000, 100000, 200000, 400000};
    uint8_t freq_count = sizeof(test_frequencies) / sizeof(test_frequencies[0]);
    
    for (uint8_t i = 0; i < freq_count; i++) {
        uint32_t target_freq = test_frequencies[i];
        
        Serial.printf("\nTesting %d Hz:\n", target_freq);
        softWire.setClock(target_freq);
        
        // 测试单字节传输速度
        testSingleByteSpeed(target_freq);
        
        // 测试多字节传输速度
        testMultiByteSpeed(target_freq);
        
        delay(500);
    }
}

/**
 * 测试单字节传输速度
 */
void testSingleByteSpeed(uint32_t target_freq) {
    const int test_count = 100;
    uint32_t start_time, end_time;
    uint8_t test_data = 0xAA;
    int success_count = 0;
    
    // 测试写操作
    start_time = micros();
    for (int i = 0; i < test_count; i++) {
        SoftI2CStatus status = softWire.writeRegister8(TEST_DEVICE_ADDR, 0x01, test_data);
        if (status == SOFT_I2C_SUCCESS) {
            success_count++;
        }
    }
    end_time = micros();
    
    uint32_t total_time_us = end_time - start_time;
    float avg_time_per_transfer = (float)total_time_us / test_count;
    
    // 每次传输包含：START + 设备地址(8bit+ACK) + 寄存器地址(8bit+ACK) + 数据(8bit+ACK) + STOP
    // 总共约 30个位 = 30 * (1/frequency) 秒的理论时间
    float theoretical_time_us = 30.0 * 1000000.0 / target_freq;
    float efficiency = (theoretical_time_us / avg_time_per_transfer) * 100.0;
    
    Serial.printf("  Single byte write (%d tests):\n", test_count);
    Serial.printf("    Success rate: %d/%d (%.1f%%)\n", 
                  success_count, test_count, (float)success_count/test_count*100);
    Serial.printf("    Avg time: %.1f us/transfer\n", avg_time_per_transfer);
    Serial.printf("    Theoretical: %.1f us/transfer\n", theoretical_time_us);
    Serial.printf("    Efficiency: %.1f%%\n", efficiency);
    
    if (efficiency > 100) {
        Serial.printf("    Note: Efficiency >100%% indicates actual speed exceeds target\n");
    }
}

/**
 * 测试多字节传输速度
 */
void testMultiByteSpeed(uint32_t target_freq) {
    const int test_count = 20;
    const int data_length = 16;
    uint8_t test_data[data_length];
    uint32_t start_time, end_time;
    int success_count = 0;
    
    // 准备测试数据
    for (int i = 0; i < data_length; i++) {
        test_data[i] = i + 0x80;
    }
    
    // 测试多字节写操作
    start_time = micros();
    for (int i = 0; i < test_count; i++) {
        SoftI2CStatus status = softWire.writeBytes(TEST_DEVICE_ADDR, 0x10, test_data, data_length);
        if (status == SOFT_I2C_SUCCESS) {
            success_count++;
        }
    }
    end_time = micros();
    
    uint32_t total_time_us = end_time - start_time;
    float avg_time_per_transfer = (float)total_time_us / test_count;
    
    // 计算理论传输时间：START + 设备地址 + 寄存器地址 + N个数据字节 + STOP
    int total_bits = 1 + 9 + 9 + (data_length * 9) + 1;  // 包含ACK位
    float theoretical_time_us = (float)total_bits * 1000000.0 / target_freq;
    float throughput_bps = (float)(data_length * 8 * test_count) * 1000000.0 / total_time_us;
    
    Serial.printf("  Multi-byte write (%d bytes x %d tests):\n", data_length, test_count);
    Serial.printf("    Success rate: %d/%d (%.1f%%)\n", 
                  success_count, test_count, (float)success_count/test_count*100);
    Serial.printf("    Avg time: %.1f us/transfer\n", avg_time_per_transfer);
    Serial.printf("    Theoretical: %.1f us/transfer\n", theoretical_time_us);
    Serial.printf("    Data throughput: %.0f bps\n", throughput_bps);
}

/**
 * 测试数据传输性能对比
 */
void testDataTransferPerformance() {
    Serial.println("\n=== Data Transfer Performance Test ===");
    
    // 设置一个中等频率进行测试
    softWire.setClock(100000);
    Serial.println("Testing at 100kHz:");
    
    // 测试不同数据长度的性能
    uint8_t data_lengths[] = {1, 4, 8, 16, 32};
    uint8_t length_count = sizeof(data_lengths) / sizeof(data_lengths[0]);
    
    for (uint8_t i = 0; i < length_count; i++) {
        uint8_t length = data_lengths[i];
        testDataLengthPerformance(length);
    }
}

/**
 * 测试特定数据长度的性能
 */
void testDataLengthPerformance(uint8_t data_length) {
    const int test_count = 50;
    uint8_t test_data[32];  // 最大支持32字节
    uint32_t start_time, end_time;
    int success_count = 0;
    
    // 准备测试数据
    for (int i = 0; i < data_length; i++) {
        test_data[i] = i;
    }
    
    // 测试写入性能
    start_time = micros();
    for (int i = 0; i < test_count; i++) {
        SoftI2CStatus status = softWire.writeBytes(TEST_DEVICE_ADDR, 0x00, test_data, data_length);
        if (status == SOFT_I2C_SUCCESS) {
            success_count++;
        }
    }
    end_time = micros();
    
    uint32_t write_time_us = end_time - start_time;
    float write_throughput = (float)(data_length * test_count * 8) * 1000000.0 / write_time_us;
    
    // 测试读取性能
    uint8_t read_data[32];
    success_count = 0;
    start_time = micros();
    for (int i = 0; i < test_count; i++) {
        SoftI2CStatus status = softWire.readBytes(TEST_DEVICE_ADDR, 0x00, read_data, data_length);
        if (status == SOFT_I2C_SUCCESS) {
            success_count++;
        }
    }
    end_time = micros();
    
    uint32_t read_time_us = end_time - start_time;
    float read_throughput = (float)(data_length * test_count * 8) * 1000000.0 / read_time_us;
    
    Serial.printf("\n  %d bytes per transfer (%d tests):\n", data_length, test_count);
    Serial.printf("    Write: %.0f bps (%.1f us/transfer)\n", 
                  write_throughput, (float)write_time_us/test_count);
    Serial.printf("    Read:  %.0f bps (%.1f us/transfer)\n", 
                  read_throughput, (float)read_time_us/test_count);
}

/**
 * 测试连续传输稳定性
 */
void testContinuousTransfer() {
    Serial.println("\n=== Continuous Transfer Stability Test ===");
    
    softWire.setClock(100000);
    
    const int total_transfers = 1000;
    int success_count = 0;
    int error_counts[5] = {0}; // 统计不同错误类型
    
    uint8_t test_data = 0;
    uint32_t start_time = millis();
    
    Serial.printf("Performing %d continuous transfers...\n", total_transfers);
    
    for (int i = 0; i < total_transfers; i++) {
        SoftI2CStatus status = softWire.writeRegister8(TEST_DEVICE_ADDR, 0x01, test_data++);
        
        if (status == SOFT_I2C_SUCCESS) {
            success_count++;
        } else {
            error_counts[status]++;
        }
        
        // 每100次传输显示进度
        if ((i + 1) % 100 == 0) {
            Serial.printf("  Progress: %d/%d (%.1f%% success)\n", 
                         i + 1, total_transfers, (float)success_count/(i+1)*100);
        }
        
        // 短暂延时，模拟实际应用场景
        delay_us(100);
    }
    
    uint32_t total_time_ms = millis() - start_time;
    float transfers_per_second = (float)total_transfers * 1000.0 / total_time_ms;
    
    Serial.printf("\nStability Test Results:\n");
    Serial.printf("  Total transfers: %d\n", total_transfers);
    Serial.printf("  Successful: %d (%.2f%%)\n", success_count, (float)success_count/total_transfers*100);
    Serial.printf("  Total time: %d ms\n", total_time_ms);
    Serial.printf("  Transfer rate: %.1f transfers/second\n", transfers_per_second);
    
    Serial.printf("  Error breakdown:\n");
    Serial.printf("    NACK_ADDR: %d\n", error_counts[SOFT_I2C_NACK_ADDR]);
    Serial.printf("    NACK_DATA: %d\n", error_counts[SOFT_I2C_NACK_DATA]);
    Serial.printf("    TIMEOUT: %d\n", error_counts[SOFT_I2C_TIMEOUT_ERROR]);
    Serial.printf("    BUS_ERROR: %d\n", error_counts[SOFT_I2C_BUS_ERROR]);
} 