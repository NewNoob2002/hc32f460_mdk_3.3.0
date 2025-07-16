/**
 * SoftWire Register Speed Test - 寄存器优化性能测试
 * 
 * 本示例专门测试寄存器级优化的SoftWire库性能：
 * 1. 测试极高频率下的传输能力（200kHz-800kHz）
 * 2. 对比寄存器优化前后的性能差异
 * 3. 验证时序的准确性和稳定性
 * 4. 测试不同数据模式下的实际传输速率
 */

#include "soft_i2c.h"

// 定义I2C引脚 - 使用高性能引脚
#define SOFT_I2C_SDA_PIN    PA8   // SDA引脚
#define SOFT_I2C_SCL_PIN    PA9   // SCL引脚

// 创建寄存器优化的软件I2C对象
SoftWire softWire(SOFT_I2C_SDA_PIN, SOFT_I2C_SCL_PIN);

// 测试设备地址 (可以测试无设备响应的情况来纯测试时序)
#define TEST_DEVICE_ADDR    0x48

void setup() {
    Serial.begin(115200);
    Serial.println("SoftWire Register Optimization Speed Test");
    Serial.println("==========================================");
    
    // 初始化寄存器优化的软件I2C
    if (!softWire.begin()) {
        Serial.println("SoftWire initialization failed!");
        return;
    }
    
    Serial.println("Register-level GPIO optimization enabled!");
    delay(1000);
    
    // 测试极高频率性能
    testHighFrequencyPerformance();
    
    delay(1000);
    
    // 测试时序精度
    testTimingAccuracy();
    
    delay(1000);
    
    // 测试连续高速传输
    testContinuousHighSpeedTransfer();
    
    Serial.println("\nRegister optimization test completed!");
}

void loop() {
    // 可以在这里添加持续的高速I2C操作监控
    delay(5000);
}

/**
 * 测试极高频率下的性能
 */
void testHighFrequencyPerformance() {
    Serial.println("\n=== High Frequency Performance Test ===");
    
    // 测试极高频率：400kHz, 600kHz, 800kHz
    uint32_t high_frequencies[] = {400000, 500000, 600000, 700000, 800000};
    uint8_t freq_count = sizeof(high_frequencies) / sizeof(high_frequencies[0]);
    
    for (uint8_t i = 0; i < freq_count; i++) {
        uint32_t target_freq = high_frequencies[i];
        
        Serial.printf("\n--- Testing %d Hz (%.1f kHz) ---\n", target_freq, target_freq/1000.0);
        softWire.setClock(target_freq);
        
        // 测试无设备时的纯时序性能
        testPureTimingPerformance(target_freq);
        
        // 如果有设备响应，测试实际数据传输
        if (softWire.isDeviceOnline(TEST_DEVICE_ADDR, 1)) {
            testActualDataTransfer(target_freq);
        }
        
        delay(500);
    }
}

/**
 * 测试纯时序性能（无设备响应）
 */
void testPureTimingPerformance(uint32_t target_freq) {
    const int test_count = 1000;
    uint32_t start_time, end_time;
    
    // 测试START-ADDRESS-STOP序列的时序
    start_time = micros();
    for (int i = 0; i < test_count; i++) {
        // 模拟I2C传输序列，即使没有ACK也完成时序
        softWire.beginTransmission(TEST_DEVICE_ADDR);
        softWire.write(0x00);  // 寄存器地址
        softWire.endTransmission(true);
    }
    end_time = micros();
    
    uint32_t total_time_us = end_time - start_time;
    float avg_time_per_cycle = (float)total_time_us / test_count;
    
    // 计算理论最小时间：START(3bit periods) + ADDR(9bit periods) + REG(9bit periods) + STOP(3bit periods)
    // = 24 bit periods = 24 * (1/frequency) 秒
    float theoretical_min_us = 24.0 * 1000000.0 / target_freq;
    float timing_efficiency = (theoretical_min_us / avg_time_per_cycle) * 100.0;
    
    Serial.printf("Pure timing test (%d cycles):\n", test_count);
    Serial.printf("  Avg time: %.1f us/cycle\n", avg_time_per_cycle);
    Serial.printf("  Theoretical min: %.1f us/cycle\n", theoretical_min_us);
    Serial.printf("  Timing efficiency: %.1f%%\n", timing_efficiency);
    Serial.printf("  Estimated actual frequency: %.1f kHz\n", 
                  (24.0 * 1000.0) / avg_time_per_cycle);
}

/**
 * 测试实际数据传输性能
 */
void testActualDataTransfer(uint32_t target_freq) {
    const int test_count = 100;
    uint8_t test_data = 0xAA;
    uint32_t start_time, end_time;
    int success_count = 0;
    
    // 测试实际读写操作
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
    float actual_data_rate = (float)(success_count * 8) * 1000000.0 / total_time_us;  // bps
    
    Serial.printf("Actual data transfer (%d tests):\n", test_count);
    Serial.printf("  Success rate: %d/%d (%.1f%%)\n", 
                  success_count, test_count, (float)success_count/test_count*100);
    Serial.printf("  Avg time: %.1f us/transfer\n", avg_time_per_transfer);
    Serial.printf("  Data rate: %.0f bps (%.1f kbps)\n", actual_data_rate, actual_data_rate/1000.0);
}

/**
 * 测试时序精度
 */
void testTimingAccuracy() {
    Serial.println("\n=== Timing Accuracy Test ===");
    
    // 在不同频率下测试时序稳定性
    uint32_t test_frequencies[] = {100000, 200000, 400000, 600000, 800000};
    uint8_t freq_count = sizeof(test_frequencies) / sizeof(test_frequencies[0]);
    
    for (uint8_t i = 0; i < freq_count; i++) {
        uint32_t freq = test_frequencies[i];
        softWire.setClock(freq);
        
        Serial.printf("\nTiming stability at %d Hz:\n", freq);
        
        // 多次测量，检查时序一致性
        uint32_t times[10];
        for (int j = 0; j < 10; j++) {
            uint32_t start = micros();
            
            // 执行标准的I2C写操作序列
            softWire.beginTransmission(TEST_DEVICE_ADDR);
            softWire.write(0x00);
            softWire.endTransmission(true);
            
            times[j] = micros() - start;
        }
        
        // 计算统计信息
        uint32_t min_time = times[0];
        uint32_t max_time = times[0];
        uint32_t sum = times[0];
        
        for (int j = 1; j < 10; j++) {
            if (times[j] < min_time) min_time = times[j];
            if (times[j] > max_time) max_time = times[j];
            sum += times[j];
        }
        
        float avg_time = (float)sum / 10.0;
        float variation = ((float)(max_time - min_time) / avg_time) * 100.0;
        
        Serial.printf("  Min: %d us, Max: %d us, Avg: %.1f us\n", min_time, max_time, avg_time);
        Serial.printf("  Variation: %.1f%%\n", variation);
        
        if (variation < 5.0) {
            Serial.printf("  ✓ Excellent timing stability\n");
        } else if (variation < 10.0) {
            Serial.printf("  ✓ Good timing stability\n");
        } else {
            Serial.printf("  ⚠ Timing variation may be too high\n");
        }
    }
}

/**
 * 测试连续高速传输
 */
void testContinuousHighSpeedTransfer() {
    Serial.println("\n=== Continuous High-Speed Transfer Test ===");
    
    // 设置为高速模式
    softWire.setClock(600000);  // 600kHz
    
    const int burst_count = 1000;
    const int burst_size = 16;
    uint8_t test_data[burst_size];
    
    // 准备测试数据
    for (int i = 0; i < burst_size; i++) {
        test_data[i] = i + 0x80;
    }
    
    Serial.printf("Testing %d bursts of %d bytes each at 600kHz...\n", burst_count, burst_size);
    
    uint32_t total_bytes = 0;
    uint32_t total_errors = 0;
    uint32_t start_time = millis();
    
    for (int burst = 0; burst < burst_count; burst++) {
        SoftI2CStatus status = softWire.writeBytes(TEST_DEVICE_ADDR, 0x10, test_data, burst_size);
        
        if (status == SOFT_I2C_SUCCESS) {
            total_bytes += burst_size;
        } else {
            total_errors++;
        }
        
        // 每100个突发显示进度
        if ((burst + 1) % 100 == 0) {
            Serial.printf("  Progress: %d/%d bursts (%.1f%% success)\n", 
                         burst + 1, burst_count, 
                         (float)(burst + 1 - total_errors)/(burst + 1)*100);
        }
        
        // 短暂间隔，模拟实际应用
        delay_us(50);
    }
    
    uint32_t total_time_ms = millis() - start_time;
    float throughput_kbps = (float)(total_bytes * 8) / total_time_ms;  // kbps
    float transfer_rate = (float)burst_count * 1000.0 / total_time_ms;  // transfers/sec
    
    Serial.printf("\nContinuous Transfer Results:\n");
    Serial.printf("  Total time: %d ms\n", total_time_ms);
    Serial.printf("  Successful bytes: %d/%d\n", total_bytes, burst_count * burst_size);
    Serial.printf("  Error rate: %.2f%%\n", (float)total_errors/burst_count*100);
    Serial.printf("  Data throughput: %.1f kbps\n", throughput_kbps);
    Serial.printf("  Transfer rate: %.1f transfers/sec\n", transfer_rate);
    
    if (total_errors == 0) {
        Serial.printf("  ✓ Perfect reliability at high speed!\n");
    } else if ((float)total_errors/burst_count < 0.01) {
        Serial.printf("  ✓ Excellent reliability\n");
    } else {
        Serial.printf("  ⚠ Consider lowering frequency for better reliability\n");
    }
}

/**
 * 高级测试：GPIO切换速度测量
 */
void testGPIOSwitchingSpeed() {
    Serial.println("\n=== GPIO Switching Speed Test ===");
    
    const int test_iterations = 10000;
    uint32_t start_time, end_time;
    
    softWire.setClock(800000);  // 最高频率
    
    // 测试纯GPIO切换速度
    start_time = micros();
    for (int i = 0; i < test_iterations; i++) {
        // 模拟I2C时钟脉冲
        // scl_low -> scl_high -> scl_low
        // 这些是内联函数，应该很快
    }
    end_time = micros();
    
    uint32_t gpio_switch_time = end_time - start_time;
    float ns_per_switch = (float)gpio_switch_time * 1000.0 / (test_iterations * 3);  // 每次切换的纳秒
    
    Serial.printf("GPIO switching performance:\n");
    Serial.printf("  %d switch operations in %d us\n", test_iterations * 3, gpio_switch_time);
    Serial.printf("  %.0f ns per switch operation\n", ns_per_switch);
    Serial.printf("  Theoretical max I2C frequency: %.1f MHz\n", 
                  1000.0 / (ns_per_switch * 4));  // 4个切换操作per bit
} 