/*******************************************************************************
 * Include files
 ******************************************************************************/
#include <Arduino.h>
#include <SparkFun_Extensible_Message_Parser.h>

#include "HardwareI2cSlave.h"

SEMP_PARSE_ROUTINE const customParserTable[] = {
    sempCustomPreamble, // Custom parser preamble
};
const int customParserCount = sizeof(customParserTable) / sizeof(customParserTable[0]);

const char *const customParserNames[] = {
    "CustomAPP",
};
const int customParserNameCount = sizeof(customParserNames) / sizeof(customParserNames[0]);
SEMP_PARSE_STATE *custom_parser = nullptr;

const uint8_t messageHeaderLength = sizeof(SEMP_CUSTOM_HEADER);

static bool i2c_receive_flag = true;
// 呼吸灯参数
static float breath_angle                    = 0.0;  // 当前角度
static const float breath_speed              = 0.05; // 呼吸速度，可调节
static const uint32_t breath_update_interval = 10;   // 更新间隔(ms)，值越小越丝滑

// 呼吸灯任务
// 该任务会周期性更新呼吸灯的PWM值
static void breath_task()
{
    // 使用正弦函数生成平滑的呼吸效果
    // sin值范围[-1, 1]，转换为[0, 1023]
    float sin_val = sin(breath_angle);
    // 将[-1,1]映射到[0,1023]，使用(sin+1)/2确保值为正
    uint32_t pwm_value = (uint32_t)((sin_val + 1.0) / 2.0 * 1023);

    // 输出PWM
    // analogWrite(PA0, pwm_value);
    analogWrite(PA1, pwm_value);
    // 更新角度
    breath_angle += breath_speed;
    // 防止角度过大，重置到0-2π范围
    if (breath_angle >= 2 * PI) {
        breath_angle = 0.0;
    }
}

uint8_t i2c_TxBuffer[512];

static void customParserCallback(SEMP_PARSE_STATE *parse, uint16_t type)
{
    // 处理自定义解析器的回调
    SEMP_CUSTOM_HEADER *messageHeader = (SEMP_CUSTOM_HEADER *)parse->buffer;
    uint16_t messageId                = messageHeader->messageId;
    uint8_t messageType               = messageHeader->messageType;

    for (int i = 0; i < parse->length; i++) {
        CORE_DEBUG_PRINTF("%02x ", parse->buffer[i]);
    }
    CORE_DEBUG_PRINTF("\n");
    switch (messageId) {
        case 1: {
            uint8_t TXbuffer[] = {0xaa, 0x44, 0x18, 0x14, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x24, 0x00, 0x00, 0x00, 0x03, 0x01, 0x00, 0x00, 0x56, 0x30, 0x2e, 0x31, 0x00, 0x00, 0x00, 0x00, 0x56, 0x31, 0x2e, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x38, 0x00, 0x00, 0x00,
                                  0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe9, 0x0d, 0x07, 0xe2};
            memcpy(i2c_TxBuffer, TXbuffer, sizeof(TXbuffer));
            i2c_slave_transmit_int(&i2c_handle_t, i2c_TxBuffer, sizeof(TXbuffer), 3000);
            break;
        }
        case 13: {
            uint8_t TXbuffer[] = {0xAA, 0x44, 0x18};
            i2c_slave_transmit_int(&i2c_handle_t, TXbuffer, sizeof(TXbuffer), 3000);
            break;
        }
    }
}
void parser_prinf_callback(const char *format, ...)
{
    va_list args;
    va_start(args, format);
    vprintf(format, args);
    va_end(args);
};

int32_t main(void)
{
    /* Register write enable for some required peripherals. */
    LL_PERIPH_WE(EXAMPLE_PERIPH_WE);
    WRITE_REG16(CM_GPIO->PSPCR, 0x03);
    clock_init();
    heap_init();
    Serial.begin(115200);
    delay_init();
    pinMode(PA0, OUTPUT);
    Wire.begin();
    if (Wire.isDeviceOnline(0x0B)) {
        CORE_DEBUG_PRINTF("found 0x0b\n");
    }
    // i2cSlave_init();
    // memset(i2c_TxBuffer, 0, sizeof(i2c_TxBuffer));
    // custom_parser = sempBeginParser(customParserTable,
    //                                 customParserCount,
    //                                 customParserNames,
    //                                 customParserNameCount,
    //                                 0,
    //                                 1024 * 3,
    //                                 customParserCallback,
    //                                 "BluetoothDebug",
    //                                 parser_prinf_callback,
    //                                 parser_prinf_callback);
    // if (!custom_parser)
    //     CORE_DEBUG_PRINTF("Failed to initialize the Bt parser");
    // Task Create
    while (true) {
        digitalToggle(PA0);
        if (Wire.isDeviceOnline(0x0B)) {
            CORE_DEBUG_PRINTF("found 0x0b\n");
        }
        //        i2c_slave_receive_int(&i2c_handle_t, 3000);
        //        // 检查是否有新的数据到来
        //        if (i2c_getcount_rxbuffer() > 0) {
        //            uint8_t data[256];
        //            size_t len = i2c_read_rxbuffer(data, i2c_getcount_rxbuffer());
        //            for (int i = 0; i < len; i++) {
        //                sempParseNextByte(custom_parser, data[i]);
        //            }
        //        }
        delay_ms(1000);
    }
}
