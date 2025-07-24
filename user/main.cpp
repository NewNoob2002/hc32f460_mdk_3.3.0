/*******************************************************************************
 * Include files
 ******************************************************************************/
#include <Arduino.h>
#include "MillisTaskManager.h"
#include "delay.h"
#include <SparkFun_Extensible_Message_Parser.h>

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

static MillisTaskManager taskManager; // 创建任务管理器实例，开启优先级
// 呼吸灯任务
// 该任务会周期性更新呼吸灯的PWM值
// static void breath_task()
// {
//     // 使用正弦函数生成平滑的呼吸效果
//     // sin值范围[-1, 1]，转换为[0, 1023]
//     float sin_val = sin(breath_angle);
//     // 将[-1,1]映射到[0,1023]，使用(sin+1)/2确保值为正
//     uint32_t pwm_value = (uint32_t)((sin_val + 1.0) / 2.0 * 1023);

//     // 输出PWM
//     // analogWrite(PA0, pwm_value);
//     analogWrite(PA1, pwm_value);
//     // 更新角度
//     breath_angle += breath_speed;
//     // 防止角度过大，重置到0-2π范围
//     if (breath_angle >= 2 * PI) {
//         breath_angle = 0.0;
//     }
// }

uint8_t i2c_TxBuffer[512];

static void customParserCallback(SEMP_PARSE_STATE *parse, uint16_t type)
{
    Wire_Slave.slave_change_mode(SLAVE_MODE_TX);
    // 处理自定义解析器的回调
    SEMP_CUSTOM_HEADER *messageHeader = (SEMP_CUSTOM_HEADER *)parse->buffer;
    uint16_t messageId                = messageHeader->messageId;
    uint8_t messageType               = messageHeader->messageType;
    switch (messageId) {
        case 1: {
            CORE_DEBUG_PRINTF("messageId:%d messageType:%d\n", messageId, messageType);
            uint8_t TXbuffer[] = {0xaa, 0x44, 0x18, 0x14, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x24, 0x00, 0x00, 0x00, 0x03, 0x01, 0x00, 0x00, 0x56, 0x30, 0x2e, 0x31, 0x00, 0x00, 0x00, 0x00, 0x56, 0x31, 0x2e, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x38, 0x00, 0x00, 0x00,
                                  0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe9, 0x0d, 0x07, 0xe2};
            memcpy(i2c_TxBuffer, TXbuffer, sizeof(TXbuffer));
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

void my_callback(void *address)
{
    CORE_DEBUG_PRINTF("Device %02x is online\n", *((uint8_t *)address));
}

static void loop_task()
{
    digitalToggle(PA0);
    Wire.scanDeivces(my_callback);
}

static void i2c_slave_task()
{
    if (Wire_Slave.slave_address_match()) {
        switch (Wire_Slave.get_slave_work_mode()) {
            case SLAVE_MODE_RX: {
                size_t len = Wire_Slave.slave_receive();
                if (len > 0) {
                    uint8_t i2c_RxBuffer[60];
                    size_t read_len = Wire_Slave.read(i2c_RxBuffer, len);
                    for (int i = 0; i < read_len; i++) {
                        sempParseNextByte(custom_parser, i2c_RxBuffer[i]);
                    }
                }
                break;
            }
            case SLAVE_MODE_TX: {
                size_t send_len = Wire_Slave.slave_transmit(i2c_TxBuffer);
                Wire_Slave.slave_change_mode(SLAVE_MODE_RX);
                break;
            }
        }
    }
}
int32_t main(void)
{
    /* Register write enable for some required peripherals. */
    LL_PERIPH_WE(EXAMPLE_PERIPH_WE);
    WRITE_REG16(CM_GPIO->PSPCR, 0x03);
    clock_init();
    delay_init();
    Serial.begin(115200);
    heap_init();
    pinMode(PA0, OUTPUT);
    Wire.begin();

    Wire_Slave.setSlaveAddress(0x11);
    Wire_Slave.begin(400 * 1000);
    memset(i2c_TxBuffer, 0xff, sizeof(i2c_TxBuffer));
    custom_parser = sempBeginParser(customParserTable,
                                    customParserCount,
                                    customParserNames,
                                    customParserNameCount,
                                    0,
                                    1024 * 3,
                                    customParserCallback,
                                    "BluetoothDebug",
                                    parser_prinf_callback,
                                    parser_prinf_callback);
    if (!custom_parser)
        CORE_DEBUG_PRINTF("Failed to initialize the Bt parser");
    // Task Create
    taskManager.Register(loop_task, 1000);
    while (true) {
        taskManager.Running(millis());
        i2c_slave_task();
    }
}
