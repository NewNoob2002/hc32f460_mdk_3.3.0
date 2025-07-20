/*******************************************************************************
 * Include files
 ******************************************************************************/
#include <Arduino.h>
#include <SparkFun_Extensible_Message_Parser.h>
#include <FreeRTOS.h>
#include "task.h"

#include "HardwareI2cSlave.h"
/*******************************************************************************
 * Macro definitions
 ******************************************************************************/
#define EXAMPLE_PERIPH_WE (LL_PERIPH_GPIO | LL_PERIPH_EFM | LL_PERIPH_FCG | \
                           LL_PERIPH_PWC_CLK_RMU | LL_PERIPH_SRAM)
#define EXAMPLE_PERIPH_WP (LL_PERIPH_EFM | LL_PERIPH_FCG | LL_PERIPH_SRAM)

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
// 呼吸灯参数
static float breath_angle                    = 0.0;  // 当前角度
static const float breath_speed              = 0.05; // 呼吸速度，可调节
static const uint32_t breath_update_interval = 10;   // 更新间隔(ms)，值越小越丝滑

TaskHandle_t test_task_handle    = nullptr;
TaskHandle_t WatchDog_TaskHandle = nullptr;
// 呼吸灯任务
// 该任务会周期性更新呼吸灯的PWM值
static void test_task(void *e)
{
    while (1) {
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

        vTaskDelay(10); // 延时10ms，避免任务过于频繁
    }
}

static void customParserCallback(SEMP_PARSE_STATE *parse, uint16_t type)
{
    // 处理自定义解析器的回调
    SEMP_CUSTOM_HEADER *messageHeader = nullptr;
    messageHeader                     = (SEMP_CUSTOM_HEADER *)parse->buffer;
    uint16_t messageId                = messageHeader->messageId;
    uint8_t messageType               = messageHeader->messageType;

    printf("Custom Parser Callback: Message ID: %u, Message Type: %u\n", messageId, messageType);
}
void parser_prinf_callback(const char *format, ...)
{
    va_list args;
    va_start(args, format);
    vprintf(format, args);
    va_end(args);
};
static void i2cSlave_task(void *e)
{
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
        printf("Failed to initialize the Bt parser");
    while (true) {
				i2c_slave_receive_int(&i2c_handle_t, 256, 3000);
        // 检查是否有新的数据到来
        if (I2C_getcount_rxbuffer() > 0) {
            uint8_t data[256];
            size_t len = I2C_read_rxbuffer(data, sizeof(data));
            printf("Received %zu bytes: ", len);
        }
        vTaskDelay(10); // 每秒检查一次
    }
}

static void WatchDog_Task(void *e)
{
    while (true) {
        WDT.reload();
        vTaskDelay(1000);
    }
}

extern void i2c_buffer_init();

int32_t main(void)
{
    /* Register write enable for some required peripherals. */
    LL_PERIPH_WE(EXAMPLE_PERIPH_WE);
    WRITE_REG16(CM_GPIO->PSPCR, 0x03);
    clock_init();
    heap_init();
    Serial.begin(115200);
    // cm_backtrace_init("HC32F460", "1.0.0", "1.0.0");
    delay_init();
    // pinMode(PA0, PWM);
    pinMode(PA1, PWM);

    if(i2cSlave_init() == LL_OK) {
        printf("I2C slave initialized successfully.\n");
    } else {
        printf("Failed to initialize I2C slave.\n");
    }
    xTaskCreate(test_task, "Breath LED Task", 1024, nullptr, 1, &test_task_handle);
    xTaskCreate(i2cSlave_task, "I2C Slave Task", 1024 * 3, nullptr, 2, nullptr);
    // 启动调度器
    vTaskStartScheduler();
    while (true) {}
}
