/*******************************************************************************
 * Include files
 ******************************************************************************/
#include <Arduino.h>
#include <hc32_ll.h>
#include <math.h>  // 用于sin函数
// #include "FreeRTOS.h"
// #include "task.h"

/* unlock/lock peripheral */
#define EXAMPLE_PERIPH_WE (LL_PERIPH_GPIO | LL_PERIPH_EFM | LL_PERIPH_FCG | \
						   LL_PERIPH_PWC_CLK_RMU | LL_PERIPH_SRAM)
#define EXAMPLE_PERIPH_WP (LL_PERIPH_EFM | LL_PERIPH_FCG | LL_PERIPH_SRAM)

static uint32_t tick_led = 0;
static uint32_t tick1_led = 0;
static uint32_t tick2_led = 0;

// 呼吸灯参数
static float breath_angle = 0.0;          // 当前角度
static const float breath_speed = 0.02;   // 呼吸速度，可调节
static const uint32_t breath_update_interval = 10; // 更新间隔(ms)，值越小越丝滑

int32_t main(void)
{
	/* Register write enable for some required peripherals. */
	LL_PERIPH_WE(EXAMPLE_PERIPH_WE);
	WRITE_REG16(CM_GPIO->PSPCR, 0x03);
	clock_init();
	xtal32_init();
	delay_init();
	Serial.begin(115200);

	// pinMode(PA0, PWM);
	pinMode(PA1, PWM);

	// 设置PWM分辨率为10位(0-1023)，提供更平滑的控制
	analogWriteResolution(10);

	// WDT.begin(10 * 1000);
	
	static uint32_t i = 0;
	while (1)
	{
		// 呼吸灯更新
		if (millis() - tick_led >= breath_update_interval)
		{
			tick_led = millis();
			
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
	}
}
