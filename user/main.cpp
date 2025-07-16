/*******************************************************************************
 * Include files
 ******************************************************************************/
#include <Arduino.h>
#include <hc32_ll.h>
// #include "FreeRTOS.h"
// #include "task.h"

/* unlock/lock peripheral */
#define EXAMPLE_PERIPH_WE (LL_PERIPH_GPIO | LL_PERIPH_EFM | LL_PERIPH_FCG | \
						   LL_PERIPH_PWC_CLK_RMU | LL_PERIPH_SRAM)
#define EXAMPLE_PERIPH_WP (LL_PERIPH_EFM | LL_PERIPH_FCG | LL_PERIPH_SRAM)

static uint32_t tick_led = 0;
static uint32_t tick1_led = 0;
static uint32_t tick2_led = 0;;

int32_t main(void)
{
	/* Register write enable for some required peripherals. */
	LL_PERIPH_WE(EXAMPLE_PERIPH_WE);
	WRITE_REG16(CM_GPIO->PSPCR, 0x03);
	Clock_Init();
	delay_init();
	Serial.begin(115200);

	pinMode(PA0, PWM);

	// WDT.begin(10 * 1000);
	printf("Hello, world\n");
	printf("float: %f\n", 1.234);
	static uint32_t i = 0;
	while (1)
	{
		if (millis() - tick_led >= 100)
		{
			tick_led = millis();
		}
	}
}
