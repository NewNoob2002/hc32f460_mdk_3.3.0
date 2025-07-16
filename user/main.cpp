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

	pinMode(PA0, OUTPUT);
	pinMode(PA1, OUTPUT);
	pinMode(PA2, OUTPUT);
	Serial.begin(115200);

	// WDT.begin(10 * 1000);
	printf("Hello, world\n");
	printf("float: %f\n", 1.234);

	while (1)
	{
		if (millis() - tick_led >= 1000)
		{
			tick_led = millis();
			digitalToggle(PA1);
		}
		if (millis() - tick1_led >= 150)
		{
			tick1_led = millis();
			digitalToggle(PA0);
		}
		if (millis() - tick2_led >= 200)
		{
			tick2_led = millis();
			digitalToggle(PA2);
		}
	}
}
