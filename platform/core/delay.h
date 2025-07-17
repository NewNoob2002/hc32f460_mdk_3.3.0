#ifndef DELAY_h
#define DELAY_h

#include <hc32_ll.h>

#ifdef __cplusplus
extern "C" {
#endif
	
#ifndef USE_FREERTOS

void delay_init();

void delay_ms(uint32_t ms);

void delay_us(uint32_t us);

uint32_t millis();
uint32_t micros();
	
void clock_init();

int32_t xtal32_init();

#endif

#ifdef __cplusplus
}
#endif
#endif