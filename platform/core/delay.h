#ifndef DELAY_h
#define DELAY_h

#include <hc32_ll.h>

#ifdef __cplusplus
extern "C" {
#endif

void delay_init();

void delay_ms(uint32_t ms);

void delay_us(uint64_t us);

uint32_t millis();
uint64_t micros();

#ifdef __cplusplus
}
#endif
#endif