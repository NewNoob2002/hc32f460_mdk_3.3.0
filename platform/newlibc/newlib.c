/**
 * stubs for compatibility with newlib.
 * as per https://sourceware.org/newlib/libc.html#Stubs
 */
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

int usart1_write(uint8_t *ch, int len);

#ifdef __cplusplus
}
#endif

 