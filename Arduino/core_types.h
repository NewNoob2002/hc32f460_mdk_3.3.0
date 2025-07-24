#ifndef _TYPES_H_
#define _TYPES_H_
#include <stdbool.h>
#include <stdint.h>
#include <stdarg.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef bool boolean;
typedef uint8_t byte;
typedef uint16_t word;

typedef void (*voidFuncPtr)(void);
typedef void (*voidFuncPtrWithArg)(void *arg);

typedef int16_t gpio_pin_t;

#endif // _TYPES_H_
