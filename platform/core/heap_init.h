#ifndef HEAP_INIT_H
#define HEAP_INIT_H

#include <stdio.h>

#ifdef __cplusplus
extern "C"
{
#endif
void heap_init();
void *my_malloc(size_t size);

void my_free(void *ptr);
#ifdef __cplusplus
}
#endif

#endif