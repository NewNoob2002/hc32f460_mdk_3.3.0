#include <stdio.h>
#include "heap_init.h"
#include "lwmem/lwmem.h"

uint8_t lw_heap[16* 1024];

static lwmem_region_t
regions[] = {
    { lw_heap, sizeof(lw_heap) },
    /* Add more regions if needed */
    { NULL, 0 }
};

void heap_init()
{
	  if (!lwmem_assignmem(regions)) {
        printf("Cannot initialize LwMEM. Make sure your regions are not overlapping each other and are in ascending memory order\r\n");
        while (1) {}
    } else {
        printf("LwMEM initialized and ready to use\r\n");
    }
}