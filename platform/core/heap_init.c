#include "heap_init.h"
#include "lwmem/lwmem.h"
#include "core_debug.h"

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
        CORE_DEBUG_PRINTF("Cannot initialize LwMEM. Make sure your regions are not overlapping each other and are in ascending memory order\r\n");
    } else {
        CORE_DEBUG_PRINTF("LwMEM initialized and ready to use\r\n");
    }
}

void *my_malloc(size_t size)
{
    void *ptr = lwmem_malloc(size);
    if (ptr == NULL) {
        CORE_DEBUG_PRINTF("Memory allocation failed for size %zu\r\n", size);
				return NULL;
    }
    return ptr;
}

void my_free(void *ptr)
{
    if (ptr == NULL) {
        CORE_DEBUG_PRINTF("Attempted to free a NULL pointer\r\n");
        return;
    }
    lwmem_free(ptr);
}