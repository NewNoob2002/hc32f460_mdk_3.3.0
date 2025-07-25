#include <hc32_ll.h>
#include "panic.h"

void DDL_AssertHandler(const char *file, int line)
{
    panic_begin();
    panic_printf("DDL assert failure in file %s, line %d\n", file, line);
    panic_end();
}
