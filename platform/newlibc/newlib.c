/**
 * stubs for compatibility with newlib.
 * as per https://sourceware.org/newlib/libc.html#Stubs
 */

#include <sys/stat.h>
#include <errno.h>
#include <stddef.h>

__asm (".global __use_no_semihosting\n\t");
// External heap symbols from linker script
extern char _end;
extern char _estack;
static char *heap_end = &_end;

__attribute__((weak)) int _close(int file)
{
  // For embedded systems, we typically only have stdin, stdout, stderr
  // File descriptors 0, 1, 2 are always "open"
  if (file < 0) {
    errno = EBADF;
    return -1;
  }
  
  if (file >= 0 && file <= 2) {
    // stdin, stdout, stderr - always successful to "close"
    return 0;
  }
  
  // For other file descriptors, return error (not implemented)
  errno = EBADF;
  return -1;
}
 
__attribute__((weak)) void *_sbrk(int incr)
{
  char *prev_heap_end;
  
  // Initialize heap_end on first call
  if (heap_end == 0) {
    heap_end = &_end;
  }
  
  prev_heap_end = heap_end;
  
  // Check for heap-stack collision
  // Use a simple heuristic: ensure we don't exceed the stack area
  if (heap_end + incr > &_estack) {
    errno = ENOMEM;
    return (void *)-1;
  }
  
  // Update heap pointer
  heap_end += incr;
  
  return (void *)prev_heap_end;
}

 __attribute__((weak)) int _fstat(int file, struct stat *st)
{
  // Check for valid file descriptor
  if (file < 0) {
    errno = EBADF;
    return -1;
  }
  
  if (!st) {
    errno = EFAULT;
    return -1;
  }
  
  // For embedded systems, treat all valid file descriptors as character devices
  if (file >= 0 && file <= 2) {
    // stdin, stdout, stderr are character devices
    st->st_mode = S_IFCHR;
    st->st_size = 0;
    st->st_blksize = 1024;
    st->st_blocks = 0;
    return 0;
  }
  
  // For other file descriptors, return error
  errno = EBADF;
  return -1;
}
 
 __attribute__((weak)) int _getpid(void)
 {
   return 1;
 }
 
 __attribute__((weak)) int _isatty(int file)
{
  // Check for valid file descriptor
  if (file < 0) {
    errno = EBADF;
    return 0;
  }
  
  // For embedded systems, stdin, stdout, stderr are typically connected to UART/serial
  // which can be considered as "tty-like"
  if (file >= 0 && file <= 2) {
    return 1; // stdin, stdout, stderr are tty
  }
  
  // Other file descriptors are not tty
  return 0;
}
 
 #include <errno.h>
 #undef errno
 extern int errno;
 __attribute__((weak)) int _kill(int pid, int sig)
 {
   errno = EINVAL;
   return -1;
 }
 
 __attribute__((weak)) int _lseek(int file, int ptr, int dir)
{
  // Check for valid file descriptor
  if (file < 0) {
    errno = EBADF;
    return -1;
  }
  
  // For embedded systems, stdin, stdout, stderr don't support seeking
  if (file >= 0 && file <= 2) {
    errno = ESPIPE; // Illegal seek - these are stream-like devices
    return -1;
  }
  
  // For other file descriptors, return error (not implemented)
  errno = EBADF;
  return -1;
}
 
 __attribute__((weak)) int _read(int file, char *ptr, int len)
{
  // Check for valid file descriptor
  if (file < 0) {
    errno = EBADF;
    return -1;
  }
  
  if (!ptr) {
    errno = EFAULT;
    return -1;
  }
  
  if (len < 0) {
    errno = EINVAL;
    return -1;
  }
  
  if (len == 0) {
    return 0;
  }
  
  // Handle stdin (file descriptor 0)
  if (file == 0) {
    // For embedded systems, stdin reading is typically not implemented
    // or would require UART RX functionality
    // For now, return 0 (EOF) to indicate no data available
    // TODO: Implement actual UART RX if needed
    return 0;
  }
  
  // stdout and stderr don't support reading
  if (file == 1 || file == 2) {
    errno = EBADF; // Bad file descriptor for read operation
    return -1;
  }
  
  // For other file descriptors, return error (not implemented)
  errno = EBADF;
  return -1;
}
 
 #include "x_printf.h"

__attribute__((weak)) int _write(int file, char *ptr, int len)
{
  // Check for valid file descriptor
  if (file < 0) {
    errno = EBADF;
    return -1;
  }
  
  if (!ptr) {
    errno = EFAULT;
    return -1;
  }
  
  if (len < 0) {
    errno = EINVAL;
    return -1;
  }
  
  if (len == 0) {
    return 0;
  }
  
  // Handle stdout and stderr (file descriptors 1 and 2)
  if (file == 1 || file == 2) {
    int todo;
    for (todo = 0; todo < len; todo++) {
      x_putchar(*ptr++);
    }
    return len;
  }
  
  // stdin doesn't support writing
  if (file == 0) {
    errno = EBADF; // Bad file descriptor for write operation
    return -1;
  }
  
  // For other file descriptors, return error (not implemented)
  errno = EBADF;
  return -1;
}

__attribute__((weak)) void _exit(int status)
{
  // For embedded systems, we typically can't actually exit
  // Instead, we enter an infinite loop or reset the system
  
  // Disable interrupts to prevent further execution
  __asm volatile ("cpsid i");
  
  // Enter infinite loop
  while (1) {
    // Optionally, you could implement a system reset here
    // For now, just loop forever
    __asm volatile ("nop");
  }
}

__attribute__((weak)) void _init(void)
{
  // Initialization function
  // For most embedded applications, this can be empty
  // Any global constructor initialization is handled elsewhere
  
  // Add any system-specific initialization here if needed
  // For now, this is a no-op
}
 