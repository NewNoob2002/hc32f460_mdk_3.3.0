#pragma once
#include "panic_api.h"
#include <stdlib.h>

#ifndef STRINGIFY
#define STRINGIFY_DETAIL(x) #x
#define STRINGIFY(x) STRINGIFY_DETAIL(x)
#endif

#define PANIC_ENABLE 1
#define HANG_ON_PANIC
#define PANIC_USART1_TX_PIN PA15
// determine if at least one panic output is defined
#ifdef PANIC_USART1_TX_PIN
  #define PANIC_USART1_AVAILABLE 1
#else
  #define PANIC_USART1_AVAILABLE 0
#endif

#ifdef PANIC_USART2_TX_PIN
  #define PANIC_USART2_AVAILABLE 1
#else
  #define PANIC_USART2_AVAILABLE 0
#endif

#ifdef PANIC_USART3_TX_PIN
  #define PANIC_USART3_AVAILABLE 1
#else
  #define PANIC_USART3_AVAILABLE 0
#endif

#ifdef PANIC_USART4_TX_PIN
  #define PANIC_USART4_AVAILABLE 1
#else
  #define PANIC_USART4_AVAILABLE 0
#endif

#define PANIC_OUTPUT_AVAILABLE (PANIC_USART1_AVAILABLE || PANIC_USART2_AVAILABLE || PANIC_USART3_AVAILABLE || PANIC_USART4_AVAILABLE)

#ifdef PANIC_ENABLE
#define ENABLE_PANIC_ENABLE 1
#else
#define ENABLE_PANIC_ENABLE 0
#endif

// determine if panic handler should be active:
// - either manually enabled
// - or panic output is available
#if (PANIC_OUTPUT_AVAILABLE && ENABLE_PANIC_ENABLE)
#define ENABLE_PANIC_HANDLER 1
#else
#define ENABLE_PANIC_HANDLER 0
#endif

#if ENABLE_PANIC_HANDLER

  // control printf buffer size
  #ifndef PANIC_PRINTF_BUFFER_SIZE
    #define PANIC_PRINTF_BUFFER_SIZE 256
  #endif

  // panic usart baud rate
  #ifndef PANIC_USART_BAUDRATE
    #define PANIC_USART_BAUDRATE 115200
  #endif

  #ifdef __cplusplus
extern "C"
{
  #endif
  /**
   * @brief print message to panic output, if enabled
   * @param fmt format string
   * @param ... format arguments
   * @return number of characters printed
   */
  size_t panic_printf(const char *fmt, ...);

  /**
   * @brief internal panic handler
   * @param message message to print before panicing. may be set to NULL to omit
   * message
   */
  void _panic(const char *message);

  #ifdef __cplusplus
}
  #endif

  // define file name macro for panic() to use
  #if defined(__PANIC_SHORT_FILENAMES) && defined(__SOURCE_FILE_NAME__)
    #define PANIC_FILE_NAME __SOURCE_FILE_NAME__ // only filename
  #else
    // no short filenames, or __SOURCE_FILE_NAME__ not available
    #if defined(__PANIC_SHORT_FILENAMES)
      #warning "__PANIC_SHORT_FILENAMES is defined, but __SOURCE_FILE_NAME__ is not available."
    #endif

    #define PANIC_FILE_NAME __FILE__
  #endif

  #define PANIC_LINE_NUMBER_STR STRINGIFY(__LINE__)

  /**
   * @brief core panic handler
   * @param message message to print before panicing. use a empty string to omit
   * @note automatically adds file and line number to message
   */
  #ifdef __OMIT_PANIC_MESSAGE
    #define panic(msg) _panic(PANIC_FILE_NAME "l" PANIC_LINE_NUMBER_STR)
  #else
    #define panic(msg) _panic("[" PANIC_FILE_NAME " l" PANIC_LINE_NUMBER_STR "]" msg)
  #endif
#else // !ENABLE_PANIC_HANDLER
  #define panic_begin()
  #define panic_end()
  #define panic_printf(fmt, ...)
  #define panic(msg)
#endif // ENABLE_PANIC_HANDLER
