#include "usart_sync.h"
// #include <drivers/usart/addon_usart.h>

// USART_TypeDef to gpio function select mapping
#define USART_DEV_TO_TX_FUNC(usart)       \
    usart == CM_USART1   ? Func_Usart1_Tx \
    : usart == CM_USART2 ? Func_Usart2_Tx \
    : usart == CM_USART3 ? Func_Usart3_Tx \
                         : Func_Usart4_Tx

// USART_TypeDef to PWC_FCG1_PERIPH_USARTx mapping
#define USART_DEV_TO_PERIPH_CLOCK(usart)          \
    usart == CM_USART1   ? PWC_FCG1_USART1 \
    : usart == CM_USART2 ? PWC_FCG1_USART2 \
    : usart == CM_USART3 ? PWC_FCG1_USART3 \
                         : PWC_FCG1_USART4

void usart_sync_init(CM_USART_TypeDef *usart, const gpio_pin_t tx_pin, const stc_usart_uart_init_t *config)
{
    // disable and de-init usart peripheral
    USART_FuncCmd(usart, USART_RX, DISABLE);
    USART_FuncCmd(usart, USART_INT_RX, DISABLE);
    USART_FuncCmd(usart, USART_TX, DISABLE);
    // USART_FuncCmd(usart, USART_INT_TX_EMPTY, DISABLE);
    // USART_FuncCmd(usart, USART_INT_TX_CPLT, DISABLE);
    USART_DeInit(usart);

    // set tx pin function to USART TX output
    GPIO_SetFunction(tx_pin, USART_DEV_TO_TX_FUNC(usart));

    // enable USART clock
    FCG_Fcg1PeriphClockCmd(USART_DEV_TO_PERIPH_CLOCK(usart), ENABLE);

    // initialize USART peripheral and set baudrate
    USART_UART_Init(usart, config, NULL);
}

void usart_sync_putc(CM_USART_TypeDef *usart, const char ch)
{
    // enable TX function
    USART_FuncCmd(usart, USART_TX, ENABLE);

    // wait until TX buffer is empty
    while (USART_GetStatus(usart, USART_FLAG_TX_EMPTY) == RESET)
        ;

    // write char to TX buffer
    USART_WriteData(usart, ch);
}

void usart_sync_write(CM_USART_TypeDef *usart, const char *str)
{
    // enable TX function
    USART_FuncCmd(usart, USART_TX, ENABLE);

    // print the string
    for (size_t i = 0; str[i] != '\0'; i++)
    {
        usart_sync_putc(usart, str[i]);
    }
}
