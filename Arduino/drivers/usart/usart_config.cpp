#include "usart_handlers.h"
#include <WVariant.h>
#include "../gpio/gpio.h"

usart_config_t *USARTx[USART_COUNT] = {
    &USART1_config,
    &USART2_config,
    &USART3_config,
    &USART4_config,
};
//
// USART configurations
//
usart_config_t USART1_config = {
    .peripheral = {
        .register_base = CM_USART1,
        .clock_id = PWC_FCG1_USART1,
        .tx_pin_function = GPIO_FUNC_32,
        .rx_pin_function = GPIO_FUNC_33,
    },
    .interrupts = {
        .rx_data_full = {
            .interrupt_source = INT_SRC_USART1_RI,
            .interrupt_handler = USARTx_rx_data_available_irq<1>,
        },        
        .rx_error = {
            .interrupt_source = INT_SRC_USART1_EI,
            .interrupt_handler = USARTx_rx_error_irq<1>,
        },
        .tx_buffer_empty = {
            .interrupt_source = INT_SRC_USART1_TI,
            .interrupt_handler = USARTx_tx_buffer_empty_irq<1>,
        },
        .tx_complete = {
            .interrupt_source = INT_SRC_USART1_TCI,
            .interrupt_handler = USARTx_tx_complete_irq<1>,
        },
    },
    .state = {
        .rx_error = usart_receive_error_t::None,
    },
    #ifdef USART_RX_DMA_SUPPORT
    .dma = {
        .rx_data_available_event = EVT_USART1_RI,
        .rx_data_available_dma_btc = {
            .interrupt_handler = USARTx_rx_da_dma_btc_irq<1>,
        },
    }
    #endif
};

usart_config_t USART2_config = {
    .peripheral = {
        .register_base = CM_USART2,
        .clock_id = PWC_FCG1_USART2,
        .tx_pin_function = GPIO_FUNC_36,
        .rx_pin_function = GPIO_FUNC_37,
    },
    .interrupts = {
        .rx_data_full = {
            .interrupt_source = INT_SRC_USART2_RI,
            .interrupt_handler = USARTx_rx_data_available_irq<2>,
        },
        .rx_error = {
            .interrupt_source = INT_SRC_USART2_EI,
            .interrupt_handler = USARTx_rx_error_irq<2>,
        },
        .tx_buffer_empty = {
            .interrupt_source = INT_SRC_USART2_TI,
            .interrupt_handler = USARTx_tx_buffer_empty_irq<2>,
        },
        .tx_complete = {
            .interrupt_source = INT_SRC_USART2_TCI,
            .interrupt_handler = USARTx_tx_complete_irq<2>,
        },
    },
    .state = {
        .rx_error = usart_receive_error_t::None,
    },
    #ifdef USART_RX_DMA_SUPPORT
    .dma = {
        .rx_data_available_event = EVT_USART2_RI,
        .rx_data_available_dma_btc = {
            .interrupt_handler = USARTx_rx_da_dma_btc_irq<2>,
        },
    }
    #endif
};

usart_config_t USART3_config = {
    .peripheral = {
        .register_base = CM_USART3,
        .clock_id = PWC_FCG1_USART3,
        .tx_pin_function = GPIO_FUNC_32,
        .rx_pin_function = GPIO_FUNC_33,
    },
    .interrupts = {
        .rx_data_full = {
            .interrupt_source = INT_SRC_USART3_RI,
            .interrupt_handler = USARTx_rx_data_available_irq<3>,
        },
        .rx_error = {
            .interrupt_source = INT_SRC_USART3_EI,
            .interrupt_handler = USARTx_rx_error_irq<3>,
        },
        .tx_buffer_empty = {
            .interrupt_source = INT_SRC_USART3_TI,
            .interrupt_handler = USARTx_tx_buffer_empty_irq<3>,
        },
        .tx_complete = {
            .interrupt_source = INT_SRC_USART3_TCI,
            .interrupt_handler = USARTx_tx_complete_irq<3>,
        },
    },
    .state = {
        .rx_error = usart_receive_error_t::None,
    },
    #ifdef USART_RX_DMA_SUPPORT
    .dma = {
        .rx_data_available_event = EVT_USART3_RI,
        .rx_data_available_dma_btc = {
            .interrupt_handler = USARTx_rx_da_dma_btc_irq<3>,
        },
    }
    #endif
};

usart_config_t USART4_config = {
    .peripheral = {
        .register_base = CM_USART4,
        .clock_id = PWC_FCG1_USART4,
        .tx_pin_function = GPIO_FUNC_36,
        .rx_pin_function = GPIO_FUNC_37,
    },
    .interrupts = {
        .rx_data_full = {
            .interrupt_source = INT_SRC_USART4_RI,
            .interrupt_handler = USARTx_rx_data_available_irq<4>,
        },
        .rx_error = {
            .interrupt_source = INT_SRC_USART4_EI,
            .interrupt_handler = USARTx_rx_error_irq<4>,
        },
        .tx_buffer_empty = {
            .interrupt_source = INT_SRC_USART4_TI,
            .interrupt_handler = USARTx_tx_buffer_empty_irq<4>,
        },
        .tx_complete = {
            .interrupt_source = INT_SRC_USART4_TCI,
            .interrupt_handler = USARTx_tx_complete_irq<4>,
        },
    },
    .state = {
        .rx_error = usart_receive_error_t::None,
    },
    #ifdef USART_RX_DMA_SUPPORT
    .dma = {
        .rx_data_available_event = EVT_USART4_RI,
        .rx_data_available_dma_btc = {
            .interrupt_handler = USARTx_rx_da_dma_btc_irq<4>,
        },
    }
    #endif
};
