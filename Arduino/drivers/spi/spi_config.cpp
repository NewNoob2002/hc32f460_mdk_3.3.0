#include "spi_config.h"
#include "hc32f460.h"
#include <drivers/gpio/gpio.h>

#define SPI1_ENABLE_DMA 0
#define SPI3_ENABLE_DMA 1

#if defined(SPI1_ENABLE_DMA) && SPI1_ENABLE_DMA
#warning "SPI1 DMA is enabled, ensure DMA is properly configured in your project settings."
#endif
#if defined(SPI3_ENABLE_DMA) && SPI3_ENABLE_DMA
#warning "SPI3 DMA is enabled, ensure DMA is properly configured in your project settings."
#endif

spi_config_t SPI1_config = {
    .peripheral = {
        .register_base = CM_SPI1,
        .clock_id      = FCG1_PERIPH_SPI1,
        .mosi_func     = GPIO_FUNC_40,
        .miso_func     = GPIO_FUNC_41,
        .sck_func      = GPIO_FUNC_43,
    },

#if defined(SPI1_ENABLE_DMA) && SPI1_ENABLE_DMA
    .tx_dma_config = {
        .register_base = CM_DMA1,
        .clock_id      = (FCG0_PERIPH_DMA1 | FCG0_PERIPH_AOS),
        .channel       = DMA_CH0,
    },
#endif
};

spi_config_t SPI2_config = {
    .peripheral = {
        .register_base = CM_SPI2,
        .clock_id      = FCG1_PERIPH_SPI2,
        .mosi_func     = GPIO_FUNC_44,
        .miso_func     = GPIO_FUNC_45,
        .sck_func      = GPIO_FUNC_47,
    },
#if defined(SPI2_ENABLE_DMA) && SPI2_ENABLE_DMA
    .tx_dma_config = {
        .register_base = CM_DMA1,
        .clock_id      = (FCG0_PERIPH_DMA1 | FCG0_PERIPH_AOS),
        .channel       = DMA_CH1,
    },
#endif
};

spi_config_t SPI3_config = {
    .peripheral = {
        .register_base = CM_SPI3,
        .clock_id      = FCG1_PERIPH_SPI3,
        .mosi_func     = GPIO_FUNC_40,
        .miso_func     = GPIO_FUNC_41,
        .sck_func      = GPIO_FUNC_43,
    },
#if defined(SPI3_ENABLE_DMA) && SPI3_ENABLE_DMA
    .tx_dma_config = {
        .register_base = CM_DMA1,
        .clock_id      = (FCG0_PERIPH_DMA1 | FCG0_PERIPH_AOS),
        .channel       = DMA_CH2,
    },
#endif
};

spi_config_t SPI4_config = {
    .peripheral = {
        .register_base = CM_SPI4,
        .clock_id      = FCG1_PERIPH_SPI4,
        .mosi_func     = GPIO_FUNC_44,
        .miso_func     = GPIO_FUNC_45,
        .sck_func      = GPIO_FUNC_47,
    },
#if defined(SPI4_ENABLE_DMA) && SPI4_ENABLE_DMA
    .tx_dma_config = {
        .register_base = CM_DMA1,
        .clock_id      = (FCG0_PERIPH_DMA1 | FCG0_PERIPH_AOS),
        .channel       = DMA_CH3,
    },
#endif
};