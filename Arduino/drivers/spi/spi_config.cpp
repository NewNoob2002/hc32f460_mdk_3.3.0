#include "spi_config.h"
#include "hc32f460.h"
#include <drivers/gpio/gpio.h>

spi_config_t SPI1_config = {
    .register_base = CM_SPI1,
    .clock_id = FCG1_PERIPH_SPI1,
    .mosi_func = GPIO_FUNC_40,
    .miso_func = GPIO_FUNC_41,
    .sck_func = GPIO_FUNC_43,

    .dma_config = {
        .register_base = CM_DMA1,
        .clock_id = (FCG0_PERIPH_DMA1 | FCG0_PERIPH_AOS),
        .channel = DMA_CH0,
    },
};

spi_config_t SPI2_config = {
    .register_base = CM_SPI2,
    .clock_id = FCG1_PERIPH_SPI2,
    .mosi_func = GPIO_FUNC_44,
    .miso_func = GPIO_FUNC_45,
    .sck_func = GPIO_FUNC_47,
};

spi_config_t SPI3_config = {
    .register_base = CM_SPI3,
    .clock_id = FCG1_PERIPH_SPI3,
    .mosi_func = GPIO_FUNC_40,
    .miso_func = GPIO_FUNC_41,
    .sck_func = GPIO_FUNC_43,
};

spi_config_t SPI4_config = {
    .register_base = CM_SPI4,
    .clock_id = FCG1_PERIPH_SPI4,
    .mosi_func = GPIO_FUNC_44,
    .miso_func = GPIO_FUNC_45,
    .sck_func = GPIO_FUNC_47,
};