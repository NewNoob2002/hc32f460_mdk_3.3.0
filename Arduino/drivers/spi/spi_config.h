#ifndef SPI_CONFIG_H_
#define SPI_CONFIG_H_

/**
 * @brief SPI peripheral configuration
 */
#include <hc32_ll.h>

typedef struct dma_config_t
{
    CM_DMA_TypeDef *register_base; // Base address of the DMA peripheral
    uint32_t clock_id;    // Clock ID for the DMA peripheral
    uint8_t channel;  // DMA channel DMA_CH0-DMA_CH3
} dma_config_t;

typedef struct spi_config_t
{
    /**
     * @brief The base address of the SPI peripheral.
     */
    CM_SPI_TypeDef *register_base;
    
    /**
     * @brief SPI peripheral channel.
     */
    uint32_t clock_id;

    /**
     * @brief SPI peripheral MOSI pin function
     */
    uint16_t mosi_func;

    /**
     * @brief SPI peripheral MISO pin function
     */
    uint16_t miso_func;

    /**
     * @brief SPI peripheral SCK pin function
     */
    uint16_t sck_func;

    /**
     * @brief DMA configuration for the SPI peripheral.
     */
    dma_config_t dma_config; // DMA configuration for the SPI peripheral
} spi_config_t;

/**
 * @brief SPI1 configuration
 */
extern spi_config_t SPI1_config;

/**
 * @brief SPI2 configuration
 */
extern spi_config_t SPI2_config;

/**
 * @brief SPI3 configuration
 */
extern spi_config_t SPI3_config;

/**
 * @brief SPI4 configuration
 */
extern spi_config_t SPI4_config;
#endif