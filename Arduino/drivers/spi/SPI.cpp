#include "SPI.h"
#include <core_debug.h>
#include <Arduino.h>
#include <drivers/gpio/gpio.h>

SPIClass SPI1(&SPI1_config);

uint32_t dma_to_aos(CM_DMA_TypeDef *dma_base, uint8_t channel)
{
    if(dma_base == CM_DMA1) {
        switch (channel) {
            case DMA_CH0:
                return AOS_DMA1_0;
            case DMA_CH1:
                return AOS_DMA1_1;
            case DMA_CH2:
                return AOS_DMA1_2;
            case DMA_CH3:
                return AOS_DMA1_3;
            default:
                return 0; // Invalid channel
        }
    } else if (dma_base == CM_DMA2) {
        switch (channel) {
            case DMA_CH0:
                return AOS_DMA2_0;
            case DMA_CH1:
                return AOS_DMA2_1;
            case DMA_CH2:
                return AOS_DMA2_2;
            case DMA_CH3:
                return AOS_DMA2_3;
            default:
                return 0; // Invalid channel
        }
    }
    return 0; // Invalid DMA base
}

void SPIClass::begin(const uint32_t frequency, const bool enable_DMA)
{
    stc_gpio_init_t stcGpioInit;
    stc_spi_init_t stcSpiInit;
    stc_dma_init_t stcDmaInit;
    stc_irq_signin_config_t stcIrqSignConfig;

    setClockFrequency(frequency);
    GPIO_StructInit(&stcGpioInit);
    stcGpioInit.u16PinDrv = PIN_HIGH_DRV;
    _GPIO_Init(this->clock_pin, &stcGpioInit);
    _GPIO_Init(this->mosi_pin, &stcGpioInit);
    _GPIO_Init(this->miso_pin, &stcGpioInit);
    // Additional initialization code can be added here
    GPIO_SetFunction(this->clock_pin, this->config->sck_func);
    GPIO_SetFunction(this->mosi_pin, this->config->mosi_func);
    GPIO_SetFunction(this->miso_pin, this->config->miso_func);

    FCG_Fcg1PeriphClockCmd(this->config->clock_id, ENABLE);
    SPI_StructInit(&stcSpiInit);
    stcSpiInit.u32WireMode          = SPI_3_WIRE;
    stcSpiInit.u32TransMode         = SPI_FULL_DUPLEX;
    stcSpiInit.u32MasterSlave       = SPI_MASTER;
    stcSpiInit.u32Parity            = SPI_PARITY_INVD;
    stcSpiInit.u32SpiMode           = SPI_MD_1;
    stcSpiInit.u32BaudRatePrescaler = SPI_BR_CLK_DIV64;
    stcSpiInit.u32DataBits          = SPI_DATA_SIZE_8BIT;
    stcSpiInit.u32FirstBit          = SPI_FIRST_MSB;
    stcSpiInit.u32FrameLevel        = SPI_1_FRAME;
    if (LL_OK == SPI_Init(this->config->register_base, &stcSpiInit)) {
        CORE_DEBUG_PRINTF("SPI initialized successfully.\n");
        if (this->enableDMA) {
            FCG_Fcg0PeriphClockCmd(this->config->dma_config.clock_id, ENABLE);
            DMA_StructInit(&stcDmaInit);
            stcDmaInit.u32BlockSize  = 1UL;
            stcDmaInit.u32TransCount = 1;
            stcDmaInit.u32DataWidth  = DMA_DATAWIDTH_8BIT;
            /* Configure TX */
            stcDmaInit.u32SrcAddrInc  = DMA_SRC_ADDR_INC;
            stcDmaInit.u32DestAddrInc = DMA_DEST_ADDR_FIX;
            stcDmaInit.u32SrcAddr     = (uint32_t)(0);
            stcDmaInit.u32DestAddr    = (uint32_t)(&this->config->register_base->DR);
            if (LL_OK != DMA_Init(this->config->dma_config.register_base, this->config->dma_config.channel, &stcDmaInit)) {
                CORE_DEBUG_PRINTF("Failed to initialize DMA for SPI.\n");
            } else {
                CORE_DEBUG_PRINTF("DMA initialized successfully for SPI.\n");
            }
            AOS_SetTriggerEventSrc(
                            dma_to_aos(this->config->dma_config.register_base, this->config->dma_config.channel), 
                            EVT_SRC_SPI1_SPTI);
                /* Enable DMA and channel */
            DMA_Cmd(this->config->dma_config.register_base, ENABLE);
            DMA_ChCmd(this->config->dma_config.register_base, this->config->dma_config.channel, ENABLE);
        }
    } else {
        CORE_DEBUG_PRINTF("Failed to initialize SPI.\n");
    }
}