#include "SPI.h"
#include "hc32_ll_dma.h"
#include <Arduino.h>
#include <drivers/gpio/gpio.h>

#define SPI_TO_EVENT_SRC(spi_base)                                                         \
    ((spi_base == CM_SPI1) ? EVT_SRC_SPI1_SPTI : (spi_base == CM_SPI2) ? EVT_SRC_SPI2_SPTI \
                                             : (spi_base == CM_SPI3)   ? EVT_SRC_SPI3_SPTI \
                                             : (spi_base == CM_SPI4)   ? EVT_SRC_SPI4_SPTI \
                                                                       : 0)

SPIClass SPI1(&SPI1_config);

uint32_t dma_to_aos(CM_DMA_TypeDef *dma_base, uint8_t channel)
{
    if (dma_base == CM_DMA1) {
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
    this->enableDMA = enable_DMA;

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
    stcSpiInit.u32BaudRatePrescaler = this->divider;
    stcSpiInit.u32DataBits          = SPI_DATA_SIZE_8BIT;
    stcSpiInit.u32FirstBit          = SPI_FIRST_MSB;
    stcSpiInit.u32FrameLevel        = SPI_1_FRAME;
    if (LL_OK == SPI_Init(this->config->register_base, &stcSpiInit)) {
        CORE_DEBUG_PRINTF("SPI initialized successfully.\n");
        printf_config();
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
                AOS_SetTriggerEventSrc(
                    dma_to_aos(this->config->dma_config.register_base, this->config->dma_config.channel),
                    (en_event_src_t)SPI_TO_EVENT_SRC(this->config->register_base));
                /* Enable DMA and channel */
                DMA_Cmd(this->config->dma_config.register_base, ENABLE);
                DMA_ChCmd(this->config->dma_config.register_base, this->config->dma_config.channel, ENABLE);
            }
        }
    } else {
        CORE_DEBUG_PRINTF("Failed to initialize SPI.\n");
    }
}

void SPIClass::end()
{
    if (this->enableDMA) {
        DMA_ChCmd(this->config->dma_config.register_base, this->config->dma_config.channel, DISABLE);
        DMA_Cmd(this->config->dma_config.register_base, DISABLE);
    }
    SPI_DeInit(this->config->register_base);
    FCG_Fcg1PeriphClockCmd(this->config->clock_id, DISABLE);
}

void SPIClass::send(const uint32_t data)
{
    SPI_Cmd(this->config->register_base, ENABLE);
    while (SPI_GetStatus(this->config->register_base, SPI_FLAG_TX_BUF_EMPTY) == RESET) {
        // Wait until the TX buffer is empty
        vTaskDelay(1); // Add a small delay to avoid busy waiting
    }
    WRITE_REG32(this->config->register_base->DR, data);
}

uint32_t SPIClass::receive()
{
    SPI_Cmd(this->config->register_base, ENABLE);
    // no need do now
    return 0; // Return 0 if no data is received
}
void SPIClass::push_inDMA(const uint8_t *buffer, const size_t count)
{
    if (this->enableDMA) {
        /* Disable SPI */
        SPI_Cmd(this->config->register_base, DISABLE);
        DMA_SetSrcAddr(this->config->dma_config.register_base, this->config->dma_config.channel, (uint32_t)buffer);
        DMA_SetTransCount(this->config->dma_config.register_base, this->config->dma_config.channel, count);
        DMA_ChCmd(this->config->dma_config.register_base, this->config->dma_config.channel, ENABLE);
        SPI_Cmd(this->config->register_base, ENABLE);
        // Wait for the DMA transfer to complete
        while (DMA_GetTransStatus(this->config->dma_config.register_base, DMA_FLAG_TC_CH0)) {
            // Wait until DMA transfer is complete
            vTaskDelay(1); // Add a small delay to avoid busy waiting
        }
    } else {
        CORE_DEBUG_PRINTF("DMA is not enabled for SPI.\n");
    }
}