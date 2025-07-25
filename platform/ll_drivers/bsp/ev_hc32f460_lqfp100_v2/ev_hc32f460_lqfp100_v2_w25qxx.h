/**
 *******************************************************************************
 * @file  ev_hc32f460_lqfp100_v2_w25qxx.h
 * @brief This file contains all the functions prototypes of the
          ev_hc32f460_lqfp100_v2_w25qxx driver library.
 @verbatim
   Change Logs:
   Date             Author          Notes
   2022-03-31       CDT             First version
   2023-09-30       CDT             Modify spi timeout value
 @endverbatim
 *******************************************************************************
 * Copyright (C) 2022-2025, Xiaohua Semiconductor Co., Ltd. All rights reserved.
 *
 * This software component is licensed by XHSC under BSD 3-Clause license
 * (the "License"); You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                    opensource.org/licenses/BSD-3-Clause
 *
 *******************************************************************************
 */
#ifndef __EV_HC32F460_LQFP100_V2_W25QXX_H__
#define __EV_HC32F460_LQFP100_V2_W25QXX_H__

/* C binding of definitions if building with C++ compiler */
#ifdef __cplusplus
extern "C"
{
#endif

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "w25qxx.h"
#include "ev_hc32f460_lqfp100_v2.h"

/**
 * @addtogroup BSP
 * @{
 */

/**
 * @addtogroup EV_HC32F460_LQFP100_V2
 * @{
 */

/**
 * @addtogroup EV_HC32F460_LQFP100_V2_W25QXX
 * @{
 */

#if ((BSP_W25QXX_ENABLE == DDL_ON) && (BSP_EV_HC32F460_LQFP100_V2 == BSP_EV_HC32F4XX))

/*******************************************************************************
 * Global type definitions ('typedef')
 ******************************************************************************/

/*******************************************************************************
 * Global pre-processor symbols/macros ('#define')
 ******************************************************************************/
/**
 * @defgroup EV_HC32F460_LQFP100_V2_W25QXX_Global_Macros EV_HC32F460_LQFP100_V2 W25QXX Global Macros
 * @{
 */

/**
 * @defgroup W25Qxx_SPI_Port W25Qxx SPI Port
 * @{
 */
#define BSP_SPI_CS_PORT                 (GPIO_PORT_C)
#define BSP_SPI_CS_PIN                  (GPIO_PIN_07)
#define BSP_SPI_CS_ACTIVE()             GPIO_ResetPins(BSP_SPI_CS_PORT, BSP_SPI_CS_PIN)
#define BSP_SPI_CS_INACTIVE()           GPIO_SetPins(BSP_SPI_CS_PORT, BSP_SPI_CS_PIN)

#define BSP_SPI_SCK_PORT                (GPIO_PORT_C)
#define BSP_SPI_SCK_PIN                 (GPIO_PIN_06)
#define BSP_SPI_SCK_PIN_FUNC            (GPIO_FUNC_43)          /*!< SPI3 SCK */

#define BSP_SPI_MOSI_PORT               (GPIO_PORT_D)           /*!< W25Qxx IO0 */
#define BSP_SPI_MOSI_PIN                (GPIO_PIN_08)
#define BSP_SPI_MOSI_PIN_FUNC           (GPIO_FUNC_40)          /*!< SPI3 MOSI */

#define BSP_SPI_MISO_PORT               (GPIO_PORT_D)           /*!< W25Qxx IO1 */
#define BSP_SPI_MISO_PIN                (GPIO_PIN_09)
#define BSP_SPI_MISO_PIN_FUNC           (GPIO_FUNC_41)          /*!< SPI3 MISO */
/**
 * @}
 */

/**
 * @defgroup W25Qxx_SPI_Instance W25Qxx SPI Instance
 * @{
 */
#define BSP_SPI_UNIT                    CM_SPI3
#define BSP_SPI_PERIPH_CLK              FCG1_PERIPH_SPI3
/**
 * @}
 */

/**
 * @defgroup W25Qxx_SPI_Timeout W25Qxx SPI Timeout
 * @{
 */
#define BSP_SPI_TIMEOUT                 (HCLK_VALUE)
/**
 * @}
 */

/**
 * @defgroup W25Qxx_Size W25Qxx Size
 * @{
 */
#define W25Q64_PAGE_SIZE                (256UL)
#define W25Q64_SECTOR_SIZE              (1024UL * 4UL)
#define W25Q64_BLOCK_SIZE               (1024UL * 64UL)
#define W25Q64_PAGE_PER_SECTOR          (W25Q64_SECTOR_SIZE / W25Q64_PAGE_SIZE)
#define W25Q64_MAX_ADDR                 (0x800000UL)
/**
 * @}
 */

/**
 * @}
 */

/*******************************************************************************
 * Global variable definitions ('extern')
 ******************************************************************************/

/*******************************************************************************
  Global function prototypes (definition in C source)
 ******************************************************************************/
/**
 * @addtogroup EV_HC32F460_LQFP100_V2_W25QXX_Global_Functions
 * @{
 */

void BSP_W25QXX_Init(void);
void BSP_W25QXX_DeInit(void);
int32_t BSP_W25QXX_Write(uint32_t u32Addr, const uint8_t *pu8Data, uint32_t u32NumByteToWrite);
int32_t BSP_W25QXX_Read(uint32_t u32Addr, uint8_t *pu8Data, uint32_t u32NumByteToRead);
int32_t BSP_W25QXX_EraseSector(uint32_t u32Addr);
int32_t BSP_W25QXX_EraseChip(void);

/**
 * @}
 */

#endif /* (BSP_W25QXX_ENABLE && BSP_EV_HC32F460_LQFP100_V2)) */

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* __EV_HC32F460_LQFP100_V2_W25QXX_H__ */

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
