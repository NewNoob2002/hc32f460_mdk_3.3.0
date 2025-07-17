#include <hc32_ll.h>
#include "sysclock.h"

system_clock_frequencies_t SYSTEM_CLOCK_FREQUENCIES = {0};

void update_system_clock_frequencies()
{
    stc_clock_freq_t clkFreq;
    CLK_GetClockFreq(&clkFreq);
    SYSTEM_CLOCK_FREQUENCIES.system = clkFreq.u32SysclkFreq;
    SYSTEM_CLOCK_FREQUENCIES.hclk = clkFreq.u32HclkFreq;
    SYSTEM_CLOCK_FREQUENCIES.pclk0 = clkFreq.u32Pclk0Freq;
    SYSTEM_CLOCK_FREQUENCIES.pclk1 = clkFreq.u32Pclk1Freq;
    SYSTEM_CLOCK_FREQUENCIES.pclk2 = clkFreq.u32Pclk2Freq;
    SYSTEM_CLOCK_FREQUENCIES.pclk3 = clkFreq.u32Pclk3Freq;
    SYSTEM_CLOCK_FREQUENCIES.pclk4 = clkFreq.u32Pclk4Freq;
    SYSTEM_CLOCK_FREQUENCIES.exclk = clkFreq.u32ExclkFreq;

    // update SystemCoreClock of DDL
    SystemCoreClock = SYSTEM_CLOCK_FREQUENCIES.system;
}

void clock_init(void)
{
    stc_clock_xtal_init_t     stcXtalInit;
    stc_clock_pll_init_t      stcMpllInit;

    GPIO_AnalogCmd(XTAL_PORT, XTAL_PIN, ENABLE);
    (void)CLK_XtalStructInit(&stcXtalInit);
    (void)CLK_PLLStructInit(&stcMpllInit);

    /* Set bus clk div. */
    CLK_SetClockDiv(CLK_BUS_CLK_ALL, (CLK_HCLK_DIV1 | CLK_EXCLK_DIV2 | CLK_PCLK0_DIV1 | CLK_PCLK1_DIV2 | \
                                      CLK_PCLK2_DIV4 | CLK_PCLK3_DIV4 | CLK_PCLK4_DIV2));

    /* Config Xtal and enable Xtal */
    stcXtalInit.u8Mode = CLK_XTAL_MD_OSC;
    stcXtalInit.u8Drv = CLK_XTAL_DRV_ULOW;
    stcXtalInit.u8State = CLK_XTAL_ON;
    stcXtalInit.u8StableTime = CLK_XTAL_STB_2MS;
    (void)CLK_XtalInit(&stcXtalInit);

    /* MPLL config (XTAL / pllmDiv * plln / PllpDiv = 200M). */
    stcMpllInit.PLLCFGR = 0UL;
    stcMpllInit.PLLCFGR_f.PLLM = 1UL - 1UL;
    stcMpllInit.PLLCFGR_f.PLLN = 50UL - 1UL;
    stcMpllInit.PLLCFGR_f.PLLP = 2UL - 1UL;
    stcMpllInit.PLLCFGR_f.PLLQ = 2UL - 1UL;
    stcMpllInit.PLLCFGR_f.PLLR = 2UL - 1UL;
    stcMpllInit.u8PLLState = CLK_PLL_ON;
    stcMpllInit.PLLCFGR_f.PLLSRC = CLK_PLL_SRC_XTAL;
    (void)CLK_PLLInit(&stcMpllInit);
    /* Wait MPLL ready. */
    while (SET != CLK_GetStableStatus(CLK_STB_FLAG_PLL)) {
        ;
    }

    /* sram init include read/write wait cycle setting */
    SRAM_SetWaitCycle(SRAM_SRAMH, SRAM_WAIT_CYCLE0, SRAM_WAIT_CYCLE0);
    SRAM_SetWaitCycle((SRAM_SRAM12 | SRAM_SRAM3 | SRAM_SRAMR), SRAM_WAIT_CYCLE1, SRAM_WAIT_CYCLE1);

    /* flash read wait cycle setting */
    (void)EFM_SetWaitCycle(EFM_WAIT_CYCLE5);
    /* 3 cycles for 126MHz ~ 200MHz */
    GPIO_SetReadWaitCycle(GPIO_RD_WAIT3);
    /* Switch driver ability */
    (void)PWC_HighSpeedToHighPerformance();
    /* Switch system clock source to MPLL. */
    CLK_SetSysClockSrc(CLK_SYSCLK_SRC_PLL);
    /* Reset cache ram */
    EFM_CacheRamReset(ENABLE);
    EFM_CacheRamReset(DISABLE);
    /* Enable cache */
    EFM_CacheCmd(ENABLE);
	update_system_clock_frequencies();
}

int32_t xtal32_init()
{
  stc_clock_xtal32_init_t stcXtal32Init;
  stc_fcm_init_t stcFcmInit;
  uint32_t u32TimeOut = 0UL;
  uint32_t u32Time = HCLK_VALUE / 5UL;

  if (CLK_XTAL32_ON == READ_REG8(CM_CMU->XTAL32CR)) {
      /* Disable xtal32 */
      (void)CLK_Xtal32Cmd(DISABLE);
  }

  /* Xtal32 config */
  (void)CLK_Xtal32StructInit(&stcXtal32Init);
  stcXtal32Init.u8State  = CLK_XTAL32_ON;
  stcXtal32Init.u8Drv    = CLK_XTAL32_DRV_MID;
  stcXtal32Init.u8Filter = CLK_XTAL32_FILTER_ALL_MD;
  GPIO_AnalogCmd(GPIO_PORT_C, GPIO_PIN_14 | GPIO_PIN_15, ENABLE);
  (void)CLK_Xtal32Init(&stcXtal32Init);

  /* FCM config */
  FCG_Fcg0PeriphClockCmd(FCG0_PERIPH_FCM, ENABLE);
  (void)FCM_StructInit(&stcFcmInit);
  stcFcmInit.u32RefClock       = FCM_REF_CLK_MRC;
  stcFcmInit.u32RefClockDiv    = FCM_REF_CLK_DIV8192;
  stcFcmInit.u32RefClockEdge   = FCM_REF_CLK_RISING;
  stcFcmInit.u32TargetClock    = FCM_TARGET_CLK_XTAL32;
  stcFcmInit.u32TargetClockDiv = FCM_TARGET_CLK_DIV1;
  stcFcmInit.u16LowerLimit     = (uint16_t)((XTAL32_VALUE / (MRC_VALUE / 8192U)) * 96UL / 100UL);
  stcFcmInit.u16UpperLimit     = (uint16_t)((XTAL32_VALUE / (MRC_VALUE / 8192U)) * 104UL / 100UL);
  (void)FCM_Init(CM_FCM, &stcFcmInit);
  /* Enable FCM, to ensure xtal32 stable */
  FCM_Cmd(CM_FCM, ENABLE);
  for (;;) {
      if (SET == FCM_GetStatus(CM_FCM, FCM_FLAG_END)) {
          FCM_ClearStatus(CM_FCM, FCM_FLAG_END);
          if (SET == FCM_GetStatus(CM_FCM, FCM_FLAG_ERR | FCM_FLAG_OVF)) {
              FCM_ClearStatus(CM_FCM, FCM_FLAG_ERR | FCM_FLAG_OVF);
          } else {
              (void)FCM_DeInit(CM_FCM);
              FCG_Fcg0PeriphClockCmd(FCG0_PERIPH_FCM, DISABLE);
              return LL_OK;
          }
      }
      u32TimeOut++;
      if (u32TimeOut > u32Time) {
          (void)FCM_DeInit(CM_FCM);
          FCG_Fcg0PeriphClockCmd(FCG0_PERIPH_FCM, DISABLE);
          return LL_ERR_TIMEOUT;
      }
  }
} 