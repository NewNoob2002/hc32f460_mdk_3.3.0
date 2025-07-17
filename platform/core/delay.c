#include <hc32_ll.h>
#include "delay.h"
#ifndef USE_FREERTOS

#ifndef SYSTICK_TICK_FREQ
#  define SYSTICK_TICK_FREQ     1000 // Hz
#endif

#define SYSTICK_TICK_STEP (1000 / SYSTICK_TICK_FREQ)
#define SYSTICK_TICK_LOAD (SystemCoreClock/ SYSTICK_TICK_FREQ)
#define CYCLES_PER_MICROSECOND (SystemCoreClock/ 1000000)

#define SYSTICK_TICK_MS (SYSTICK_TICK_STEP * systick_ticks)

static volatile uint32_t systick_ticks = 0;
/*
 * @brief SysTick Init
 * @param none
 * @return none
 */
void delay_init() {
	SysTick_Config(SYSTICK_TICK_LOAD);
	NVIC_SetPriority(SysTick_IRQn, (1<<__NVIC_PRIO_BITS) - 1);
}
/*
 * @brief SysTick Handler
 * @param none
 * @return none
 */
void SysTick_Handler() {
	systick_ticks += SYSTICK_TICK_STEP;
}

uint32_t millis()
{
	return SYSTICK_TICK_MS;
}

uint32_t micros(void)
{
	return (SYSTICK_TICK_MS * 1000 + (SYSTICK_TICK_LOAD - SysTick->VAL) / CYCLES_PER_MICROSECOND);
}

/**
  * @brief  毫秒级延时
  * @param  ms: 要延时的毫秒数
  * @retval 无
  */
void delay_ms(const uint32_t ms)
{
	const uint32_t start = systick_ticks;
	const uint32_t wait = ms / SYSTICK_TICK_STEP;

	while((systick_ticks - start) < wait)
	{
	}
}

/**
  * @brief  微秒级延时
  * @param  us: 要延时的微秒数
  * @retval 无
  */
void delay_us(const uint32_t us)
{
	uint32_t total = 0;
	const uint32_t target = CYCLES_PER_MICROSECOND * us;
	int last = SysTick->VAL;
	int now = last;
	int diff = 0;
	start:
		now = SysTick->VAL;
	diff = last - now;
	if(diff > 0)
	{
		total += diff;
	}
	else
	{
		total += diff + SYSTICK_TICK_LOAD;
	}
	if(total > target)
	{
		return;
	}
	last = now;
	goto start;
}

void clock_init()
{
    stc_clock_xtal_init_t     stcXtalInit;
    stc_clock_pll_init_t      stcMpllInit;
		int32_t ret;
    GPIO_AnalogCmd(GPIO_PORT_H, GPIO_PIN_00 | GPIO_PIN_01, ENABLE);
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
    if(( ret = CLK_XtalInit(&stcXtalInit)) != LL_OK)
		{
			while(1)
			{
			}
		}

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
#else
//do nothing
#endif

