#include "Timer0.h"
#include "hc32_ll_tmr0.h"
#include <drivers/sysclock/sysclock.h>
#include <drivers/irqn/irqn.h>

//
// helpers
//

/**
 * @brief timer register to timer number
 */
#define TIMER0x_REG_TO_X(reg) reg == CM_TMR0_1 ? 1 : 2

/**
 * @brief timer channel to channel string
 */
#define TIMER0x_CHANNEL_TO_CH_STR(channel) \
    channel == TMR0_CH_A ? "A" : "B"

#define TIMER0x_CH_TO_INT(channel) \
    channel == TMR0_CH_A ? TMR0_INT_CMP_A : TMR0_INT_CMP_B

#define TIMER0x_CH_TO_FLAG(channel) \
    channel == TMR0_CH_A ? TMR0_FLAG_CMP_A : TMR0_FLAG_CMP_B
/**
 * @brief debug print helper
 */
#define TIMER0_DEBUG_PRINTF(fmt, ...)                                              \
    CORE_DEBUG_PRINTF("[Timer0%d%s] " fmt,                                         \
                      TIMER0x_REG_TO_X(this->config->peripheral.register_base),    \
                      TIMER0x_CHANNEL_TO_CH_STR(this->config->peripheral.channel), \
                      ##__VA_ARGS__)

/**
 * @brief convert numerical value to en_tim0_clock_div_t
 * @note assert fails if invalid value
 */
inline uint8_t numeric_to_clock_div(const uint16_t n)
{
    switch (n) {
        case 0:
        case 1:
            return TMR0_CLK_DIV1;
        case 2:
            return TMR0_CLK_DIV2;
        case 4:
            return TMR0_CLK_DIV4;
        case 8:
            return TMR0_CLK_DIV8;
        case 16:
            return TMR0_CLK_DIV16;
        case 32:
            return TMR0_CLK_DIV32;
        case 64:
            return TMR0_CLK_DIV64;
        case 128:
            return TMR0_CLK_DIV128;
        case 256:
            return TMR0_CLK_DIV256;
        case 512:
            return TMR0_CLK_DIV512;
        case 1024:
            return TMR0_CLK_DIV1024;
        default:
            CORE_ASSERT_FAIL("Invalid clock divider value");
            return TMR0_CLK_DIV1;
    }
}

/**
 * @brief timer0 interrupt registration
 */
inline void timer0_irq_register(timer0_interrupt_config_t &irq, voidFuncPtr callback, const char *name)
{
    // get auto-assigned irqn and set in irq struct
    IRQn_Type irqn;
    irqn_aa_get(irqn, name);
    irq.interrupt_number = irqn;

    // create irq registration struct
    stc_irq_signin_config_t irqConf = {
        .enIntSrc    = irq.interrupt_source,
        .enIRQn      = irq.interrupt_number,
        .pfnCallback = callback,
    };

    // register and enable irq with default priority
    INTC_IrqSignIn(&irqConf);
    NVIC_SetPriority(irqConf.enIRQn, DDL_IRQ_PRIO_DEFAULT);
    NVIC_ClearPendingIRQ(irqConf.enIRQn);
    NVIC_EnableIRQ(irqConf.enIRQn);
}

/**
 * @brief timer0 interrupt resign
 */
inline void timer0_irq_resign(timer0_interrupt_config_t &irq, const char *name)
{
    // disable interrupt and clear pending
    NVIC_DisableIRQ(irq.interrupt_number);
    NVIC_ClearPendingIRQ(irq.interrupt_number);
    INTC_IrqSignOut(irq.interrupt_number);

    // resign auto-assigned irqn
    irqn_aa_resign(irq.interrupt_number, name);
}

//
// Timer0 class implementation
//
Timer0::Timer0(timer0_channel_config_t *config, voidFuncPtr callback)
{
    CORE_ASSERT(config != nullptr, "Timer0 config must not be null")

    this->config   = config;
    this->callback = callback;
}

void Timer0::setCallback(const voidFuncPtr callback)
{
    CORE_ASSERT(callback != nullptr, "Timer0 callback must not be null")
    this->callback = callback;
}

void Timer0::start(const uint32_t frequency, const uint16_t prescaler, bool use_callback)
{
    stc_tmr0_init_t channel_config;
    channel_config.u32ClockSrc = TMR0_CLK_SRC_XTAL32;
    channel_config.u32Func     = TMR0_FUNC_CMP;
    // setup channel clock source and set base frequency
    uint32_t base_frequency;
    if (this->config->peripheral.register_base == CM_TMR0_1 && this->config->interrupt.interrupt_source == INT_SRC_TMR0_1_CMP_A) {
        // Timer0 Unit 1 Channel A does not support Sync mode, and thus does not support PCLK1 as clock source
        // instead, LRC is used as clock source

        // get LRC frequency (fixed, calibrated at factory)
        base_frequency = XTAL32_VALUE;

        // set channel clock source:
        // Async mode, LRC
        TIMER0_DEBUG_PRINTF("using Timer01A is not recommended, as it does not support sync mode\n");
    } else {
        // all other Timer channels support PCLK1 as clock source
        channel_config.u32ClockSrc = TMR0_CLK_SRC_INTERN_CLK;
        // update clock frequencies and get PCLK1 frequency
        update_system_clock_frequencies();
        base_frequency = SYSTEM_CLOCK_FREQUENCIES.pclk1;
    }

    // calculate the compare value needed to match the target frequency
    // CMP = (base_freq / prescaler) / frequency
    uint32_t compare = (base_frequency / uint32_t(prescaler)) / frequency;
    // ensure compare value does not exceed 16 bits, and larger than 0
    CORE_ASSERT(compare > 0 && compare <= 0xFFFF, "Timer0::start(): compare value exceeds 16 bits");

    // set prescaler and compare value
    channel_config.u32ClockDiv     = numeric_to_clock_div(prescaler);
    channel_config.u16CompareValue = uint16_t(compare);

    // debug print auto-config values
    TIMER0_DEBUG_PRINTF("auto-found cmp= %d for fBase=%d and prescaler=%d\n", int(compare), int(base_frequency), int(prescaler));

    // start timer channel with config
    this->use_callback = use_callback;
    start(&channel_config);
}

void Timer0::start(const stc_tmr0_init_t *channel_config)
{
    CORE_ASSERT(channel_config != nullptr, "Timer0::start(): channel_config is null");

    // if already started, stop first
    if (this->isStarted) {
        stop();
    }

    // enable Timer0 peripheral clock
    FCG_Fcg2PeriphClockCmd(this->config->peripheral.clock_id, ENABLE);

    // enable LRC clock if used
    if (channel_config->u32ClockSrc == TMR0_CLK_SRC_XTAL32) {
        xtal32_init();
    }

    // initialize timer channel
    TMR0_Init(this->config->peripheral.register_base, this->config->peripheral.channel, channel_config);

    if (this->use_callback) {
        // register interrupt
        CORE_ASSERT(this->callback != nullptr, "Timer0::start(): callback not set");
        timer0_irq_register(this->config->interrupt, this->callback, "Timer0");

        // enable timer interrupt
        TMR0_IntCmd(this->config->peripheral.register_base, TIMER0x_CH_TO_INT(this->config->peripheral.channel), ENABLE);
    }
    // set channel initialized flag
    this->isStarted = true;

    TIMER0_DEBUG_PRINTF("started with compare value %d\n", channel_config->u16CompareValue);
    TMR0_Start(this->config->peripheral.register_base, this->config->peripheral.channel);
}

void Timer0::stop()
{
    if (!this->isStarted) {
        return;
    }

    // pause timer
    pause();
    // reset channel initialized flag early
    this->isStarted = false;

    if (this->use_callback) {
        // disable timer interrupt
        TMR0_IntCmd(this->config->peripheral.register_base, TIMER0x_CH_TO_INT(this->config->peripheral.channel), DISABLE);

        // resign interrupt
        timer0_irq_resign(this->config->interrupt, "Timer0");
    }
    // de-init timer channel
    TMR0_DeInit(this->config->peripheral.register_base);

    TIMER0_DEBUG_PRINTF("stopped channel\n");
}

void Timer0::Timer0_delayUs(uint64_t us)
{
    static uint64_t start = 0;
    TMR0_Stop(this->config->peripheral.register_base, this->config->peripheral.channel);
    TMR0_SetCountValue(this->config->peripheral.register_base, this->config->peripheral.channel, 0);
    TMR0_Start(this->config->peripheral.register_base, this->config->peripheral.channel);

    while(start < us)
    {
        if(TMR0_GetStatus(this->config->peripheral.register_base,TIMER0x_CH_TO_FLAG(this->config->peripheral.channel)))
        {
            TMR0_SetCountValue(this->config->peripheral.register_base, this->config->peripheral.channel, 0);
            TMR0_ClearStatus(this->config->peripheral.register_base, TIMER0x_CH_TO_FLAG(this->config->peripheral.channel));
            start++;
        }
    }
    start = 0;
    TMR0_Stop(this->config->peripheral.register_base, this->config->peripheral.channel);
}