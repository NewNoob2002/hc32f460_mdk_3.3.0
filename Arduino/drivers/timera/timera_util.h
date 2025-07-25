#pragma once
#include <hc32_ll.h>
#include "timera_config.h"
#include "../sysclock/sysclock.h"
#include "../../WVariant.h"
#include "../../core_types.h"
#include "../../core_debug.h"

//
// debug print helpers
//
/**
 * @brief convert TimerA register to unit number
 * @note e.g. M4_TMRA1 -> 1
 */
#define TIMERA_REG_TO_X(reg) \
    reg == CM_TMRA_1   ? 1    \
    : reg == CM_TMRA_2 ? 2    \
    : reg == CM_TMRA_3 ? 3    \
    : reg == CM_TMRA_4 ? 4    \
    : reg == CM_TMRA_5 ? 5    \
    : reg == CM_TMRA_6 ? 6    \
                      : 0

/**
 * @brief convert TimerA channel to channel number
 * @note e.g. TimeraCh1 -> 1
 *
 * @note Timera channel enum is 0-based, so TimeraCh1 is enum value0
 */
#define TIMERA_CHANNEL_TO_X(ch) (int(ch) + 1)

/**
 * @brief TimerA CORE_DEBUG_PRINTF wrapper
 */
#define TIMERA_DEBUG_PRINTF(unit, channel, fmt, ...) \
    CORE_DEBUG_PRINTF("[TimerA%d_CH%d] " fmt, TIMERA_REG_TO_X((unit)->peripheral.register_base), TIMERA_CHANNEL_TO_X(channel), ##__VA_ARGS__)

//
// utility functions
//

/**
 * @brief get TimerA assignment for a gpio pin
 * @param pin the gpio pin to get assignment for
 * @param unit_config the TimerA unit config
 * @param output_channel the TimerA output channel
 * @param output_function the TimerA output function
 * @return true if pin has a TimerA assignment, false otherwise
 */
inline bool timera_get_assignment(
    gpio_pin_t pin,
    timera_config_t *&unit_config,
    uint8_t &output_channel,
    uint8_t &output_function)
{
    // ensure pin is valid
    ASSERT_GPIO_PIN_VALID(pin, "timera_get_assignment", return false);

    // get timera info from pin map
    const pin_timera_info_t timera_info = PIN_MAP[pin].timera_info;

    // if no assignment, return false
    if (timera_info.unit == 0)
    {
        return false;
    }

    // get timera unit config
    constexpr timera_config_t *UNIT_CONFIGS[6] = {
        &TIMERA1_config,
        &TIMERA2_config,
        &TIMERA3_config,
        &TIMERA4_config,
        &TIMERA5_config,
        &TIMERA6_config,
    };

    // unit is 1 <= unit <= 6
    CORE_ASSERT(timera_info.unit >= 1 && timera_info.unit <= 6, "timera unit assignment out-of-range", return false);
    unit_config = UNIT_CONFIGS[timera_info.unit - 1];

    // get pin output channel
    // channel is 0 <= channel <= 7, 0 is channel 1
    CORE_ASSERT(timera_info.channel >= 0 && timera_info.channel <= 7, "timera channel assignment out-of-range", return false);
    output_channel = static_cast<uint8_t>(timera_info.channel);

    // get pin output gpio function
    // function is 0 <= function <= 2, 0 is Func_Tima0
    switch (timera_info.function)
    {
    case 0:
        output_function = GPIO_FUNC_4;
        break;
    case 1:
        output_function = GPIO_FUNC_5;
        break;
    case 2:
        output_function = GPIO_FUNC_6;
        break;
    default:
        CORE_ASSERT_FAIL("timera function assignment out-of-range");
        return false;
    }

    // all ok
    // TIMERA_DEBUG_PRINTF(unit_config, output_channel, "get_assignment: pin %d Func%d\n", pin, output_function);
    return true;
}

/**
 * @brief check if a TimerA channel is active
 * @param unit_config the TimerA unit config
 * @param channel the TimerA channel
 * @return true if channel is active, false otherwise
 */
inline bool timera_is_channel_active(timera_config_t *unit_config, const uint8_t channel)
{
    CORE_ASSERT(unit_config != NULL, "timera unit config is NULL", return false);

    // cast channel to integer
    // must be 0 <= ch <= 7, 0 == channel 1
    const uint8_t ch = static_cast<uint8_t>(channel);
    CORE_ASSERT(ch >= 0 && ch <= 7, "timera channel out-of-range", return false);

    // check if channel is in use
    return static_cast<bool>(unit_config->state.active_channels & TIMERA_STATE_ACTIVE_CHANNEL_BIT(ch + 1));
}

/**
 * @brief set a TimerA channel active flag
 * @param unit_config the TimerA unit config
 * @param channel the TimerA channel
 * @param is_active true to set channel as active, false to set channel as inactive
 */
inline void timera_set_channel_active_flag(timera_config_t *unit_config, const uint8_t channel, bool is_active)
{
    CORE_ASSERT(unit_config != NULL, "timera unit config is NULL", return);

    // cast channel to integer
    // must be 0 <= ch <= 7, 0 == channel 1
    const uint8_t ch = static_cast<uint8_t>(channel);
    CORE_ASSERT(ch >= 0 && ch <= 7, "timera channel out-of-range", return);

    // update channel active flag
    if (is_active)
    {
        unit_config->state.active_channels |= TIMERA_STATE_ACTIVE_CHANNEL_BIT(ch + 1);
    }
    else
    {
        unit_config->state.active_channels &= ~TIMERA_STATE_ACTIVE_CHANNEL_BIT(ch + 1);
    }

    TIMERA_DEBUG_PRINTF(unit_config, channel, "set_channel_active_flag: %d\n", is_active ? 1 : 0);
}

/**
 * @brief check if a TimerA unit is initialized
 * @param unit_config the TimerA unit config
 * @return true if unit is initialized, false otherwise
 */
#define timera_is_unit_initialized(unit_config) ((unit_config)->state.base_init != NULL)

/**
 * @brief get TimerA base clock (PCLK1)
 * @return TimerA base clock (PCLK1)
 */
inline uint32_t timera_get_base_clock()
{
    update_system_clock_frequencies();
    return SYSTEM_CLOCK_FREQUENCIES.pclk1;
}

/**
 * @brief number to TimerA clock divider
 */
inline uint8_t timera_n_to_clk_div(uint16_t n)
{
    switch (n)
    {
    default:
        CORE_ASSERT_FAIL("invalid timera clock divider");
    case 1:
        return TMRA_CLK_DIV1;
    case 2:
        return TMRA_CLK_DIV2;
    case 4:
        return TMRA_CLK_DIV4;
    case 8:
        return TMRA_CLK_DIV8;
    case 16:
        return TMRA_CLK_DIV16;
    case 32:
        return TMRA_CLK_DIV32;
    case 64:
        return TMRA_CLK_DIV64;
    case 128:
        return TMRA_CLK_DIV128;
    case 256:
        return TMRA_CLK_DIV256;
    case 512:
        return TMRA_CLK_DIV512;
    case 1024:
        return TMRA_CLK_DIV1024;
    }
}

/**
 * @brief TimerA clock divider to number
 */
inline uint16_t timera_clk_div_to_n(uint8_t clk_div)
{
    switch (clk_div)
    {
    default:
        CORE_ASSERT_FAIL("invalid timera clock divider");
    case TMRA_CLK_DIV1:
        return 1;
    case TMRA_CLK_DIV2:
        return 2;
    case TMRA_CLK_DIV4:
        return 4;
    case TMRA_CLK_DIV8:
        return 8;
    case TMRA_CLK_DIV16:
        return 16;
    case TMRA_CLK_DIV32:
        return 32;
    case TMRA_CLK_DIV64:
        return 64;
    case TMRA_CLK_DIV128:
        return 128;
    case TMRA_CLK_DIV256:
        return 256;
    case TMRA_CLK_DIV512:
        return 512;
    case TMRA_CLK_DIV1024:
        return 1024;
    }
}
