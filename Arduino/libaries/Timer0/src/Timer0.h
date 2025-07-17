#pragma once

// check DDL configuration
#if (LL_TMR0_ENABLE != DDL_ON)
#error "Timer0 library requires TIMER0 LL to be enabled"
#endif

#if (LL_PWC_ENABLE != DDL_ON)
#error "Timer0 library requires PWC LL to be enabled"
#endif

#include <core_debug.h>
#include <core_types.h>
#include <hc32_ll.h>
#include "timer0_config.h"

/**
 * @brief timer0 base frequency
 * @note timer0 uses PCLK1 as base frequency. with custom config, this can be changed
 */
#define TIMER0_BASE_FREQUENCY (SYSTEM_CLOCK_FREQUENCIES.pclk1)

/**
 * @brief timer0 lib inline function attribute
 */
#define TIMER0_INLINE_ATTR __attribute__((always_inline)) inline

class Timer0
{
public:
    /**
     * @brief Construct a new Timer0 object
     * @param config pointer to timer0 peripheral configuration
     * @param callback pointer to callback function for timer interrupt
     */
    Timer0(timer0_channel_config_t *config, const voidFuncPtr callback = nullptr);

    /**
     * @brief set the callback function for the timer0 channel
     * @param callback pointer to callback function for timer interrupt
     */
    void setCallback(const voidFuncPtr callback);

    /**
     * @brief start timer0 channel with frequency and prescaler
     * @param frequency the frequency to set the timer to
     * @param prescaler the prescaler to use. must be one of [1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024]
     * @note compare value = (base_freq / prescaler) / frequency
     * @note Timer0 Unit 1 Channel A will use LRC clock (32KHz) instead of PCLK1 (EXPERIMENTAL, might not work properly)
     * @note this function will not automatically start the timer interrupt. call resume() to start the interrupt
     * @note if the channel is already start()-ed, this function will stop the channel first
     */
    void start(const uint32_t frequency, const uint16_t prescaler = 1, bool use_callback = true);

    /**
     * @brief start timer0 channel with custom channel config
     * @param channel_config pointer to timer0 channel start configuration
     * @note this function will not automatically start the timer interrupt. call resume() to start the interrupt
     * @note if the channel is already start()-ed, this function will stop the channel first
     */
    void start(const stc_tmr0_init_t *channel_config);

    /**
     * @brief stop timer0 channel
     * @note this function will automatically stop the timer interrupt
     * @note if the channel is not start()-ed, this function will do nothing
     * @note if all channels are stopped, the timer0 peripheral will be disabled automatically
     */
    void stop();

    //
    // inlined functions
    // (inlined since they are somewhat timing-critical)
    //

    /**
     * @brief pause timer0 channel interrupt
     * @param channel timer0 channel to pause
     * @note if the channel is not start()-ed, this function will do nothing
     */
    TIMER0_INLINE_ATTR void pause()
    {
        // if not initialized, return false
        if (!this->isStarted)
        {
            return;
        }

        TMR0_Stop(this->config->peripheral.register_base, this->config->peripheral.channel);
    }

    /**
     * @brief resume timer0 channel interrupt
     * @param channel timer0 channel to resume
     * @note if the channel is not start()-ed, this function will do nothing
     */
    TIMER0_INLINE_ATTR void resume()
    {
        // if not initialized, return false
        if (!this->isStarted)
        {
            return;
        }

        TMR0_Start(this->config->peripheral.register_base, this->config->peripheral.channel);
    }

    /**
     * @brief check if timer0 channel interrupt is currently paused
     * @param channel timer0 channel to check
     * @return true if paused
     * @note if the channel is not start()-ed, this function will return false
     */
    TIMER0_INLINE_ATTR bool isPaused()
    {
        // if not initialized, return false
        if (!this->isStarted)
        {
            return false;
        }

        // otherwise, return the channel status
        if (this->config->peripheral.channel == TMR0_CH_A)
        {
            return !READ_REG32_BIT(this->config->peripheral.register_base->BCONR, 1<<0);
        }
        else
        {
            return !READ_REG32_BIT(this->config->peripheral.register_base->BCONR, 1<<16);
        }
    }

    /**
     * @brief set timer0 channel compare value
     * @param channel timer0 channel to set compare value
     * @param compare compare value to set
     * @note the channel must be start()-ed before calling this function
     */
    TIMER0_INLINE_ATTR void setCompareValue(const uint16_t compare)
    {
        CORE_ASSERT(this->isStarted, "Timer0::setCompare(): channel not initialized");
        TMR0_SetCompareValue(this->config->peripheral.register_base, this->config->peripheral.channel, compare);
    }

    /**
     * @brief get timer0 channel counter value
     * @param channel timer0 channel to get counter value
     * @return current counter value
     * @note the channel must be start()-ed before calling this function
     */
    TIMER0_INLINE_ATTR uint16_t getCount()
    {
        CORE_ASSERT(this->isStarted, "Timer0::getCount(): channel not initialized");
        return TMR0_GetCountValue(this->config->peripheral.register_base, this->config->peripheral.channel);
    }

    /**
     * @brief set timer0 channel callback priority
     * @param channel timer0 channel to set callback priority for
     * @param priority priority to set
     */
    TIMER0_INLINE_ATTR void setCallbackPriority(const uint32_t priority)
    {
        if (!this->isStarted)
        {
            CORE_ASSERT_FAIL("Timer0::setCallbackPriority(): channel not initialized");
            return;
        }

        NVIC_SetPriority(this->config->interrupt.interrupt_number, priority);
    }

    /**
     * @brief clear timer0 channel interrupt flag
     * @note must be called in the interrupt handler to clear the interrupt flag.
     *       failure to do so may cause undefined behavior
     */
    TIMER0_INLINE_ATTR void clearInterruptFlag()
    {
        if (this->config->peripheral.channel == TMR0_CH_A)
        {
            TMR0_ClearStatus(this->config->peripheral.register_base, TMR0_FLAG_CMP_A);
        }
        else
        {
            TMR0_ClearStatus(this->config->peripheral.register_base, TMR0_FLAG_CMP_B);
        }
    }

    void Timer0_delayUs(uint64_t us);
private:
    timer0_channel_config_t *config;
    voidFuncPtr callback;
    bool use_callback = true;
    bool isStarted = false;
};
