#include "timer0_config.h"
#include "hc32f460.h"

timer0_channel_config_t TIMER01A_config = {
    .peripheral = {
        .register_base = CM_TMR0_1,
        .channel = TMR0_CH_A,
        .clock_id = FCG2_PERIPH_TMR0_1,
    },
    .interrupt = {
        .interrupt_source = INT_SRC_TMR0_1_CMP_A,
    },
};

timer0_channel_config_t TIMER01B_config = {
    .peripheral = {
        .register_base = CM_TMR0_1,
        .channel = TMR0_CH_B,
        .clock_id = FCG2_PERIPH_TMR0_1,
    },
    .interrupt = {
        .interrupt_source = INT_SRC_TMR0_1_CMP_B,
    },
};

timer0_channel_config_t TIMER02A_config = {
    .peripheral = {
        .register_base = CM_TMR0_2,
        .channel = TMR0_CH_A,
        .clock_id = FCG2_PERIPH_TMR0_2,
    },
    .interrupt = {
        .interrupt_source = INT_SRC_TMR0_2_CMP_A,
    },
};

timer0_channel_config_t TIMER02B_config = {
    .peripheral = {
        .register_base = CM_TMR0_2,
        .channel = TMR0_CH_B,
        .clock_id = FCG2_PERIPH_TMR0_2,
    },
    .interrupt = {
        .interrupt_source = INT_SRC_TMR0_2_CMP_B,
    },
};
