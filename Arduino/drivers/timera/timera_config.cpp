#include "timera_config.h"

timera_config_t TIMERA1_config = {
    .peripheral = {
        .register_base = CM_TMRA_1,
        .clock_id = FCG2_PERIPH_TMRA_1,
    },
    .overflow_interrupt = {
        .interrupt_source = INT_SRC_TMRA_1_OVF,
    },
    .underflow_interrupt = {
        .interrupt_source = INT_SRC_TMRA_1_UDF,
    },
    .compare_interrupt = {
        .interrupt_source = INT_SRC_TMRA_1_CMP,
    },
};

timera_config_t TIMERA2_config = {
    .peripheral = {
        .register_base = CM_TMRA_2,
        .clock_id = FCG2_PERIPH_TMRA_2,
    },
    .overflow_interrupt = {
        .interrupt_source = INT_SRC_TMRA_2_OVF,
    },
    .underflow_interrupt = {
        .interrupt_source = INT_SRC_TMRA_2_UDF,
    },
    .compare_interrupt = {
        .interrupt_source = INT_SRC_TMRA_2_CMP,
    },
};

timera_config_t TIMERA3_config = {
    .peripheral = {
        .register_base = CM_TMRA_3,
        .clock_id = FCG2_PERIPH_TMRA_3,
    },
    .overflow_interrupt = {
        .interrupt_source = INT_SRC_TMRA_3_OVF,
    },
    .underflow_interrupt = {
        .interrupt_source = INT_SRC_TMRA_3_UDF,
    },
    .compare_interrupt = {
        .interrupt_source = INT_SRC_TMRA_3_CMP,
    },
};

timera_config_t TIMERA4_config = {
    .peripheral = {
        .register_base = CM_TMRA_4,
        .clock_id = FCG2_PERIPH_TMRA_4,
    },
    .overflow_interrupt = {
        .interrupt_source = INT_SRC_TMRA_4_OVF,
    },
    .underflow_interrupt = {
        .interrupt_source = INT_SRC_TMRA_4_UDF,
    },
    .compare_interrupt = {
        .interrupt_source = INT_SRC_TMRA_4_CMP,
    },
};

timera_config_t TIMERA5_config = {
    .peripheral = {
        .register_base = CM_TMRA_5,
        .clock_id = FCG2_PERIPH_TMRA_5,
    },
    .overflow_interrupt = {
        .interrupt_source = INT_SRC_TMRA_5_OVF,
    },
    .underflow_interrupt = {
        .interrupt_source = INT_SRC_TMRA_5_UDF,
    },
    .compare_interrupt = {
        .interrupt_source = INT_SRC_TMRA_5_CMP,
    },
};

timera_config_t TIMERA6_config = {
    .peripheral = {
        .register_base = CM_TMRA_6,
        .clock_id = FCG2_PERIPH_TMRA_6,
    },
    .overflow_interrupt = {
        .interrupt_source = INT_SRC_TMRA_6_OVF,
    },
    .underflow_interrupt = {
        .interrupt_source = INT_SRC_TMRA_6_UDF,
    },
    .compare_interrupt = {
        .interrupt_source = INT_SRC_TMRA_6_CMP,
    },
};
