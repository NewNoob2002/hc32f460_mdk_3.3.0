#ifndef MCU_CONFIG_H
#define MCU_CONFIG_H

#define EXAMPLE_PERIPH_WE               (LL_PERIPH_GPIO | LL_PERIPH_EFM | LL_PERIPH_FCG | \
                                         LL_PERIPH_PWC_CLK_RMU | LL_PERIPH_SRAM)
#define EXAMPLE_PERIPH_WP               (LL_PERIPH_EFM | LL_PERIPH_FCG | LL_PERIPH_SRAM)
//power control
#define POWER_CONTROL_PORT     GPIO_PORT_B     
#define POWER_CONTROL_PIN      GPIO_PIN_03
//power led
#define POWER_LED_PORT     GPIO_PORT_C    
#define POWER_LED_PIN      GPIO_PIN_13
//function led
#define FUNC_LED_PORT     GPIO_PORT_B    
#define FUNC_LED_PIN      GPIO_PIN_05
//charge led
#define CHARGE_LED_PORT     GPIO_PORT_H    
#define CHARGE_LED_PIN      GPIO_PIN_02

//watchdog pin
#define WATCHDOG_FEED_PORT     GPIO_PORT_A     
#define WATCHDOG_FEED_PIN      GPIO_PIN_06
#endif