/*
 Copyright (c) 2015 Arduino LLC.  All right reserved.

 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.

 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 See the GNU Lesser General Public License for more details.

 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "variant.h"
#include "WVariant.h"

//
// ADC helpers
//

/**
 * @brief ADC config struct shorthand
 * @param channel ADC channel number. 0 == ADC1_IN0, ...
 * @note device is always ADC1_device
 */
#define ADC(dev, ch)                    \
	{                              \
		.device = dev, .channel = ch \
	}

/**
 * @brief ADC config struct shorthand for no ADC function
 */
#define ADC_NONE                  \
	{                             \
		.device = 0, .channel = 0 \
	}

//
// TimerA helpers
//

/**
 * @brief TimerA config struct shorthand
 * @param uni TimerA unit. 1 == M4_TMRA1, ...
 * @param ch TimerA channel. 1 == TimeraCh1, ...
 * @param func TimerA GPIO function. 4 == Func_Tima0, 5 == Func_Tima1
 */
#define TIMA(uni, ch, func)                                      \
	{                                                            \
		.unit = uni, .channel = (ch - 1), .function = (func - 4) \
	}

/**
 * @brief TimerA config struct shorthand for no TimerA function
 */
#define TIMA_NONE                              \
	{                                          \
		.unit = 0, .channel = 0, .function = 0 \
	}

//
// Pin Map
//

extern const pin_info_t PIN_MAP[BOARD_NR_GPIO_PINS] = {
	//GPIO_PIN , GPIO_PORT, ADC&channel, TIM,
	// ---  PAx  ---
	{0,  GPIO_PORT_A, ADC(1, ADC_CH0), TIMA(2, 1, 4)},	// PA0
	{1,  GPIO_PORT_A, ADC(1, ADC_CH1), TIMA(2, 2, 4)},	// PA1
	{2,  GPIO_PORT_A, ADC(1, ADC_CH2), TIMA(5, 1, 5)},	// PA2
	{3,  GPIO_PORT_A, ADC(1, ADC_CH3), TIMA(5, 2, 5)},	// PA3
	{4,  GPIO_PORT_A, ADC(1, ADC_CH4), TIMA(3, 5, 5)},	// PA4
	{5,  GPIO_PORT_A, ADC(1, ADC_CH5), TIMA(3, 6, 5)},	// PA5
	{6,  GPIO_PORT_A, ADC(1, ADC_CH6), TIMA(3, 1, 5)},	// PA6
	{7,  GPIO_PORT_A, ADC(1, ADC_CH7), TIMA(3, 2, 5)},	// PA7
	{8,  GPIO_PORT_A, ADC_NONE, TIMA(1, 1, 4)},		// PA8
	{9,  GPIO_PORT_A, ADC_NONE, TIMA(1, 2, 4)},		// PA9
	{10, GPIO_PORT_A, ADC_NONE, TIMA(1, 3, 4)},		// PA10
	{11, GPIO_PORT_A, ADC_NONE, TIMA(1, 4, 4)},		// PA11
	{12, GPIO_PORT_A, ADC_NONE, TIMA(6, 1, 5)},		// PA12
	{13, GPIO_PORT_A, ADC_NONE, TIMA(6, 2, 5)},		// PA13
	{14, GPIO_PORT_A, ADC_NONE, TIMA(6, 3, 5)},		// PA14
	{15, GPIO_PORT_A, ADC_NONE, TIMA(6, 4, 5)},		// PA15
												//
												// ---  PBx  ---
												//
	{0,  GPIO_PORT_B, ADC(1, ADC_CH8), TIMA(3, 3, 5)},	// PB0
	{1,  GPIO_PORT_B, ADC(1, ADC_CH9), TIMA(3, 4, 5)},	// PB1
	{2,  GPIO_PORT_B, ADC_NONE, TIMA(1, 8, 4)},		// PB2
	{3,  GPIO_PORT_B, ADC_NONE, TIMA(6, 5, 5)},		// PB3
	{4,  GPIO_PORT_B, ADC_NONE, TIMA(6, 6, 5)},		// PB4
	{5,  GPIO_PORT_B, ADC_NONE, TIMA(6, 7, 5)},		// PB5
	{6,  GPIO_PORT_B, ADC_NONE, TIMA(6, 8, 5)},		// PB6
	{7,  GPIO_PORT_B, ADC_NONE, TIMA(4, 2, 4)},		// PB7
	{8,  GPIO_PORT_B, ADC_NONE, TIMA(4, 3, 4)},		// PB8
	{9,  GPIO_PORT_B, ADC_NONE, TIMA(4, 4, 4)},		// PB9
	{10, GPIO_PORT_B, ADC_NONE, TIMA(5, 8, 5)},		// PB10
	{11, GPIO_PORT_B, ADC_NONE, TIMA_NONE},			// PB11
	{12, GPIO_PORT_B, ADC_NONE, TIMA(1, 8, 4)},		// PB12
	{13, GPIO_PORT_B, ADC_NONE, TIMA(1, 5, 4)},		// PB13
	{14, GPIO_PORT_B, ADC_NONE, TIMA(1, 6, 4)},		// PB14
	{15, GPIO_PORT_B, ADC_NONE, TIMA(1, 7, 4)},		// PB15
												//
												// ---  PCx  ---
												//
	{0,  GPIO_PORT_C, ADC(1, ADC_CH10), TIMA(2, 5, 4)}, // PC0
	{1,  GPIO_PORT_C, ADC(1, ADC_CH11), TIMA(2, 6, 4)}, // PC1
	{2,  GPIO_PORT_C, ADC(1, ADC_CH12), TIMA(2, 7, 4)},	// PC2
	{3,  GPIO_PORT_C, ADC(1, ADC_CH13), TIMA(2, 8, 4)},	// PC3
	{4,  GPIO_PORT_C, ADC(1, ADC_CH14), TIMA(3, 7, 5)},	// PC4
	{5,  GPIO_PORT_C, ADC(1, ADC_CH15), TIMA(3, 8, 5)},	// PC5
	{6,  GPIO_PORT_C, ADC_NONE, TIMA(5, 8, 5)},		// PC6
	{7,  GPIO_PORT_C, ADC_NONE, TIMA(5, 7, 5)},		// PC7
	{8,  GPIO_PORT_C, ADC_NONE, TIMA(5, 6, 5)},		// PC8
	{9,  GPIO_PORT_C, ADC_NONE, TIMA(5, 5, 5)},		// PC9
	{10, GPIO_PORT_C, ADC_NONE, TIMA(5, 1, 5)},		// PC10
	{11, GPIO_PORT_C, ADC_NONE, TIMA(5, 2, 5)},		// PC11
	{12, GPIO_PORT_C, ADC_NONE, TIMA(5, 3, 5)},		// PC12
	{13, GPIO_PORT_C, ADC_NONE, TIMA(4, 8, 4)},		// PC13
	{14, GPIO_PORT_C, ADC_NONE, TIMA(4, 5, 4)},		// PC14 XTAL32_OUT
	{15, GPIO_PORT_C, ADC_NONE, TIMA(4, 6, 4)},		// PC15 XTAL32_IN
												//
												// ---  PDx  ---
												//
	#ifdef GPIO_D
	{0,  GPIO_PORT_D, ADC_NONE, TIMA(5, 4, 5)},		// PD0
	{1,  GPIO_PORT_D, ADC_NONE, TIMA(6, 5, 5)},		// PD1
	{2,  GPIO_PORT_D, ADC_NONE, TIMA(6, 6, 5)},		// PD2
	{3,  GPIO_PORT_D, ADC_NONE, TIMA(6, 7, 5)},		// PD3
	{4,  GPIO_PORT_D, ADC_NONE, TIMA(6, 8, 5)},		// PD4
	{5,  GPIO_PORT_D, ADC_NONE, TIMA_NONE},			// PD5
	{6,  GPIO_PORT_D, ADC_NONE, TIMA_NONE},			// PD6
	{7,  GPIO_PORT_D, ADC_NONE, TIMA_NONE},			// PD7
	{8,  GPIO_PORT_D, ADC_NONE, TIMA(6, 1, 5)},		// PD8
	{9,  GPIO_PORT_D, ADC_NONE, TIMA(6, 2, 5)},		// PD9
	{10, GPIO_PORT_D, ADC_NONE, TIMA(6, 3, 5)},		// PD10
	{11, GPIO_PORT_D, ADC_NONE, TIMA(6, 4, 5)},		// PD11
	{12, GPIO_PORT_D, ADC_NONE, TIMA(5, 5, 5)},		// PD12
	{13, GPIO_PORT_D, ADC_NONE, TIMA(5, 6, 5)},		// PD13
	{14, GPIO_PORT_D, ADC_NONE, TIMA(5, 7, 5)},		// PD14
	{15, GPIO_PORT_D, ADC_NONE, TIMA(5, 8, 5)},		// PD15
	#endif
												//
												// ---  PEx  ---
												//
	#ifdef GPIO_E
	{0,  GPIO_PORT_E, ADC_NONE, TIMA_NONE},			// PE0
	{1,  GPIO_PORT_E, ADC_NONE, TIMA_NONE},			// PE1
	{2,  GPIO_PORT_E, ADC_NONE, TIMA(3, 5, 4)},		// PE2
	{3,  GPIO_PORT_E, ADC_NONE, TIMA(3, 6, 4)},		// PE3
	{4,  GPIO_PORT_E, ADC_NONE, TIMA(3, 7, 4)},		// PE4
	{5,  GPIO_PORT_E, ADC_NONE, TIMA(3, 8, 4)},		// PE5
	{6,  GPIO_PORT_E, ADC_NONE, TIMA_NONE},			// PE6
	{7,  GPIO_PORT_E, ADC_NONE, TIMA_NONE},			// PE7
	{8,  GPIO_PORT_E, ADC_NONE, TIMA(1, 5, 4)},		// PE8
	{9,  GPIO_PORT_E, ADC_NONE, TIMA(1, 1, 4)},		// PE9
	{10, GPIO_PORT_E, ADC_NONE, TIMA(1, 6, 4)},		// PE10
	{11, GPIO_PORT_E, ADC_NONE, TIMA(1, 2, 4)},		// PE11
	{12, GPIO_PORT_E, ADC_NONE, TIMA(1, 7, 4)},		// PE12
	{13, GPIO_PORT_E, ADC_NONE, TIMA(1, 3, 4)},		// PE13
	{14, GPIO_PORT_E, ADC_NONE, TIMA(1, 4, 4)},		// PE14
	{15, GPIO_PORT_E, ADC_NONE, TIMA(1, 8, 4)},		// PE15
	#endif
												//
												// ---  PHx  ---
												//
	{0,  GPIO_PORT_H, ADC_NONE, TIMA(5, 3, 5)},		// PH0   XTAL_IN
	{1,  GPIO_PORT_H, ADC_NONE, TIMA(5, 4, 5)},		// PH1   XTAL_OUT
	{2,  GPIO_PORT_H, ADC_NONE, TIMA(4, 7, 4)},		// PH2
};
