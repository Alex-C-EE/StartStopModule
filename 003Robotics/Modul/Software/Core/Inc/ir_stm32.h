/*
 * ir_stm32.h
 *
 * Created on: 25 dec 2009
 * Author: benjamin
 * Ported to STM32C0 by Gemini
 *
 * This file replaces the LPC1769-specific ir.h and maps the
 * IR driver logic to the STM32C0 HAL.
 */

#ifndef IR_STM32_H_
#define IR_STM32_H_

#include "main.h"
#include <stdint.h>

/*
 * Get the main timer handle from main.c
 */
extern TIM_HandleTypeDef htim1;

/*
 * Proocol
 */
#define IR_RC5_PROTOCOL

/*
 * HW-dependent stuff for STM32C0
 */

// --- Helper Functions (implemented in ir_rc5.c) ---
void IR_SetFallingEdgeInterrupt(void);
void IR_SetRisingEdgeInterrupt(void);

// --- Macro Remapping ---
#define IR_INT_INIT()		IR_SetFallingEdgeInterrupt() // Start by looking for a falling edge

#define IR_FALLING_INT()	IR_SetFallingEdgeInterrupt()
#define IR_RISING_INT()		IR_SetRisingEdgeInterrupt()

#define IR_TIMER_INIT()		HAL_TIM_Base_Start_IT(&htim1)

// Get/Set the timer counter
#define IR_CNT				(__HAL_TIM_GET_COUNTER(&htim1))
#define IR_CNT_SET(val)		__HAL_TIM_SET_COUNTER(&htim1, (val))

// ISR function prototypes (called from HAL callbacks in main.c)
#define IR_INT_ISR			void process_ir_pin_interrupt(void)
#define IR_CNT_ISR			void process_ir_timer_timeout(void)

// Interrupt clear macros (handled by HAL, so they are empty)
#define IR_CLR_INT_ISR()	{/* Cleared by HAL_GPIO_EXTI_IRQHandler */}
#define IR_CLR_TIM_ISR()	{/* Cleared by HAL_TIM_IRQHandler */}

// Timer clock frequency (from MX_TIM1_Init: 48MHz / (47+1) = 1MHz)
#define IR_COUNTER_CLOCK	1000000UL

// Pin reading macros
#define IR_GET_EDGE()		HAL_GPIO_ReadPin(IN_IR_GPIO_Port, IN_IR_Pin)
#define IR_EDGE_LOW()		(HAL_GPIO_ReadPin(IN_IR_GPIO_Port, IN_IR_Pin) == GPIO_PIN_RESET)
#define IR_EDGE_HIGH()		(HAL_GPIO_ReadPin(IN_IR_GPIO_Port, IN_IR_Pin) == GPIO_PIN_SET)
// ------------- END HW-stuff -------------------- //

/*
 * Some macros
 */
#ifndef _BV
#define	_BV(bit) 						(1<<(bit))
#endif

/*
 * Debugging
 */
#define IR_DBG_EN	0

/*
 * Buffer size
 */
#define IR_BUFFER_SIZE 20


#ifdef	IR_RC5_PROTOCOL

// RC5 specific settings
#define IR_RC5_DISABLE_REPEATS	1

typedef struct {
	uint8_t address_low;
	uint8_t command;
} IRDATA;

#if IR_DBG_EN
extern volatile unsigned int 	cnt_start2,
								cnt_toggle,
								cnt_addr,
								cnt_cmd;
#endif

/*
 * Functions (from ir_rc5.c)
 */
void ir_init(void);
signed char ir_has_next(void);
void ir_get_next(IRDATA *data);
unsigned char ir_get_repeats(void);
void ir_set_data_handler(void(*func)(IRDATA *data));
void ir_remove_data_handler(void);

#endif

/*
 * RC5 protocol standard addresses and commands
 * (Copied from original ir.h)
 */
#define RC5_ADR_TV1				0x00
#define RC5_ADR_TV2				0x01
#define RC5_ADR_TELETEXT		0x02
#define RC5_ADR_VIDEO			0x03
#define RC5_ADR_LV1				0x04
#define RC5_ADR_VCR1			0x05
#define RC5_ADR_VCR2			0x06
#define RC5_ADR_EXPERIMENTAL	0x07
#define RC5_ADR_SAT1			0x08
#define RC5_ADR_CAMERA			0x09
#define RC5_ADR_SAT2			0x0A
#define RC5_ADR_CDV				0x0C
#define RC5_ADR_CAMCORDER		0x0D
#define RC5_ADR_PREAMP			0x10
#define RC5_ADR_TUNER			0x11
#define RC5_ADR_RECORDER1		0x12
#define RC5_ADR_PREAMP2			0x13
#define RC5_ADR_CDPLAYER		0x14
#define RC5_ADR_PHONO			0x15
#define RC5_ADR_SATA			0x16
#define RC5_ADR_RECORDER2		0x17
#define RC5_ADR_CDR				0x1A
#define RC5_ADR_LIGHTING		0x1D
#define RC5_ADR_LIGHTING2		0x1E
#define RC5_ADR_PHONE			0x1F

#define RC5_CMD_0				0x00
#define RC5_CMD_1				0x01
#define RC5_CMD_2				0x02
#define RC5_CMD_3				0x03
#define RC5_CMD_4				0x04
#define RC5_CMD_5				0x05
#define RC5_CMD_6				0x06
#define RC5_CMD_7				0x07
#define RC5_CMD_8				0x08
#define RC5_CMD_9				0x09
#define RC5_CMD_MIN				0x0A
#define RC5_CMD_STANDBY			0x0C
#define RC5_CMD_MUTE			0x0D
#define RC5_CMD_VPLUS			0x10
#define RC5_CMD_VMIN			0x11
#define RC5_CMD_BPLUS			0x12
#define RC5_CMD_BMIN			0x13
#define RC5_CMD_PPLUS			0x20
#define RC5_CMD_PMIN			0x21
#define RC5_CMD_FRWD			0x32
#define RC5_CMD_FFWD			0x34
#define RC5_CMD_PLAY			0x35
#define RC5_CMD_STOP			0x36
#define RC5_CMD_RECORDING		0x37

#endif /* IR_STM32_H_ */
