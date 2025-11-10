/*
 * start.h
 *
 * Created on: 20 jan 2012
 * Author: benjamin
 *
 * Ported to STM32C0 by Gemini
 */

#ifndef START_H_
#define START_H_

#include "main.h" // Include base STM32 definitions
#include "ir_stm32.h" // Get RC5 definitions

/*
 * Settings
 */
// Home_mode or competition_mode?
#define IS_COMP_MODE			1

// Home-mode commands
#define START_HOME_START		RC5_CMD_1
#define START_HOME_STOP			RC5_CMD_2

// Default STOP command (before something else is programmed)
#define START_DEFAULT_STOP_CMD	0x04

// IR address for competition mode
#define START_ADDRESS_COMP		RC5_ADR_EXPERIMENTAL

// IR address for programming
#define START_ADDRESS_PROG		0x0B

// Delay between STOPPED_SAFE and STOPPED in 1s/100
// (This is based on the 100Hz tick from start_timerfunc)
#define START_STOP_DELAY		100 // 100 * 10ms = 1000ms = 1 second

/*
 * States
 */
#define START_STATE_POWER_ON		0
#define START_STATE_STARTED			1
#define START_STATE_STOPPED_SAFE	2
#define START_STATE_STOPPED			3

/*
 * Non-volatile data
 */
typedef struct {
	char state;
	char stop_cmd;
} START_DATA;

/*
 * Functions
 */
char start_get_state(void);
char start_get_saved_state(void);
char start_get_stop_cmd(void);
void start_set_home_mode(void);
void start_set_competition_mode(void);
START_DATA* start_get_saved_data(void);
void start_timerfunc(void);
char start_init(void(*save_func)(START_DATA *data),
				void(*load_func)(START_DATA *data),
				void(*programming_done)(char stop_cmd),
				void(*state_change_func)(char state));

#endif /* START_H_ */
