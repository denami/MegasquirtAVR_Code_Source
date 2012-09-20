/*********************************************************\
*  AVR MegaSquirt - 2003                                  *
*  (C) 2003 [Michael Kristensen, mik@caffrey.dk]          *
***********************************************************
*  -- History --                                          *
*    release 0.1 - 06-03-2003                             *
*            http://caffrey.dk/megasquirt                 *
*    initial Motorola version by Bowling & Grippo (2002)  *
*            http://www.bgsoflex.com/megasquirt.html      *
\*********************************************************/


#include "global.h"
#include "actuators.h"
#include "helpers.h"

#include <inttypes.h>
#include <avr/io.h>
#include <stdlib.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>


volatile extern struct engine_t engine;
volatile extern uint8_t sensors[];
volatile extern struct config_t config;

volatile struct step_t step; // stepper motor variables



/***************************************************************************/
/*** вентилятор радиатора ОЖ                                             ***/
/***************************************************************************/
void coolant_fan(void) {
  if (engine.coolant > config.fan_temp)
    PORTB |= _BV(CLT_FAN);              // start fan
  else if (engine.coolant <= (config.fan_temp - config.fan_hyst))
    PORTB &= ~_BV(CLT_FAN); 			// stop fan
}

/***************************************************************************/
/*** check fast idle                                                     ***/
/***************************************************************************/
void check_fast_idle(void) {

  // ths FAST_IDLE_THRES avoids erratic idle when coolant == fastidle
  if (engine.coolant < (config.fastidle - FAST_IDLE_THRES)) {
    PORTB |= _BV(IDLEVALVE); //idlevalve on
    step.rpm_dev = 3;
	//step.dest_idle_rpm = config.iac_cold_idle;
  } else if (engine.coolant >= config.fastidle) {
    PORTB &= ~_BV(IDLEVALVE); //idlevalve off
    step.rpm_dev = 2;
	//step.dest_idle_rpm = config.iac_warm_idle;
  }

  // hardcoded threshold value, testing only!
  //step.rpm_dev = 2; //abs(config.iac_cold_idle - config.iac_warm_idle);

  step.dest_idle_rpm = linear_interp(0, 				/* установить желаемые обороты ХХ */
									 205, 
									 config.iac_cold_idle, 
									 config.iac_warm_idle, 
									 engine.coolant);

}

/***************************************************************************/
/*** init idle stepper                                                   ***/
/***************************************************************************/
void init_idle_stepper(void) {
  step.status = _BV(busy) | _BV(sync);
  step.status &= ~_BV(direction); 
  step.count = config.iac_max_close;					/* закрыть полностью клапан РХХ */ 
  
  step.dest_idle_rpm = config.iac_cold_idle; 			/* установить желаемые обороты ХХ */

  step.backoff = linear_interp(0, 						/* открыть клапан на нужное число шагов */
							   205, 
							   config.iac_backoff_cold, 
							   config.iac_backoff_warm, 
							   engine.coolant); 

}

/***************************************************************************/
/*** move idle stepper                                                   ***/
/***************************************************************************/
void move_idle_stepper(void) {
  static uint8_t phase;

  if (step.count) {							// требуется перемещение клапана РХХ
    step.count--;
    
	if (step.status & _BV(direction)) {  	// закрытие клапана 
		phase--;
    } else { 								// открытие клапана
		phase++;
    }
    
	PORTC = next_step(phase & 0x03) | _BV(STEP_EN);

    if (step.count == 0) {
		step.status &= ~_BV(busy);
    }
  
  } else {									// перемешение кланана РХХ не требуется, удерживать позицию
    step.status &= ~_BV(busy);
    
	if (config.iac_conf & _BV(power_off_iac))
		PORTC &= ~(_BV(STEP_EN) | _BV(STEP_A) | _BV(STEP_B) | _BV(STEP_C) | _BV(STEP_D)); 
  }
}

uint8_t PROGMEM STEP_PINS[] = { _BV(STEP_A), _BV(STEP_B), _BV(STEP_C), _BV(STEP_D) };

/***************************************************************************/
/*** next step                                                           ***/
/***************************************************************************/
// Allow allmost total reconfiguration of the steppers windings connections 
// through PC configuration program. Nice for people too lazy for soldering the
// stepper wires correctly!
uint8_t next_step(uint8_t phase) {
  uint8_t val, idx;

  switch (phase) {
	case 0: idx = config.iac_step_seq & 0x03; break;
	case 1: idx = (config.iac_step_seq >> 2) & 0x03; break;
	case 2: idx = (config.iac_step_seq >> 4) & 0x03; break;
	case 3: idx = (config.iac_step_seq >> 6) & 0x03; break;
	default: idx = 0; break;
  }
  
  val = PRG_RDB(&STEP_PINS[idx]);
  return val;
}

