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


#include "megasquirt.h"
#include "global.h"
#include "comm.h"
#include "fuelcalc.h"
#include "helpers.h"
#include "storage.h"
#include "adc.h"
#include "actuators.h"
#include "tables.h"

#include <avr/io.h>
#include <inttypes.h>
#include <avr/wdt.h>
#include <stdlib.h>
#include <avr/interrupt.h>
#include <avr/sfr_defs.h>
#include <avr/pgmspace.h>


/*
TODO:
  Check reset source at power-up
  Watchdog timer 
*/


extern uint8_t sensors[];
extern volatile struct squirt_t inj_port1, inj_port2;
extern volatile struct engine_t engine;
extern volatile struct config_t config;

extern uint8_t tpsfuelcut;

volatile uint8_t alive;
extern volatile uint8_t crank_enable_timeout;

int main(void) {

  /* инициализация портов IO */
  PORTA = 0x0F; // misc LEDs
  DDRA = 0xF8;

  PORTB = 0x63; // pwm active low
  DDRB = 0xFC;

  PORTC = 0xE0; // stepmotor
  DDRC = 0x1F;

  PORTD = 0xFF;
  DDRD = 0x00;
  
  PORTE = 0xFF;
  DDRE = 0x00;

  PORTF = 0x00; // adc channels
  DDRF = 0x00;

  PORTG = 0xFF; // unused
  DDRG = 0x18;

  /* инициализация датчиков и оперативной информации */ 
  initSensors();

  /*
    инициализация таймеров
    установить для timer0 прерывание с частотой 10 kHz
  */
  TCCR0 = _BV(WGM01) | _BV(CS01); 				// CTC, div8
  OCR0 = 200;
  TIMSK = _BV(OCIE0);  							//output-compare interrupt

  /* 
    инициализация pwm  
    Set OCnA/OCnB/OCnC on compare match, clear OCnA/OCnB/OCnC at TOP
    fast pwm mode, TOP is stored in ICR1
    TCCR1A = _BV(COM1A1) | _BV(COM1A0) | _BV(COM1B1) | _BV(COM1B0) | _BV(WGM11);
  */
  TCCR1A = _BV(WGM11);
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS10);
  ICR1 = PWM_FREQUENCY; 						// change pwm frequency

  /* характеризация внешнего прерывания */ 
  EICRB = _BV(ISC61);  							/* спадающий фронт */ 
  EIMSK = _BV(INT6);   							/* разрешить прерывание INT6 */

  /* инициализация канала связи (rs232) */
  initUART();

  /* инициализация глобальных переменных */ 
  engine.status = 0;
  engine.status_ext = _BV(o2_not_ready) | _BV(crank_enable);
  engine.rpm = 0;
  engine.rpm_p = 0;

  inj_port1.pw = 0;
  inj_port2.pw = 0;

  crank_enable_timeout = CRANK_TIMEOUT;

  /* */
  init_fuelcalc();

  /* загрузить данные из eeprom в sram */
  loadConfig();

  // get current barometric pressure
  sensors[BARO] = sensors[MAP];

  /* fully close stepper so position is known */
  init_idle_stepper();

  // how about priming the fuelpump for a couple of seconds first
  PORTB |= _BV(FUELPUMP); /* включить топливный насос */

  // We have pressure, the fuelpump is really fast!

  // Do a temperature interpolation and schedule a priming pulse
  primepulse();

  /* разрешаем прерывания */ 
  sei();

  /* главный цикл программы*/
  for (;;) {
    calc_parameters();
    
	/* управление вентилятором радиатора ОЖ */
	coolant_fan(); 
    
	/* */
	check_fast_idle();

    if (engine.status & _BV(running)) {

		/* если двигатель вращается */
		if (engine.rpm_p) {

			/* расчитываем rpm только если это необходимо*/ 
			if (engine.status_ext & _BV(new_rpm)) {
				cli();
				engine.status_ext &= ~_BV(new_rpm);
				sei();
				calc_rpm();
			}

			/*
			  вход в режим пуска только по включению питания, 
			  или спустя CRANK_TIMEOUT секунд после выхода из режима пуска, 
			  или если обороты двигателя (rpm) упали до нуля.
			*/
			if ((engine.status_ext & _BV(crank_enable)) && (engine.rpm <= config.cranking_thres)) {
				cranking();
				cli();
				engine.status_ext &= ~_BV(left_crankmode); /* флаг, выход из режима пуска */
				crank_enable_timeout = CRANK_TIMEOUT;
				sei();
			} else {    								   /* двигатель работает */ 
				cli();
				engine.status_ext |= _BV(left_crankmode);  /* флаг, выход из режима пуска */
				sei();
				// check if idle ?
				warmup_enrich();
				tps_acc_enrich();
				o2();
				ve_table_lookup();
				calc_total_enrichment();
			}
		}
    }

	alive = 1;  /* указывает, что достигнут конец главного цикла */
  }

  return (0);
}

