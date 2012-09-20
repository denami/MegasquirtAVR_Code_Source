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


#include "fuelcontrol.h"
#include "fuelcalc.h"
#include "global.h"
#include "adc.h"
#include "comm.h"
#include "actuators.h"

#include <avr/io.h>
#include <inttypes.h>
#include <avr/interrupt.h>
#include <avr/sfr_defs.h>


extern uint8_t sensors[];

volatile struct squirt_t inj_port1, inj_port2;
volatile struct engine_t engine;
struct time_t rtc;

extern struct config_t config;
extern struct step_t step;

extern uint8_t alive;
uint8_t led_timeout;

extern uint8_t tpsaclk;
extern uint8_t egocount;
extern uint8_t asecount;

volatile uint8_t crank_enable_timeout;

/***************************************************************************/
/*** INTERRUPT: Timer @ 0.1 ms ***/
/***************************************************************************/
// this is a long and painful interrupt handler!
SIGNAL (SIG_OUTPUT_COMPARE0) {

  //start new injection, channel 1
  if (inj_port1.status & _BV(scheduled)) {
    inj_port1.status &= ~_BV(scheduled);
    inj_port1.status |= _BV(firing) | _BV(enabled);
    PORTA |= _BV(LED_SQUIRT);  // sled on
    PORTB &= ~_BV(PWM_A);      // inject1 active
    PORTG |= _BV(FLYBACK_A);   // flyback enable
  }

  //start new injection, channel 2
  if (inj_port2.status & _BV(scheduled)) {
    inj_port2.status &= ~_BV(scheduled);
    inj_port2.status |= _BV(firing) | _BV(enabled);
    PORTA |= _BV(LED_SQUIRT);  // sled on
    PORTB &= ~_BV(PWM_B);      // inject2 active
    PORTG |= _BV(FLYBACK_B);   // flyback enable
  }

  //check for end of injection, channel 1
  if (inj_port1.status & _BV(firing)) {
    if (inj_port1.pwrun >= inj_port1.pw) {
      inj_port1.status &= ~(_BV(firing) | _BV(enabled) | _BV(scheduled));
      TCCR1A &= ~( _BV(COM1A1) | _BV(COM1A0) );	// disable pwm
      PORTB |= _BV(PWM_A);                      // inject1 deactive
      PORTG &= ~_BV(FLYBACK_A);                 // flyback disable
    } else if (inj_port1.pwrun == inj_port1.pwm_delay) {
      OCR1A = inj_port1.pwm_dc * 10;  // pwm duty cycle, multiply by 10 to increase range
      TCCR1A |= _BV(COM1A1) | _BV(COM1A0);	// enable pwm
    }
    inj_port1.pwrun++;
  }

  //check for end of injection, channel 2
  if (inj_port2.status & _BV(firing)) {
    if (inj_port2.pwrun >= inj_port2.pw) {
      inj_port2.status &= ~(_BV(firing) | _BV(enabled) | _BV(scheduled));
      TCCR1A &= ~( _BV(COM1B1) | _BV(COM1B0) );	// disable pwm
      PORTB |= _BV(PWM_B);                      // inject2 deactive
      PORTG &= ~_BV(FLYBACK_B);                 // flyback disable
    } else if (inj_port2.pwrun == inj_port2.pwm_delay) {
      OCR1B = inj_port2.pwm_dc * 10;  // pwm duty cycle, multiply by 10 to increase range
      TCCR1A |= _BV(COM1B1) | _BV(COM1B0);	// enable pwm
    }
    inj_port2.pwrun++;
  }


  /* двигатель все еще вращается? */
  if (engine.status & _BV(running)) {
    
	if (! ((inj_port1.status & _BV(enabled)) || (inj_port2.status & _BV(enabled))) ) {
      PORTA &= ~_BV(LED_SQUIRT); //sled off
    } 

    //filter noise on interrupt6, eg. wait some time before reenabling the interrupt
    if (engine.rpm_c == (engine.rpm_p >> 1)) {
      EIFR |= _BV(INTF6);  // clear interrupt flag
      EIMSK |= _BV(INT6);  // enable external interrupt
    }

    // update the rpm counter
    engine.rpm_c++;
    
	/* период выше порога (2.5 sec) */
	if ((engine.rpm_c>>8) >= 100) { 

	  /* двигатель остановлен, сбросить все флаги */
	  engine.status = 0; 
      /* установить флаги: О2 не готов, режим пуска */
	  engine.status_ext |= _BV(o2_not_ready) | _BV(crank_enable);
      /* сбросить флаг: выход из режима пуска */
	  engine.status_ext &= ~_BV(left_crankmode);
      crank_enable_timeout = CRANK_TIMEOUT;

      engine.rpm_c = 0;
      engine.rpm = 0;
	  
	  /* fuelpump off, idlevalve off */
      PORTB &= ~( _BV(FUELPUMP) | _BV(IDLEVALVE) ); 	
      /* sled off, wled off */
	  PORTA &= ~( _BV(LED_SQUIRT) | _BV(LED_WARMUP) );   

      inj_port1.pw = 0;
      inj_port2.pw = 0;
    }
  /* not running, enable the interrupt */
  } else { 
    EIMSK |= _BV(INT6);     //enable external interrupt
  }

  // RTC
  if (++rtc.tick == 10) { // every ms
    static uint8_t step_clk, adc_clk;

    rtc.tick=0;
    statusLed();

    if (++adc_clk == ADC_PERIOD) {
        adc_clk=0;
        startADC(); //start adc
    }

    // acc. clock
    if (++rtc.tsec == 100) {  // 1/10 second
        rtc.tsec=0;
        tpsaclk++;
        engine.last_tps = engine.tps;

      // stepper movement calculations
      // this will fast get the engine idle speed somewhat near the desired idle rpm

        if (!(config.iac_conf & _BV(disable_iac))) {
	        if (!(step.status & _BV(busy))) {
	            if (step.status & _BV(sync)) { 
					/* клапан сейчас полностью закрыт, необходимо его сколько-то открыть */
	                step.status &= ~_BV(sync);
	                step.status |= _BV(busy) | _BV(direction);
	                step.count = step.backoff; /* количество шагов на открытие клапана */
	            } else {
	                if ((engine.status & _BV(tpsden)) && (!(step.status & _BV(decel)))) {
	                    step.status |= _BV(decel) | _BV(busy);
						step.status &= ~_BV(direction);
	                    step.count = config.iac_decel;
	                } else {
	                    if (engine.tps <= config.iac_skip) { 
							/* дроссельная заслонка закрыта */
		                    if (engine.rpm <= (step.dest_idle_rpm - step.rpm_dev)) {
		                        /* обороты слишком малы, открытие клапана РХХ */
		                        step.count = config.iac_step_coarse;
		                        step.status |= _BV(direction);
		                        step.status |= _BV(busy);
		                    } else {
		                        if (engine.rpm >= (step.dest_idle_rpm + 2*step.rpm_dev)) {
		                            /* обороты слишком велики, закрытие клапана РХХ */
		                            step.count = config.iac_step_coarse;
		                            step.status &= ~_BV(direction);
		                            step.status |= _BV(busy);
		                        }
		                    }
	                    }
	                }
	            }
	        }
        }
    }

    // seconds
    if (++rtc.ms == 1000) {
        rtc.ms=0;
        rtc.sec++;

        // stepper movement calculations
        // this will slowly get the engine idle speed hitting desired idle rpm
 
       if (!(config.iac_conf & _BV(disable_iac))) {
	        if (!(step.status & _BV(busy))) {
	            if (engine.tps <= config.iac_skip) {
					/* дроссельная заслонка закрыта */
	                if (engine.rpm < step.dest_idle_rpm) { 
						/* обороты слишком малы, открытие клапана РХХ */
	                    step.count = 2*config.iac_step_fine;
	                    step.status |= _BV(direction);
	                    step.status |= _BV(busy);
	                } else {
	                    if (engine.rpm > step.dest_idle_rpm) { 
							/* обороты слишком велики, закрытие клапана РХХ */
		                    step.count = config.iac_step_fine;
		                    step.status &= ~_BV(direction);
		                    step.status |= _BV(busy);
	                    } else {
		                    step.status &= ~_BV(decel);
	                    }
	                }
	            }
	        }
        }
      
        if (engine.status_ext & _BV(left_crankmode)) {
	        if (crank_enable_timeout)
	            crank_enable_timeout--;
	        else
	            engine.status_ext &= ~_BV(crank_enable);
        }

        if (engine.status_ext & _BV(o2_not_ready)) {
	        if ((rtc.sec == 30) && (engine.status & _BV(running)))
	            engine.status_ext &= ~_BV(o2_not_ready);
        }
    }

    // act
    if (step_clk++ >= ((config.iac_conf & 0xF0)>>4)) {
        step_clk=0;
            if ((engine.status & _BV(running)) || (config.iac_conf & _BV(debug_iac)))
	            move_idle_stepper();
    }
  }
}


/***************************************************************************/
/*** INTERRUPT: ignition (triggers on falling edge) ***/
/***************************************************************************/
SIGNAL(SIG_INTERRUPT6) {
  uint8_t sched_squirt;
  static uint8_t altcount; 	// Alternate count selector
  static uint8_t igncount; 	/*  счетчик импульсов событий зажигания */

  EIMSK &= ~_BV(INT6);   	// disable external interrupt

  if (asecount < 255) 
    asecount++;

  if (egocount < 255)
    egocount++;

  /* лучше конечно использовать захват по входу или читать значение таймера вместо этого */ 
#warning "do we want this to increase or decrease?"
  engine.drpm_p = engine.rpm_c - engine.rpm_p; 	// used by idle PID
  engine.rpm_p = engine.rpm_c;
  engine.rpm_c = 0;
  engine.status_ext |= _BV(new_rpm);  			// start a rpm calculation

  PORTB |= _BV(FUELPUMP); 						// fuelpump on
  engine.status |= _BV(running);

  if (engine.status & _BV(crank)) {
    engine.status &= ~(_BV(tpsaen) | _BV(tpsden));
    sched_squirt = true;
  } else {
    igncount++;
    if (igncount == config.divider) {
      sched_squirt = true;
    } else {
      sched_squirt = false;
      if (igncount == 0x08) {
	igncount = 0;
      }
    }
  }

  if (sched_squirt) {
    igncount = 0;

    if ( (engine.status & _BV(crank)) || (config.alternate == 0) ) {

      //schedule both injectors
      inj_port1.pw = inj_port1.pwcalc;
      inj_port2.pw = inj_port2.pwcalc;

      inj_port1.pwrun = 0;
      inj_port2.pwrun = 0;

      inj_port1.status |= _BV(scheduled) | _BV(enabled);
      inj_port2.status |= _BV(scheduled) | _BV(enabled);

    } else {
      altcount++;
      if (altcount & 0x01) {
		inj_port2.pw = inj_port2.pwcalc;
		inj_port2.pwrun = 0;
		inj_port2.status |= _BV(scheduled) | _BV(enabled);
      } else {
		inj_port1.pw = inj_port1.pwcalc;
		inj_port1.pwrun = 0;
		inj_port1.status |= _BV(scheduled) | _BV(enabled);
      }
    }
  }
}


/*
  Светодиод состояния EFI, указывает, что прерывание по таймеру является активным,
  и основной цикл продолжает выполняться (CPU не завис). Вызывается из прерывания.
*/
void statusLed(void) {
  static uint8_t alivecount;

	if ((alive == 1) && (alivecount == 0)) {
		alive = 0;
		alivecount++;
		PORTA &= ~_BV(LED_STATUS);  		/* светодиод вкл. */
	} else {
		if (alivecount == 20) {
			PORTA |= _BV(LED_STATUS); 		/* светодиод выкл. */
		}

		if (alivecount) { 					/* считать только когда разрешено */
			alivecount++;

			if (engine.status_ext & _BV(o2_not_ready)) {
				if (alivecount == 125)
				alivecount=0; 				/* быстрое мигание в режиме прогрева O2S */
			}
		}
	}
}
