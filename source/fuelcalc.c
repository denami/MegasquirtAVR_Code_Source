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


/*
TODO:
check rpmk

There is a timeout during acceleration, this should be fixed.
Is the same timeout appering in the B&G asm code?

*/


#include "global.h"
#include "fuelcalc.h"
#include "helpers.h"
#include "adc.h"
#include "tables.h"

#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <avr/pgmspace.h>
#include <string.h>

extern uint8_t PROGMEM KPAFACTOR4250[];
extern uint8_t PROGMEM KPAFACTOR4115[];
extern uint8_t PROGMEM BAROFAC4250[];
extern uint8_t PROGMEM BAROFAC4115[];
extern uint8_t PROGMEM THERMFACTOR[];
extern uint8_t PROGMEM AIRDENFACTOR[];
extern uint8_t PROGMEM THERMFACTOR[];

volatile extern struct squirt_t inj_port1, inj_port2;
volatile extern struct engine_t engine;
volatile extern struct time_t rtc;
volatile extern uint8_t sensors[];
extern struct config_t config;

volatile uint8_t tpsaclk; // TPS enrichment timer clock in 0.1 second resolution

uint8_t tpsaclkcmp; // Comparison value for TPS acceleration time - from lookup table
//uint8_t tpsfuelcut; // TPS Fuel Cut (percent)

volatile uint8_t egocount; // Counter value for EGO step - incremented every ignition pulse
volatile uint8_t asecount; // Counter value for after-start enrichment counter - every ignition pulse

struct corr_t corr;


void init_fuelcalc(void) {
  corr.ego = 100;
  corr.air = 100;
  corr.warm = 100;
  corr.baro = 100;
  corr.gammae = 100;
  corr.ve = 100;
  corr.tpsaccel = 0;
  corr.tpsfuelcut = 100;

}


/***************************************************************************/
/*** warm-up and after-start enrichment                                  ***/
/***************************************************************************/
void warmup_enrich(void) {
  uint8_t warm_enrich;
  uint8_t my_status;
  struct search_table_t st;

  /* обогащение на прогреве */
  cli();
  my_status = engine.status;
  if (my_status & (uint8_t) _BV(crank)) {
    my_status &= ~_BV(crank);
    my_status |= _BV(startw) | _BV(warmup);
    engine.status = my_status;
    asecount = 0;
  }
  sei();

#warning "wwurange should be located in flash"

  /* используем температуру двигателя для нахождения wwurange, */
  /* "упакуем" 256 возможных значений сигнала датчика в [0-9] шкалу */
  search_table(config.wwurange, sizeof(config.wwurange), engine.coolant, &st);

  /* затем, используя интерполяцию по wwu-table, */
  /* а также шкалу [0-9] для нахождения значения обогащения */
  warm_enrich = linear_interp(st.lbound, st.ubound, config.wwu[st.index-1], 
                              config.wwu[st.index], engine.coolant);

  if (warm_enrich > 100) {
    PORTA |= _BV(LED_WARMUP); /* зажечь светодиод - "прогрев" */
    
	cli();
    engine.status |= _BV(warmup);
    sei();
    
	/* обогащение после пуска двигателя */
	if (engine.status & _BV(startw)) {
        if (asecount <= config.awc) {
	      uint16_t wue;
	      wue = warm_enrich + linear_interp(0, config.awc, config.awev, 0, asecount);
	      if ((wue >> 8) != 0) /* проверка на переполнение */
	        wue = 0xFF;
	      warm_enrich = (uint8_t)wue;
        } else {
	      cli();
	      engine.status &= ~_BV(startw);
	      sei();
        }
    }
  } else {
    /* обогащения на прогреве закончилось */
    warm_enrich = 100;
    
	cli();
    engine.status &= ~( _BV(startw) | _BV(warmup) );
    sei();
    
	PORTA &= ~_BV(LED_WARMUP);  /* погасить свтодиод - "прогрев" */
  }

  corr.warm = warm_enrich;
}


/***************************************************************************/
/*** Throttle posistion acceleration enrichment                          ***/
/***************************************************************************/
void tps_acc_enrich(void) {
  uint8_t tps_diff;
  uint8_t my_tps, my_last_tps;
  
  cli();
  my_tps = engine.tps;
  my_last_tps = engine.last_tps;
  sei();

  if (my_tps < my_last_tps) {                             /* замедление */
    tps_diff = my_last_tps - my_tps;
    
    if (tps_diff >= config.tps_thresh) {                  /* скорость изменения положения дросселя выше порога? */

        if (engine.status & _BV(tpsaen)) {
		    corr.tpsfuelcut = 100;
		    corr.tpsaccel = 0;
		    cli();
		    engine.status &= ~( _BV(tpsaen) | _BV(tpsden) );
		    sei();
		    PORTA &= ~_BV(LED_ACCEL);                     /* погасить свтодиод - "ускорение" */
	  
        } else {                                          /* топливоограничение */
		    if (engine.rpm >= config.fuelcut_thres) {
			    corr.tpsfuelcut = config.tpsdq;
			    cli();
			    engine.status &= ~_BV(tpsaen);
			    engine.status |= _BV(tpsden);
			    sei();
			    PORTA &= ~_BV(LED_ACCEL);                 /* погасить свтодиод - "ускорение" */
		    }
        }
    } else {                                              /* топливоограничение выполнено */
        if ((engine.status & _BV(tpsden)) && (engine.rpm < config.fuelcut_thres)) {            
		    cli();
		    engine.status &= ~( _BV(tpsaen) | _BV(tpsden) );
		    sei();
		    corr.tpsfuelcut = 100;
		    corr.tpsaccel = 0;
      }
    }
  } else {                                                /* ускорение */
    tps_diff = my_tps - my_last_tps;
    if (tps_diff >= config.tps_thresh) {
    /*
	  Вычисление ускорения должно быть переделано, это - неточно.
      Это очень долго, и вычисление обогащения начинается снова.
	*/
      if (engine.status & _BV(tpsaen)) {
		uint8_t acc_temp_offset, acc_temp_mult, acc_enrich;
		uint16_t mtmp;
		struct search_table_t st;

		/* multiplier amount based on cold temperature */
		acc_temp_mult = linear_interp(0, 205, config.acmult, 100, engine.coolant);
	
		/* lookup table amount based on tpsdot */
		search_table(config.tpsdotrate, sizeof(config.tpsdotrate), tps_diff, &st);
		acc_enrich = linear_interp(st.lbound, st.ubound, config.tpsaq[st.index-1], config.tpsaq[st.index], tps_diff);

		/* calculate acc-enrichment, catch overflow */
		mtmp = mult_div100(acc_temp_mult, acc_enrich);
		if ((mtmp >> 8) != 0) // will the result fit in a byte?
			mtmp = 200;         // ... no!

		/* extra fuel due to temperature */
		acc_temp_offset = linear_interp(0, 205, config.tpsacold, 0, engine.coolant);

		mtmp += acc_temp_offset;
		if ((mtmp >> 8) != 0) // will the result fit in a byte?
			mtmp = 0xFE;        // ... no!
	
		if (mtmp > corr.tpsaccel) 
			corr.tpsaccel = (uint8_t)mtmp; // tpsaccel is the result of this function

		if ((engine.status & _BV(tpsden)) || (tpsaclk >= tpsaclkcmp))  {
			cli();
			engine.status &= ~( _BV(tpsaen) | _BV(tpsden) );
			sei();
			corr.tpsfuelcut = 100;
			corr.tpsaccel = 0;
			PORTA &= ~_BV(LED_ACCEL); 	/* погасить светодиод - "ускорение" */
		}
      
	  /* старт ускорения, инициализация некоторых переменных */
	  } else { 
		corr.tpsaccel = config.tpsaq[0];
		tpsaclk = 0;
		tpsaclkcmp = config.tpsasync;
		cli();
		engine.status |= _BV(tpsaen); 	/* установить флаг - ускорение */
		engine.status &= ~_BV(tpsden); 	/* сбросить флаг - замедление */
		sei();
		PORTA |= _BV(LED_ACCEL); 		/* зажечь светодиод - "ускорение" */
      }
    
	/* ускорение выполненно */
	} else { 
      if ((engine.status & _BV(tpsden)) || (tpsaclk >= tpsaclkcmp)) {
		cli();
		engine.status &= ~( _BV(tpsaen) | _BV(tpsden) );
		sei();
		corr.tpsfuelcut = 100;
		corr.tpsaccel = 0;
		PORTA &= ~_BV(LED_ACCEL);     	/* погасить свтодиод - "ускорение" */
      }
    }
  }
}


/***************************************************************************/
/*** Exhaust gas oxygen sensor measurement section                       ***/
/***************************************************************************/
void o2(void) {
  uint8_t limit, new_ego, o2_is_lean;

  if ( (config.egodelta == 0) ||                            /* EGO коррекция отключена */
       (engine.rpm < config.rpmoxlimit) ||                  /* обороты двигателя малы */
       (engine.status & (_BV(tpsaen) | _BV(tpsden))) ||     /* активно обогащение смеси */
       (engine.coolant < config.egotemp) ||                 /* температура двигателя мала */
       (engine.tps > O2_MAX_TPS) || 						/* тапка в пол! */
       (engine.status_ext & _BV(o2_not_ready)) ||           /* O2S не готов (не прогрет) */ 
       (engine.kpa > O2_MAX_KPA) ) {                        /* велика нагрузка на двигатель */

    corr.ego = 100;
    egocount = 0;

  } else {
    if (egocount > config.egocountcmp) {          /* расстояние (в событиях зажигания) между шагами EGO корректора */
      egocount = 0;

      // do we want variable AFR?
      // then search bin and interpolate(kpa)
    
        if (sensors[EGO] != config.voltoxtarget) { /* если равно, нет никакой необходимости в регулировании */
	        if (sensors[EGO] < config.voltoxtarget) {
	            if (config.config13 & _BV(O2_WB_SENSOR))
	                o2_is_lean = false;            /* характеристика ШДК имеет обратный наклон */
	            else
	                o2_is_lean = true;
	        } else {
	            if (config.config13 & _BV(O2_WB_SENSOR))
	                o2_is_lean = true;             /* характеристика ШДК имеет обратный наклон */
	            else
	                o2_is_lean = false;
	        }
	        if (o2_is_lean) {                      /* бедно */
	            limit = 100 + config.egolimit;
	            new_ego = corr.ego + config.egodelta;

	            if (new_ego > limit)
	                corr.ego = limit;
	            else
	                corr.ego = new_ego;
	        } else {      			               /* богато */
	            limit = 100 - config.egolimit;
	            new_ego = corr.ego - config.egodelta;

	            if (new_ego < limit)
	                corr.ego = limit;
	            else
	                corr.ego = new_ego;
	        }
        }
    }
  }
}


/***************************************************************************/
/*** VE table lookup                                                     ***/
/***************************************************************************/
void ve_table_lookup(void) {
  uint8_t ve_11, ve_12, ve_21, ve_22;
  uint8_t ve_low_kpa, ve_high_kpa;
  struct search_table_t kpa, rpm;

  if (config.config13 & _BV(CONTROL_STRATEGY)) { //Alpha-N
    engine.kpa = engine.tps;
  }

  search_table(config.kparangeve, sizeof(config.kparangeve), engine.kpa, &kpa);
  search_table(config.rpmrangeve, sizeof(config.rpmrangeve), engine.rpm, &rpm);

  ve_11 = *(config.VE+8*(kpa.index-1)+(rpm.index-1));
  ve_12 = *(config.VE+8*(kpa.index-1)+rpm.index);
  ve_21 = *(config.VE+8*kpa.index+(rpm.index-1));
  ve_22 = *(config.VE+8*kpa.index+rpm.index);
  
  ve_low_kpa = linear_interp(rpm.lbound, rpm.ubound, ve_11, ve_12, engine.rpm);
  ve_high_kpa = linear_interp(rpm.lbound, rpm.ubound, ve_21, ve_22, engine.rpm);

  corr.ve = linear_interp(kpa.lbound, kpa.ubound, ve_low_kpa, ve_high_kpa, engine.kpa);
}




/***************************************************************************/
/*** calc total enrichment                                               ***/
/***************************************************************************/
void calc_total_enrichment(void) {
  uint8_t fuel_tmp, batt_tmp, pw;
  uint8_t batt_high, batt_low;
  uint16_t res;

  // 8-bit x 16-bit multiplications
  res = (uint16_t)corr.warm;
  res = mult_div100(corr.tpsfuelcut, res);
  res = mult_div100(corr.air, res);
  res = mult_div100(corr.ego, res);
  res = mult_div100(corr.baro, res);

  if ((res >> 8) != 0)                             /* проверка на переполнение */
    corr.gammae = 0xFF;
  else
    corr.gammae = (uint8_t)res;                    /* используется для ведения логов */

  res = mult_div100(corr.ve, res);
  
  if (!(config.config13 & _BV(CONTROL_STRATEGY)))  // speed-density
    res = mult_div100(engine.kpa, res);

  res = mult_div100(config.req_fuel, res);

  if ((res >> 8) != 0)                             // противное переполнение "... синий экран смерти!"
    fuel_tmp = 0xFF;
  else
    fuel_tmp = (uint8_t)res;


  /* battery voltage compensation */
  /* вспомним, низкое напряжение бортовой сети только увеличивает время открытия форсунок */
  batt_low = config.injopen + config.battfac;

  if (config.injopen <= config.battfac)
    batt_high = 0;
  else
    batt_high = config.injopen - config.battfac;

#warning "this should be configurable via configuration struct"
  batt_tmp = linear_interp(61, 164, batt_low, batt_high, sensors[BATT]);

  // final pulsewidth calculation, wuhuw!
  if (fuel_tmp) {
    res = batt_tmp + fuel_tmp + corr.tpsaccel - config.injocfuel;
    if ((res >> 8) != 0)
      pw = 0xFF;
    else
      pw = (uint8_t)res;
  } else {
    pw = 0;
  }

  cli();
  inj_port1.pwcalc = pw;
  inj_port2.pwcalc = pw;
  sei();

}


/***************************************************************************/
/*** calc parameters                                                     ***/
/***************************************************************************/
void calc_parameters(void) {

  // Manifold Air Pressure in kiloPascals
  if (config.config11 & _BV(MAP_SENSOR))
    engine.kpa = PRG_RDB(&KPAFACTOR4250[sensors[MAP]]);
  else
    engine.kpa = PRG_RDB(&KPAFACTOR4115[sensors[MAP]]);

  // Barometric correction enabled?
  if (config.config13 & _BV(BARO_CORRECTION)) {
    uint8_t val = sensors[BARO];

    // If the mcu resets while the engine is running, we don't believe 
    // 20 kPa being the atmospheric pressure.
    if ( (val < (config.baro - config.dbaro)) || 
	 (val > (config.baro + config.dbaro)) ) {
#warning "MegaTune wants the raw sensor value and will not discover the cheat!"
      corr.baro = config.baro;
      cli();
      engine.status_ext |= _BV(baro_problem);
      sei();
    } else {
      if (config.config11 & _BV(MAP_SENSOR))
	corr.baro = PRG_RDB(&BAROFAC4250[sensors[BARO]]);
      else
	corr.baro = PRG_RDB(&BAROFAC4115[sensors[BARO]]);
    }
  } else {
    corr.baro = 100; // no, not enabled
  }

  // Coolant temperature in degrees F + 40
  engine.coolant = PRG_RDB(&THERMFACTOR[sensors[CLT]]);

  // Air Density Correction Factor
  corr.air = PRG_RDB(&AIRDENFACTOR[sensors[MAT]]);

  // Interpolate throttle position
  // this makes it really smooth to install the tps
  // Should the tps be scaled 0..255 or 0..100% ?
  engine.tps = linear_interp(config.tps_low, config.tps_high, 0, 255, sensors[TPS]);

}


/***************************************************************************/
/*** расчет rpm                                                          ***/
/***************************************************************************/
void calc_rpm(void) {
  uint16_t my_rpm, my_rpmk;

  /* необходимо чтобы оба байта переменнй были переданны атомарно */
  cli();
  my_rpm = engine.rpm_p;
  sei();

  my_rpmk = (config.rpmk_1 << 8) | config.rpmk_2;

  /* расчет rpm */
  if (my_rpm) {                  /* удостоверимся, что двигатель вращается */
      my_rpm = my_rpmk/my_rpm;

      if ((my_rpm >> 8) != 0)
	     engine.rpm = 255;       /* диапаона в 25500 об/мин " ... хватит каждому!" (Bill Gates '81) */
      else
	     engine.rpm = my_rpm & 0xFF;
  } else {
      engine.rpm = 0;
  }
}


/***************************************************************************/
/*** режим запуска двигателя                                             ***/
/***************************************************************************/
void cranking(void) {
  uint8_t pw;
  uint8_t my_status;

  cli();
  my_status = engine.status;
  my_status |= _BV(crank);
  my_status &= ~( _BV(startw) | _BV(warmup) );
  engine.status = my_status;
  sei();

  /* расчет обогащения при пуске */
  
  /* если TPS меньше 3 volt */
  if (engine.tps < FLOOD_CLEAR) { 
    
	/* расчет ширины импульса [cwl..cwh] и интерполяция по температуре ОЖ */
	pw = linear_interp(0, 205, config.cwl, config.cwh, engine.coolant);
    // battery correction?
  
  /* продувка камер сгорания*/
  } else { 
    pw = 0; 
  }
  
  cli();
  inj_port1.pwcalc = pw;
  inj_port2.pwcalc = pw;
  sei();
}


/***************************************************************************/
/*** primepulse                                                          ***/
/***************************************************************************/
void primepulse(void) {

  if (config.primep_cold || config.primep_warm) {
    uint8_t pw, tps;

    tps = linear_interp(config.tps_low, config.tps_high, 0, 255, sensors[TPS]);

    // don't prime if the engine is flooded
    if (tps < FLOOD_CLEAR) {
      pw = linear_interp(0, 205, config.primep_cold, config.primep_warm, PRG_RDB(&THERMFACTOR[sensors[CLT]]));
      // battery correction?
      inj_port1.pw = pw;
      inj_port2.pw = pw;
      inj_port1.pwrun = 0;
      inj_port2.pwrun = 0;
      inj_port1.status |= _BV(enabled) | _BV(scheduled);
      inj_port2.status |= _BV(enabled) | _BV(scheduled);
      engine.status |= _BV(running);
    }
  }
}
