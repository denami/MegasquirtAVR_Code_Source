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

// Type declarations global to everything

#ifndef __GLOBAL_H__
#define __GLOBAL_H__

#include <inttypes.h>

#ifndef true 
#define true 1
#endif

#ifndef false
#define false 0
#endif

/***********************************************************************/

struct time_t {
  uint8_t tick;  // ms /10
  uint16_t ms;   // ms
  uint8_t tsec;  // seconds /10
  uint16_t sec;  // seconds
};

/***********************************************************************/

#define enabled   0 //squirt
#define scheduled 1 //sched
#define firing    2 //firing

struct squirt_t {
  uint8_t status;    // uses the defines above
  uint8_t pw;        // injector squirt time in 1/10 millesec (0 to 25.5 ms)
  uint8_t pwrun;     // Pulsewidth timing variable - from 0 to 25.5ms
  uint8_t pwcalc;    // next pulsewidth
  uint8_t pwm_dc;    // pwm duty cycle (injpwm)
  uint8_t pwm_delay; // delay (0.1 ms) before going to pwm-mode (injpwmt)
};

/***********************************************************************/

#define busy      0
#define direction 2  // 0 == close, 1 == open
#define decel     3
#define sync      4

struct step_t {
  uint8_t status;        // как определено выше
  uint8_t count;         // сколько шагов требуется
  uint8_t dest_idle_rpm; // желаемые обороты ХХ
  uint8_t rpm_dev;       // максимальная девиация целевых оборотов при грубом регулировании
  uint8_t backoff;       // необходимое количество шагов на прикрывание клапана после инициализации
  int8_t corr;
  //  int16_t position;
  //  uint8_t rpm_low;
  //  uint8_t rpm_high;
};

/***********************************************************************/

//status
#define running 0  // engine running
#define crank   1  // cranking
#define startw  2  // start warmup enrichment
#define warmup  3  // warmup
#define tpsaen  4  // tps acceleration
#define tpsden  5  // tps deceleration
//#define mapaen  6  // map acceleration

//status_ext
#define new_rpm        0  // need for recalculation of rpm
#define baro_problem   1  // somehow the barometric pressure is way off
#define o2_not_ready   2  // this is set for the first 30 seconds
#define crank_enable   4  // allowed to enter crank mode
#define left_crankmode 5  // _has been_ in crank mode

struct engine_t {
  uint8_t status;      // uses the defines above
  uint8_t status_ext;  // uses the defines above (below)
  uint8_t kpa;         // MAP value in units of KPa
  uint8_t coolant;     // Coolant temperature in Degrees F plus 40 (allows -40 degress to fit in integer)
  uint8_t batt;        // current battery voltage (compensate for unprecise voltagedivider)
  uint8_t tps;         // throttle posision (percent)
  uint8_t last_tps;    // throttle posision (percent)
  uint16_t rpm_p;      // rpm period
  uint16_t rpm_c;      // rpm counter
  int16_t drpm_p;      // change in rpm_p
  uint8_t rpm;         // Computed engine RPM - rpm/100
  int8_t drpm;         // change in computed rpm (rpm_now - rpm_before)
};

/***********************************************************************/

// RAW sensor values

#define BATT  0  //Battery Voltage ADC Raw Reading - counts
#define EGO   1  //Exhaust Gas Oxygen ADC Raw Reading - counts
#define TPS   2  //Throttle Position Sensor ADC Raw Reading - counts, represents 0 - 5 volts
#define CLT   3  //Coolant Temperature ADC Raw Reading - counts (0 - 255)
#define MAT   4  //Manifold Air Temp ADC Raw Reading - counts (0 - 255)
#define MAP   5  //Manifold Absolute Pressure ADC Raw Reading - KPa (0 - 255)
#define BARO  6  //Barometer ADC Raw Reading - KPa (0 - 255)

/***********************************************************************/

// argument to search_table function
struct search_table_t {
  uint8_t lbound;
  uint8_t ubound;
  uint8_t index;
};

/***********************************************************************/

struct corr_t {
  uint8_t ego;       // Oxygen Sensor Correction
  uint8_t air;       // Air Density Correction lookup
  uint8_t warm;      // Total Warmup Correction
  uint8_t baro;      // Barometer Lookup Correction
  uint8_t gammae;    // Total Gamma Enrichments
  uint8_t ve;        // Current VE value from lookup table
  uint8_t tpsaccel;  // Acceleration enrichment
  uint8_t tpsfuelcut; // fuelcut
};


/***********************************************************************/

#define O2_MAX_TPS 180  // WOT, disable closed loop
#define O2_MAX_KPA 95   // Turbo, disable closed loop

// define the adc sampling period [ms]
#define ADC_PERIOD  5

// PWM_FREQUENCY = 16MHz / 15.984kHz - 1
#define PWM_FREQUENCY   1000

// x seconds after leaving crankmode we can't reenter (unless rpm drops to 0)
#define CRANK_TIMEOUT 10

// tps threshold
#define FLOOD_CLEAR 155

// hysteresis for fastidle
#define FAST_IDLE_THRES 5


/***********************************************************************/
/* Configuration variables */
/***********************************************************************/

// beware when changing the order of the variables
struct config_t {
  uint8_t VE[64];			// таблица VE, 64 байта
  uint8_t cwl;				// обогащение на пуске при -40 С
  uint8_t cwh;				// обогащение на пуске при +77 С
  uint8_t awev; 			// After-start Warmup Percent enrichment add-on value
  uint8_t awc;				// After-start number of cycles
  uint8_t wwu[10];			// Warmup bins(function of temperature)
  uint8_t tpsaq[4]; 		// TPS acceleration amount (function TPSDOT) in 0.1 ms units
  uint8_t tpsacold; 		// Cold acceleration amount (at -40 degrees) in 0.1 ms units
  uint8_t tps_thresh; 		// Accel TPS DOT threshold
  uint8_t tpsasync; 		// ***** TPS Acceleration clock value
  uint8_t tpsdq; 			// Deacceleration fuel cut
  uint8_t egotemp;			// Coolant Temperature where EGO is active
  uint8_t egocountcmp;		// Counter value where EGO step is to occur
  uint8_t egodelta; 		// размер шага EGO корректора для обогащения/обеднения смеси
  uint8_t egolimit; 		// предел, верхний/нижний для работы EGO корректора (коррекция внутри 100% +/- limit)
  uint8_t req_fuel; 		// основная топливная постоянная - требуемое топливо (in 0.1 ms units)
  uint8_t divider;			// IRQ divide factor for pulse
  uint8_t alternate; 		// Alternate injector drivers
  uint8_t injopen;			// время отркрытия форсунки
  uint8_t injocfuel; 		// PW-correlated amount of fuel injected during injector open
							// waste, backwards compatibility
  uint8_t injpwm;			// Injector PWM duty cycle at current limit
  uint8_t injpwmt;			// Injector PWM mmillisec time at which to activate.
							// waste, backwards compatibility
  uint8_t battfac;			// Battery Gamma Factor
  uint8_t rpmk_1;			// Constant for RPM = 12,000/ncyl - downloaded constant
  uint8_t rpmk_2;			// Constant for RPM = 12,000/ncyl - downloaded constant
  uint8_t rpmrangeve[8];  	// VE table RPM Bins for 2-D interpolation
  uint8_t kparangeve[8];  	// VE Table MAP Pressure Bins for 2_D interp.
  uint8_t config11; 		// Configuration for PC Configurator
  uint8_t config12;  		// Configuration for PC Configurator
  uint8_t config13; 		// Configuration for PC Configurator
							// waste, backwards compatibility
  uint8_t primep; 			// Priming pulses (0.1 millisec units)
							// waste, backwards compatibility
  uint8_t rpmoxlimit; 		// минимальные обороты, при которых еще активна обратная связь по О2
  uint8_t fastidle; 		// Fast Idle Temperature
  uint8_t voltoxtarget; 	// O2 sensor flip target value
  uint8_t acmult; 			// Acceleration cold multiplication factor (percent/100)

  uint8_t wwurange[10]; 	// WWURANGE has an offset of +40 F
  uint8_t tpsdotrate[4];

  uint8_t cranking_thres; 	/* порог режима пуска по оборотам двигателя (х/100) */
  uint8_t fuelcut_thres;  	/* порог топливоограничения по оборотам двигателя (х/100) */

  uint8_t primep_cold;   	/* длительность подготовительного импульса при -40 С */
  uint8_t primep_warm;    	/* длительность подготовительного импульса при +77 С */

  uint8_t tps_low;  		/* самое низкое возможное значение ADC TPS */
  uint8_t tps_high;  		/* самое высокое возможное значение ADC TPS */

  uint8_t baro;				// mean barometric pressure
  uint8_t dbaro;			// max difference in barometric pressure

  uint8_t fan_temp;  		/* температура включения вентилятора охлаждения */
  uint8_t fan_hyst;  		/* гистерезис для включения/выключения вентилятора охлаждения */

  uint8_t iac_step_seq; 	/* определяет последовательность 'шагания' ШД РХХ */
  uint8_t iac_conf;     	/* различные конфигурационные опции для ШД РХХ (скорость работы ШД РХХ, power-off) */
  uint8_t iac_warm_idle;  	/* обороты ХХ прогретого двигателя (x/100 rpm) */
  uint8_t iac_cold_idle;  	/* обороты ХХ непрогретого двигателя (х/100 rpm) */
  uint8_t iac_skip;       	/* положение дросселя для начала регулирования ХХ (raw TPS ADC) */
  uint8_t iac_decel;      	/* количество шагов для открытия клапана при замедлении */
  uint8_t iac_step_coarse;	/* количество шагов за одну итерацию для грубой регулировки */
  uint8_t iac_step_fine;	/* количество шагов за одну итерацию для тонкой регулировки */
  uint8_t iac_backoff_cold; /* количество шагов на открытие после инициализации для -40 С */
  uint8_t iac_backoff_warm;	/* количество шагов на открытие после инициализации для +77 С */
  uint8_t iac_max_close;	/* количество шагов для максимального закрытия клапана */
  
};


/***********************************************************************/

//config11
#define MAP_SENSOR      0  // 0:mpx4115ap, 		1:mpx4250ap
#define ENGINE_STROKE   2  // 0:4-stroke, 		1:2-stroke
#define INJ_TYPE        3  // 0:port injection, 1:throttle body
//bit 7-4 defines no of cylinders

/***********************************************************************/

//config12
//bit 1-0 coolant sensor type
//bit 3-2 mat sensor type
//bit 7-4 number of injectors

/***********************************************************************/

//config13
#define ODDFIRE               0 // 0:normal, 1:odd-fire
#define O2_WB_SENSOR          1 // 0:narrowband, 1:diy-wb
#define CONTROL_STRATEGY      2 // 0:speed-density, 1:alpha-N
#define BARO_CORRECTION       3 // 0:off, 1:on
//bit 7-4 unsused

/***********************************************************************/
// step_conf
#define power_off_iac  0    // power off stepper 1 step cycle after each move
#define disable_iac    1    // disable stepper idle regulation
#define debug_iac      2    // enable stepper movement when engine stopped

//bit 7-4 defines speed:
// 0 (fastest, 0ms delay between steps)
// f (slowest, 15ms delay between steps)


/***********************************************************************/
/***********************************************************************/
/***********************************************************************/

/* Hardware definitions */

/* analog */

//Port: F
#define ADC_12V      0 // 12V voltage measurement
#define ADC_O2       1 // O2 sensor
#define ADC_TPS      2 // throttle posistion sensor
#define ADC_CLT      3 // coolant
#define ADC_MAT      4 // manifold air temperature
#define ADC_MAP      5 // manifold absolute pressure

#define ADC_AUX6     6 // free io, not used
#define ADC_AUX7     7 // free io, not used

/* digital */

//Port: A
#define LED_STATUS  3 // active LOW!
#define LED_MISC    4 // active HIGH!
#define LED_ACCEL   5
#define LED_WARMUP  6
#define LED_SQUIRT  7 // next to rs232 connector

//Port: B
#define J1          0 // свободный вывод, не используется
#define SCK         1 // sck, used by isp
#define CLT_FAN     2 // coolant fan
#define FUELPUMP    3 // fuel pump
#define IDLEVALVE   4 // idle solenoid (OC0) / (OC1C)
#define PWM_B       5 // injector port2 (OC1B)
#define PWM_A       6 // injector port1 (OC1A)
#define MISC        7 // pwm output (OC2) - not used

//Port: C
#define STEP_A       0 // idle-stepper winding A
#define STEP_EN      1 // idle-stepper driver enable
#define STEP_C       2 // idle-stepper winding C
#define STEP_D       3 // idle-stepper winding D
#define STEP_B       4 // idle-stepper winding B

//Port: D
#define ex_SCL       0 // свободный вывод, не используется
#define ex_SDA       1 // свободный вывод, не используется
#define ex_RXD1      2 // свободный вывод, не используется
#define ex_TXD1      3 // свободный вывод, не используется
#define ex_IC1       4 // свободный вывод, не используется


//Port: E
#define ex_rx0       0 // rs232 receive
#define ex_tx0       1 // rs232 transmit
#define J3           2 // свободный вывод, не используется
#define ex_TMR3_A    3 // свободный вывод, не используется
#define ex_TMR3_B    4 // свободный вывод, не используется
#define ex_TMR3_C    5 // свободный вывод, не используется
#define TACH         6 // ignition signal (int6)
#define J2           7 // свободный вывод, не используется

//Port: G
#define ex_PG0       0 // свободный вывод, не используется
#define ex_PG1       1 // свободный вывод, не используется
#define FLYBACK_A    3
#define FLYBACK_B    4

#endif

// -------------------------------------------------
// должно быть в <avr/pgmspace.h> но это отсутствует в более новой avrlibc:
#ifndef PRG_RDB
#define PRG_RDB(x) pgm_read_byte(x)
#endif

#ifndef PRG_RDW 
#define PRG_RDW(x) pgm_read_word(x)
#endif
