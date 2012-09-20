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


#include "storage.h"
#include "global.h"
#include <inttypes.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>

extern volatile struct squirt_t inj_port1, inj_port2;

struct config_t config;
struct config_t config_ee SEEPROM;

const uint8_t code_ver = 20;
extern uint8_t gammae;

void loadConfig(void) {

  /* чтение калибровок из eeprom в sram */
  eeprom_read_block(&config, &config_ee, sizeof(struct config_t));

  /*
    the squirt-datastructure contains its own pwm constants,
    MegaTune supports _one_ shared set of constants. Copy that set
    to both data structures until configuration program is done.
  */
  
  inj_port1.pwm_delay = config.injpwmt;
  inj_port1.pwm_dc = config.injpwm;

  inj_port2.pwm_delay = config.injpwmt;
  inj_port2.pwm_dc = config.injpwm;

  /* 
     these are currently initialized statically,
     haven't had time for doing a Linux program 
     for configuration of these variables
  */

  config.fuelcut_thres = 0x0F;         /* порог топливоограничения по оборотам двигателя (1500 rpm) */
  config.cranking_thres = 3;           /* порог режима пуска по оборотам двигателя (300 rpm) */

  config.primep_warm = config.primep;  /* длительность подготовительного импульса при -40 С */
  config.primep_cold = config.primep;  /* длительность подготовительного импульса при +77 С */

  config.tps_low = 0; //12;            /* самое низкое возможное значение ADC TPS */
  config.tps_high = 255; //220;        /* самое высокое возможное значение ADC TPS */

  config.wwurange[0] = 0;              // should be stored in flash instead
  config.wwurange[1] = 20;
  config.wwurange[2] = 40;
  config.wwurange[3] = 60;
  config.wwurange[4] = 80;
  config.wwurange[5] = 100;
  config.wwurange[6] = 120;
  config.wwurange[7] = 140;
  config.wwurange[8] = 170;
  config.wwurange[9] = 200;

  config.tpsdotrate[0] = 5;            // should be stored in flash instead
  config.tpsdotrate[1] = 20;
  config.tpsdotrate[2] = 40;
  config.tpsdotrate[3] = 77;

  config.fan_temp = 234;               /* температура включения вентилятора охлаждения */
  config.fan_hyst = 5;                 /* гистерезис для включения/выключения вентилятора охлаждения */

  /* эти настройки для шагового двигателя клапана РХХ нельзя изменить из программы настройки */
  config.iac_step_seq = 0xD8;			/* определяет последовательность 'шагания' ШД РХХ */
  config.iac_conf = 0x90;				/* bit 7:4 - скорость работы ШД РХХ [мсек/шаг] (0x90 == 9 мсек/шаг) */
  config.iac_warm_idle = 9;				/* обороты ХХ прогретого двигателя [x100 rpm] */
  config.iac_cold_idle = 15;			/* обороты ХХ непрогретого двигателя [x100 rpm] */
  config.iac_skip = 40;					/* положение дросселя для начала регулирования ХХ [raw TPS ADC] */
  config.iac_decel = 5;					/* количество шагов для прикрытия клапана при замедлении */
  config.iac_step_coarse = 5;			/* количество шагов за одну итерацию для грубой регулировки */
  config.iac_step_fine = 1;				/* количество шагов за одну итерацию для тонкой регулировки */
  config.iac_backoff_cold = 80;			/* количество шагов на открытие после инициализации для -40 С */
  config.iac_backoff_warm = 20;			/* количество шагов на открытие после инициализации для +77 С */
  config.iac_max_close = 100;			/* количество шагов для максимального закрытия клапана */

  config.baro = 100;                   /* mean barometric reading */
  config.dbaro = 20;                   /* max deviation from mean reading */

}

volatile uint8_t eeprom_store_idx;     /* счетчик обрабатываемых байт данных*/
volatile uint8_t eeprom_store_busy;    /* флаг, eeprom занято*/

/*
  **************************************************************************
  *** storeConfig                                                        ***
  **************************************************************************

  Эта функция запускает процесс сохранение структуры конфигурации из sram в 
  eeprom и немедленно возвращает управление,позволяя обработчику прерывания 
  EEPROM продолжить работу по сохранению данных.
*/
uint8_t storeConfig(void) {

  if (eeprom_store_busy) {
    return -1; 				/* eeprom занято, подождите! */
  }

  eeprom_store_idx = 0;		/* обнулить счетчик обрабатываемых байт данных*/
  eeprom_store_busy = 1;	/* установить флаг, eeprom занято*/
  
  EECR |= _BV(EERIE);		/* разрешить прерывание от eeprom */
  return 0;
}


/*
  ************************************************************************
  *** INTERRUPT: EEPROM ready 										   ***
  ************************************************************************

  время выполнения этого прерывания может быть достаточно высоким в случае 
  самой плохой ситуации, это должно быть обязательно исследовано!
*/
SIGNAL(SIG_EEPROM_READY) {
  uint8_t *pe, *ps;
  uint8_t ebyte;

  pe = (uint8_t *)&config_ee + eeprom_store_idx; /* адрес данных в eeprom */
  ps = (uint8_t *)&config + eeprom_store_idx;    /* адрес данных в sram */
  ebyte = eeprom_read_byte(pe);

  /* перезаписывать данные в eeprom будем только если это необходимо */

  /* 
    сравним содержимое структур в sram и eeprom, выход из цикла 
	осуществляем по первому найденному различию в данных
  */	 
  while( (ebyte == *ps) && (eeprom_store_idx != (sizeof(struct config_t) - 1)) ) {
    eeprom_store_idx++;
    pe++;
    ps++;
    ebyte = eeprom_read_byte(pe);
  }

  /* записываем новое значение в eeprom */
  if (ebyte != *ps)
    eeprom_write_byte(pe, *ps);

  /*
    если последняя запись в структуре не достигнута переходим к поиску 
	следующего различия данных в sram и eeprom 
  */
  if (eeprom_store_idx == sizeof(struct config_t)-1) { 	/* запись закончена */
      EECR &= ~_BV(EERIE);     							/* запретить прерывание от eeprom */
      eeprom_store_busy = 0;							/* сбросить флаг, eeprom занято*/
      PORTA &= ~_BV(LED_MISC); 							/* погасить светодиод */
  } else { 												/* переходим к следующему различию */
      eeprom_store_idx++;
      PORTA |= _BV(LED_MISC); 							/* зажечь светодиод */
  /* 
    необходимо отметить, если светодиод горит более 5 сек следует снять питание 
	с контроллера иначе возможно повреждение eeprom
  */
  }
}

