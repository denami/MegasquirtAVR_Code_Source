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
  ¬ основном все функции в этом файле должны быть оптимизированы - 
  то есть, вручную закодированы в avr-ассемблере.
  
  ќсобенно должны быть оптимизированны подпрограммы делени€, так как у avr
  нет машинных командах делени€. ѕосмотрите application note фирмы Atmel на 
  предмет быстрых процедур деление на ассемблере.
*/

#include "global.h"
#include "helpers.h"
#include <inttypes.h>
#include <stdlib.h>


/***************************************************************************/
/*** linear interpolation                                                ***/
/*                                                                         */
/*   x1 - нижн€€ граница исходного диапазона                               */
/*   x2 - верхн€€ граница исходного диапазона                              */
/*   y1 - нижн€€ граница искомого диапазона                                */
/*   y2 - верхн€€ граница искомого диапазона                               */
/*   x - исходное значение                                                 */
/*   функци€ возвращает искомое значение                                   */
/*                                                                         */
/***************************************************************************/
uint8_t linear_interp(uint8_t x1, uint8_t x2, uint8_t y1, uint8_t y2, uint8_t x) {
  uint8_t x21, dx;
  div_t d;

  if (x <= x1)  					// меньше/равно нижней границе
    return y1;

  if (x >= x2)  					// больше/равно верхней границе
    return y2;

  x21 = x2 - x1;
  if (x21 == 0) 					// на ноль делить нельз€!
    return y1;

  dx = x - x1;

  if (y2 < y1) {  					// отрицательный наклон хар-ки
    d = div( (y1-y2)*dx, x21 );
    if (d.rem >= (x21/2))  			// округление результата
      d.quot++;
    return y1 - d.quot;
  } else {        					// положительный наклон хар-ки
    d = div( (y2-y1)*dx, x21 );
    if (d.rem >= (x21/2))  			// округление результата
      d.quot++;
    return y1 + d.quot;
  }
}


/***************************************************************************/
/*** search table                                                        ***/
/***************************************************************************/
void search_table(uint8_t *tbl, uint8_t tbl_length, uint8_t item, struct search_table_t *r) {
  uint8_t i, searching;

  searching = true;
  r->index = 0;

  for (i=0; (i<tbl_length) && searching; i++) {
    r->index = i;
    if (item < tbl[i]) {
      searching = false;
    }
  }

  if (r->index == 0) { // lbound and ubound can't point to the same element
    r->lbound = tbl[0];
    r->ubound = tbl[1];
    r->index = 1;
  } else {
    r->lbound = tbl[r->index-1];
    r->ubound = tbl[r->index];
  }
}


/***************************************************************************/
/*** mult div100                                                         ***/
/***************************************************************************/
uint16_t mult_div100(uint8_t a, uint16_t b) {
  div_t d;

  // this takes ~260 cycles
  d = div(a*b, 100);

  if (d.rem >= 50)
    d.quot++;

  return d.quot;
}

