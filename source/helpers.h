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


#ifndef __HELPERS_H__
#define __HELPERS_H__

#include "global.h"
#include <inttypes.h>

uint8_t linear_interp(uint8_t x1, uint8_t x2, uint8_t y1, uint8_t y2, uint8_t x);
void search_table(uint8_t *tbl, uint8_t tbl_length, uint8_t item, struct search_table_t *r);
uint16_t mult_div100(uint8_t a, uint16_t b);

#endif
