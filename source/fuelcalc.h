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


#ifndef FUELCALC_H
#define FUELCALC_H

#include <inttypes.h>

void init_fuelcalc(void);


void calc_parameters(void);

void warmup_enrich(void);
void tps_acc_enrich(void);
void o2(void);
void ve_table_lookup(void);
void calc_total_enrichment(void);

void calc_rpm(void);
void check_fast_idle(void);
void cranking(void);
void primepulse(void);


#endif
