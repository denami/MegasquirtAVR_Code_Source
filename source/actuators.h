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


#ifndef ACTUATORS_H
#define ACTUATORS_H


void init_idle_stepper(void);
void move_idle_stepper(void);
uint8_t next_step(uint8_t phase);
void coolant_fan(void);


#endif
