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


#ifndef __STORAGE_H__
#define __STORAGE_H__

#include <inttypes.h>

#define SEEPROM __attribute__((section(".eeprom")))

void loadConfig(void);
uint8_t storeConfig(void);

#endif
