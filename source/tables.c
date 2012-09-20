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


#include "tables.h"
#include <inttypes.h>
#include <avr/pgmspace.h>


#include "thermfactor.c"
#include "airdenfactor.c"
#include "barofac4115.c"
#include "barofac4250.c"
#include "kpafactor4115.c"
#include "kpafactor4250.c"

