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


#ifndef COMM_H
#define COMM_H

#include <inttypes.h>

#define CMD_QUEUE_SIZE 32

struct cmd_queue_t {
  uint8_t (*func)(uint8_t i);   // адрес функции обработчика посылки
  uint8_t len;					// длинна посылки в байтах
  uint8_t count;				// число переданных байт из посылки
};

void initUART(void);
void comm(uint8_t data);
void storeConfigVar(uint8_t addr, uint8_t value);

uint8_t sendRTvar(uint8_t i);
uint8_t sendConfigVar(uint8_t i);
uint8_t testComm(uint8_t i);
uint8_t codeVer(uint8_t i);
uint8_t pushfunc(uint8_t (*f)(uint8_t i), uint8_t len);

#endif
