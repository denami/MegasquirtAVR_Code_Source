
#Target processor:
	MCU = atmega128
	FREQ = 16000000UL

#Main target file
	TRG = megasquirt

#C sources
	SRC  = megasquirt.c helpers.c adc.c comm.c fuelcalc.c tables.c fuelcontrol.c storage.c actuators.c

### System sources
#	SRC += sys\time.c

#ASM sources
#	ASRC     = xram.S

#Assembler flags
	ASFLAGS = -Wa, -gstabs

#C Compiler flags
	CPFLAGS	= -g -O2 -Wall -Wstrict-prototypes -Wa,-ahlms=$(<:.c=.lst)
# -DDEBUG

#	CPFLAGS = -g -O0 -Wall -Wstrict-prototypes -Wa,-ahlms=$(<:.c=.lst)  # Debug mode. Don't optimize
#	CPFLAGS = -g -O3 -Wall -Wstrict-prototypes -Wa,-ahlms=$(<:.c=.lst)  # Optimize for speed

#Linker flags
	LDFLAGS = -Wl,-Map=$(TRG).map,--cref
#	LDFLAGS = -Wl,-Map=$(TRG).map,--cref,--defsym=__heap_start=0x801000,--defsym=__heap_end=0x807fff
#   LDFLAGS = -Wl,-Map=$(TRG).map,--cref


include avr_make

###### dependencies, add any dependencies you need here ###################
$(TRG).o	: $(TRG).c megasquirt.h global.h helpers.h adc.h storage.h comm.h actuators.h tables.h
helpers.o	: helpers.c helpers.h global.h
adc.o		: adc.c adc.h global.h
comm.o		: comm.c comm.h global.h storage.h actuators.h
fuelcalc.o	: fuelcalc.c fuelcalc.h global.h tables.h
fuelcontrol.o	: fuelcontrol.c fuelcontrol.h fuelcalc.h global.h adc.h comm.h actuators.h
storage.o	: storage.c storage.h global.h
tables.o	: tables.c tables.h thermfactor.c airdenfactor.c barofac4115.c barofac4250.c kpafactor4115.c kpafactor4250.c
actuators.o	: actuators.c actuators.h global.h
