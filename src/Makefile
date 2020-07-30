
PATMOS_CONFIG := ../config/drone.xml
VHDL_FILES    := ../config/drone.vhdl

GEN_RAFFO_H := ../util/gen_raffo_params.py

CC = patmos-clang

PROG = drone.elf
SOURCES = main.c ahrs.c raffo.c kalman.c

CFLAGS ?= -target patmos-unknown-unknown-elf -O2 -mpatmos-disable-vliw        \
          -I../libfix32math

LDFLAGS ?= -mpatmos-method-cache-size=0x1000                                  \
           -mpatmos-stack-base=0x100000 -mpatmos-shadow-stack-base=0x0f8000   \
           -L../libfix32math

all: $(PROG)

$(PROG): $(SOURCES) raffo_params.h
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $(SOURCES) -lfix32math

raffo_params.h: $(GEN_RAFFO_H)
	$(GEN_RAFFO_H)

clean:
	rm -f $(PROG) *.o raffo_params.h raffo_params.md
