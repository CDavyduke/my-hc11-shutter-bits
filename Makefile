# ShutterBits Makefile
# Author:			Corey Davyduke
# Created:			2012-04-30
# Modified:			2012-04-30
# Description:		This is the Makefile for my Shutter Bits project.
SRCDIR=c:/gel

GEL_BASEDIR=$(SRCDIR)

# Use the cme11 board configuration files (compilation flags
# and specific includes).
override TARGET_BOARD=m68hc11-corey

include $(GEL_BASEDIR)/config/make.defs

CSRCS=ShutterBits.c

OBJS=$(CSRCS:.c=.o)

PROGS=ShutterBits.elf

all::	$(PROGS) ShutterBits.s19

ShutterBits.elf:	$(OBJS)
	$(CC) $(LDFLAGS) -o $@ $(OBJS) $(GEL_LIBS)

install::	$(PROGS)
	$(INSTALL) $(PROGS) $(GEL_INSTALL_BIN)

