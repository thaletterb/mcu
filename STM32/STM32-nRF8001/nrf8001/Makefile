TEMPLATEROOT = ..

# compilation flags for gdb

CFLAGS  = -O1 -g
ASFLAGS = -g 

# object files

OBJS=  $(STARTUP) main.o
OBJS+= stm32f10x_gpio.o stm32f10x_rcc.o spi.o stm32f10x_spi.o
OBJS+=  $(STARTUP) lib_aci.o


# include common make file

include $(TEMPLATEROOT)/Makefile.common


