TEMPLATEROOT = ..

# compilation flags for gdb

CFLAGS  = -O1 -g
ASFLAGS = -g 

FreeRTOS = /home/brian/github/mcu/STM32/FreeRTOSv9.0.0/FreeRTOS/Source

CFLAGS += -I$(FreeRTOS)/include -DGCC_ARMCM3

vpath %.c $(FreeRTOS)/
vpath %.c $(FreeRTOS)/portable/MemMang
vpath %.c $(FreeRTOS)/portable/GCC/ARM_CM3

# object files
OBJS=  $(STARTUP) main.o
OBJS+= tasks.o queue.o list.o timers.o heap_1.o port.o
OBJS+= stm32f10x_gpio.o stm32f10x_rcc.o stm32f10x_i2c.o stm32f10x_tim.o stm32f10x_usart.o
OBJS+= i2c.o
OBJS+= my_i2c.o my_ssd1306.o my_font.o

# include common make file
include $(TEMPLATEROOT)/Makefile.common


