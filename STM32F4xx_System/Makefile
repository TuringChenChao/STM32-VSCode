CC=arm-none-eabi-gcc
AR=arm-none-eabi-ar

CUR_DIR = $(shell pwd)
SRCDIR  = ./src
INCDIR  = ./inc
OBJDIR  = ../out/objects
###########################################
# vpath %.c src  
CFLAGS  = -g -O0 -Wall
CFLAGS += -mthumb -mcpu=cortex-m4
CFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
CFLAGS += -mlittle-endian  -mthumb-interwork

CFLAGS += -I../CMSIS/Include
CFLAGS += -I../STM32F4xx_HAL_Driver/inc -I../STM32F4xx_System/inc -I../STM32F429I-Discovery/inc
CFLAGS += -I$(INCDIR)
CFLAGS += -I../APP/inc

CFLAGS += -DSTM32F429xx -DUSE_HAL_DRIVER

SRCS = $(notdir $(wildcard ${SRCDIR}/*.c))
OBJS = $(addprefix $(OBJDIR)/, $(SRCS:.c=.o))

.PHONY: all

all: $(OBJS)
$(OBJDIR)/%.o : $(SRCDIR)/%.c
	$(CC) $^ $(CFLAGS) -c -o $@

clean: 
	rm -f $(OBJS)