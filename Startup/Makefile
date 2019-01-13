CC=arm-none-eabi-gcc
AR=arm-none-eabi-ar
AS=arm-none-eabi-as

CUR_DIR = $(shell pwd)
SRCDIR  = ./gcc
INCDIR  = ./
OBJDIR  = ../out/objects
###########################################
# vpath %.c src  
CFLAGS  = -g -O1 -Wall
CFLAGS += -mthumb -mcpu=cortex-m4
CFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
CFLAGS += -mlittle-endian  -mthumb-interwork
# CFLAGS += -ffreestanding -nostdlib
# CFLAGS += -DSTM32F429_439xx -DUSE_STDPERIPH_DRIVER
# CFLAGS += -I../STM32F4xx_StdPeriph_Driver/inc -I../STM32F4xx/ -I../CMSIS/Include -I../STM32F429I-Discovery
# CFLAGS += -I../Include
# CFLAGS += -I../FreeRTOS/include -I../FreeRTOS/portable/GCC/ARM_CM4F
ASFLAGS = 

SRCS = $(notdir $(wildcard ${SRCDIR}/*.s))
OBJS = $(addprefix $(OBJDIR)/, $(SRCS:.s=.o))

.PHONY: all

all: $(OBJS)
$(OBJDIR)/%.o : $(SRCDIR)/%.s
	$(AS) $^ $(ASFLAGS) -o $@

clean: 
	rm -f $(OBJS)