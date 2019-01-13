CC=arm-none-eabi-gcc
AR=arm-none-eabi-ar

CUR_DIR = $(shell pwd)
SRCDIR  = ./FreeRTOS
INCDIR  = ./FreeRTOS/include
OBJDIR  = ../out/objects
###########################################
# vpath %.c src  
CFLAGS  = -g -O0 -Wall
CFLAGS += -mthumb -mcpu=cortex-m4
CFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
CFLAGS += -mlittle-endian  -mthumb-interwork

CFLAGS += -I../APP/inc
CFLAGS += -I../CMSIS/Include
CFLAGS += -I../STM32F4xx_HAL_Driver/inc -I../STM32F4xx_System/inc -I../STM32F429I-Discovery/inc
CFLAGS += -I./FreeRTOS/portable/GCC/ARM_CM4F
CFLAGS += -I$(INCDIR)



SRCS = $(notdir $(wildcard ${SRCDIR}/*.c))
SRCS += $(notdir $(wildcard ${SRCDIR}/portable/GCC/ARM_CM4F/*.c))
SRCS += $(notdir $(wildcard ${SRCDIR}/portable/MemMang/heap_4.c))
OBJS = $(addprefix $(OBJDIR)/, $(SRCS:.c=.o))

.PHONY: all

all: $(OBJS)
$(OBJDIR)/%.o : $(SRCDIR)/%.c
	$(CC) $^ $(CFLAGS) -c -o $@

$(OBJDIR)/%.o : $(SRCDIR)/portable/GCC/ARM_CM4F/%.c
	$(CC) $^ $(CFLAGS) -c -o $@

$(OBJDIR)/heap_4.o : $(SRCDIR)/portable/MemMang/heap_4.c
	$(CC) $^ $(CFLAGS) -c -o $@    

clean: 
	rm -f $(OBJS)