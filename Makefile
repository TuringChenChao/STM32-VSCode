PWD= $(shell pwd)
PROJ_NAME  = HAL_demo
OUTPATH = out
OBJPATH = objects
LOGPATH = logs
LIBPATH := ./lib

CC=arm-none-eabi-gcc
AS=arm-none-eabi-as
LD=arm-none-eabi-ld
AR=arm-none-eabi-ar
OBJCOPY=arm-none-eabi-objcopy
OBJDUMP=arm-none-eabi-objdump

CFLAGS  = -g -O2 -Wall
CFLAGS += -mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16
CFLAGS += -mlittle-endian  -mthumb-interwork
CFLAGS += --specs=nosys.specs

CFLAGS += -DSTM32F429xx -DUSE_HAL_DRIVER

CFLAGS += -I../CMSIS/Include
CFLAGS += -I../STM32F4xx_HAL_Driver/inc -I../STM32F4xx_System/inc -I../STM32F429I-Discovery/inc
CFLAGS += -I../APP/inc

# LDFLAGS = --specs=nano.specs -lnosys -nostartfiles
# LDFLAGS += -Wl,-wrap=malloc -Wl,-wrap=calloc -Wl,-wrap=realloc -Wl,-wrap=free
# LDFLAGS += -Wl,-T./LinkerScripts/STM32F429ZI_FLASH.ld -Wl,--gc-sections
LDFLAGS += -nostartfiles --gc-sections
# LDFLAGS += --gc-sections

LDFLAGS += -L$(LIBPATH)/v7e-m/fpv4-sp/hard -lc
LDFLAGS += -L$(LIBPATH)/v7e-m/fpv4-sp/hard -lg
# LDFLAGS += -L$(LIBPATH)/arm-none-eabi/6.3.1/hard -lgcc
LDFLAGS += -L$(LIBPATH)/arm-none-eabi/6.3.1/thumb/v7e-m/fpv4-sp/hard -lgcc
LDFLAGS += -T./LinkerScripts/STM32F429ZI_FLASH.ld
LDFLAGS += -Map=$(OUTPATH)/$(PROJ_NAME).map

C_FILES :=
S_FILES :=

include ./APP/module.mk
include ./Kernel/module.mk
include ./Startup/module.mk
include ./STM32F4xx_HAL_Driver/module.mk
include ./STM32F4xx_System/module.mk
include ./STM32F429I-Discovery/module.mk

C_OBJS := $(addprefix $(OUTPATH)/$(OBJPATH)/, $(C_FILES:.c=.o))
S_OBJS += $(addprefix $(OUTPATH)/$(OBJPATH)/, $(S_FILES:.s=.o))
LIB_OBJS := libc.a

subdirs :=
subdirs += ./Startup ./STM32F4xx_System ./STM32F4xx_HAL_Driver ./STM32F429I-Discovery ./Kernel ./APP

all:
	@mkdir $(OUTPATH)
	@mkdir $(OUTPATH)/$(OBJPATH)
	@mkdir $(OUTPATH)/$(LOGPATH)
	@echo building $(S_OBJS) $(C_OBJS) $(LIB_OBJS)
	for d in $(subdirs); do make -C $$d || exit 1; done
	# for d in $(subdirs); do make -C $$d || exit 1; done > $(OUTPATH)/$(LOGPATH)/build.log
	@echo building $(OUTPATH)/$(PROJ_NAME).elf
	$(LD) $(C_OBJS) $(S_OBJS) $(LDFLAGS) -o $(OUTPATH)/$(PROJ_NAME).elf
	@$(OBJDUMP) -D $(OUTPATH)/$(PROJ_NAME).elf > $(OUTPATH)/$(PROJ_NAME).dis
	@$(OBJCOPY) -O ihex $(OUTPATH)/$(PROJ_NAME).elf $(OUTPATH)/$(PROJ_NAME).hex
	@$(OBJCOPY) -O binary $(OUTPATH)/$(PROJ_NAME).elf $(OUTPATH)/$(PROJ_NAME).bin
	@echo Done 

clean:
	@echo clean $(S_OBJS) $(C_OBJS) $(LIB_OBJS)
	for d in $(subdirs); do make -C $$d $@; done
	@echo cleaning $(OUTPATH)/$(PROJ_NAME).elf
	@rm -rf $(OUTPATH)