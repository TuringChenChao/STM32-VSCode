# C_FILES += stm32f4xx_hal_msp.c
# C_FILES += stm32f4xx_it.c
# C_FILES += system_stm32f4xx.c

STM32F4XX_SYSTEM_SRCDIR = $(PWD)/STM32F4xx_System/src
C_FILES += $(notdir $(wildcard ${STM32F4XX_SYSTEM_SRCDIR}/*.c))