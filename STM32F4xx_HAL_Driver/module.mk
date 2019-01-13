STM32F4XX_HAL_SRCDIR = $(PWD)/STM32F4xx_HAL_Driver/src
C_FILES += $(notdir $(wildcard ${STM32F4XX_HAL_SRCDIR}/*.c))