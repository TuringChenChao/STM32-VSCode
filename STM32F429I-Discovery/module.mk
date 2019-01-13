# C_FILES += stm32f429i_discovery_eeprom.c
# C_FILES += stm32f429i_discovery_gyroscope.c
# C_FILES += stm32f429i_discovery_io.c
# C_FILES += stm32f429i_discovery_lcd.c
# C_FILES += stm32f429i_discovery_sdram.c
# C_FILES += stm32f429i_discovery_ts.c
# C_FILES += stm32f429i_discovery.c

STM32F4XX_DISCOVERY_SRCDIR = $(PWD)/STM32F429I-Discovery/src
C_FILES += $(notdir $(wildcard ${STM32F4XX_DISCOVERY_SRCDIR}/*.c))