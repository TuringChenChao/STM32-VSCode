# S_FILES += startup_stm32f429_439xx.s
STARTUP_SRCDIR = $(PWD)/Startup/gcc
S_FILES += $(notdir $(wildcard $(STARTUP_SRCDIR)/*.s))