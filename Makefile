# Makefile for Reconstructed SIMATIC S7-226 (APM32E103 / STM32F103HD) Firmware
CC      := arm-none-eabi-gcc
OBJDUMP := arm-none-eabi-objdump
SIZE    := arm-none-eabi-size

CFLAGS  := -mcpu=cortex-m3 -mthumb -std=c11 -O2 -g \
           -Iinclude -fno-strict-aliasing -fsigned-char \
           -ffunction-sections -fdata-sections

SRCS := \
    src/startup_apm32e103.c \
    src/main.c \
    src/plc_prog_parser.c \
    src/plc_vm_interpreter.c \
    src/s7_comm_eth_port.c \
    src/s7_ppi_serial.c \
    src/plc_hsc_exti.c \
    src/plc_int_dispatch.c \
    src/plc_sdb_expansion.c \
    src/bsp_peripherals.c \
    src/plc_timers_pto.c \
    src/plc_usart_rs485.c

BUILD_DIR := build
OBJS := $(patsubst src/%.c,$(BUILD_DIR)/%.o,$(SRCS))

.PHONY: all check clean

all: $(OBJS)
	@$(SIZE) $(OBJS)

$(BUILD_DIR)/%.o: src/%.c
	@mkdir -p $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

check:
	@for f in $(SRCS); do \
		$(CC) $(CFLAGS) -fsyntax-only $$f || exit 1; \
	done
	@echo "All $(words $(SRCS)) source files passed syntax & type verification."

clean:
	rm -rf $(BUILD_DIR)
