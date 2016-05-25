# A modified version of Steffen Vogel's make file.
# this version is meant to work with STM32F103C8T6 generic blue boards.
# i have not implement the openocd function yet, however, one can flash the board
# with make flash command. You can find the original version at
# https://github.com/stv0g/stm32cube-gcc.
#
#
#	# STM32 Makefile for GNU toolchain and openocd
#	#
#	# This Makefile fetches the Cube firmware package from ST's' website.
#	# This includes: CMSIS, STM32 HAL, BSPs, USB drivers and examples.
#	#
#	# Usage:
#	#	make cube		Download and unzip Cube firmware
#	#	make program		Flash the board with OpenOCD
#	#	make openocd		Start OpenOCD
#	#	make debug		Start GDB and attach to OpenOCD
#	#	make dirs		Create subdirs like obj, dep, ..
#	#	make template		Prepare a simple example project in this dir
#	#
#	# Copyright	2015 Steffen Vogel
#	# License	http://www.gnu.org/licenses/gpl.txt GNU Public License
#	# Author	Steffen Vogel <post@steffenvogel.de>
#	# Link		http://www.steffenvogel.de
#	#
#	# edited for the STM32F4-Discovery
#

# A name common to all output files (elf, map, hex, bin, lst)
TARGET     = demo

# Take a look into $(CUBE_DIR)/Drivers/BSP for available BSPs
BOARD      = STM32F103C8T6_BLUE
EXMP_BOARD = STM32F103RB-Nucleo

#OCDFLAGS   = -f board/stm32f4discovery.cfg
GDBFLAGS   =

#EXAMPLE   = Templates
EXAMPLE    = Examples/GPIO/GPIO_IOToggle
# MCU family and type in various capitalizations o_O
MCU_FAMILY = stm32f1xx
MCU        = STM32F103C8



# Your C files from the /src directory
SRCS       = main.c
SRCS      += system_$(MCU_FAMILY).c
SRCS      += stm32f1xx_it.c
SRCS      += stm32f1xx_hal_msp.c



# Basic HAL libraries
SRCS      += stm32f1xx_hal_rcc.c stm32f1xx_hal_rcc_ex.c stm32f1xx_hal.c stm32f1xx_hal_cortex.c stm32f1xx_hal_gpio.c stm32f1xx_hal_uart.c


# Directories
OCD_DIR    = /usr/local/share/openocd/scripts

CUBE_DIR   = ../STM32CubeF1

HAL_DIR    = $(CUBE_DIR)/Drivers/STM32F1xx_HAL_Driver
CMSIS_DIR  = $(CUBE_DIR)/Drivers/CMSIS
DEV_DIR    = $(CMSIS_DIR)/Device/ST/STM32F1xx
BSP_DIR    = $(CUBE_DIR)/Drivers/BSP/STM32F1xx_Nucleo


CUBE_URL   = http://www.st.com/st-web-ui/static/active/en/st_prod_software_internet/resource/technical/software/firmware/stm32cubef1.zip

# that's it, no need to change anything below this line!

###############################################################################
# Toolchain

#change it with the output path of whereis arm-none-eabi-gcc  if you have binries in path
PREFIX     = /opt/gcc-arm-none-eabi-5_3-2016q1/bin/arm-none-eabi

CC         = $(PREFIX)-gcc
AR         = $(PREFIX)-ar
OBJCOPY    = $(PREFIX)-objcopy
OBJDUMP    = $(PREFIX)-objdump
SIZE       = $(PREFIX)-size
GDB        = $(PREFIX)-gdb

OCD        = openocd

###############################################################################
# Options

# Defines
DEFS       = -DSTM32F103xB -DUSE_HAL_DRIVER -DHSE_VALUE=8000000 -DUSE_STDPERIPH_DRIVER



# Debug specific definitions for semihosting
DEFS       += -DUSE_DBPRINTF

# Include search paths (-I)
INCS       = -IInclude
INCS      += -I$(BSP_DIR)
INCS      += -I$(CMSIS_DIR)/Include
INCS      += -I$(DEV_DIR)/Include
INCS      += -I$(HAL_DIR)/Inc
INCS	  += -I/opt/gcc-arm-none-eabi-5_3-2016q1/arm-none-eabi/include

# Library search paths
LIBS       = -L$(CMSIS_DIR)/Lib

# Compiler flags
CFLAGS     = -Wall -g -std=gnu11 -Os
CFLAGS    += -mlittle-endian -mcpu=cortex-m3 -mthumb
CFLAGS    += -ffunction-sections -fdata-sections
CFLAGS    += $(INCS) $(DEFS)

# Linker flags
LDFLAGS    = -Wl,--gc-sections -Wl,-Map=Release/$(TARGET).map $(LIBS) -TSTM32F103C8_FLASH.ld

# Enable Semihosting
LDFLAGS   += --specs=rdimon.specs -lc -lrdimon

# Source search paths
VPATH      = ./src
VPATH     += $(HAL_DIR)/Src
VPATH     += $(DEV_DIR)/Source/
VPATH     += $(BSP_DIR)

OBJS       = $(addprefix Release/obj/,$(SRCS:.c=.o))
DEPS       = $(addprefix Release/dep/,$(SRCS:.c=.d))

# Prettify output
V = 0
ifeq ($V, 0)
	Q = @
	P = > /dev/null
endif

###################################################

.PHONY: all dirs program debug template clean

all: Release/$(TARGET).elf Release/$(TARGET).bin Release/$(TARGET).hex

-include $(DEPS)

dirs:  $(CUBE_DIR)  Release/obj Release/dep
src Include  Release/obj Release/dep:
	@echo "[MKDIR]   $@"
	$Qmkdir -p $@

Release/obj/%.o : %.c | dirs
	@echo "[CC]      $(notdir $<)"
	$Q$(CC) $(CFLAGS) -c -o $@ $< -MMD -MF Release/dep/$(*F).d

Release/$(TARGET).elf: $(OBJS)
	@echo "[LD]      $(TARGET).elf"
	$Q$(CC) $(CFLAGS) $(LDFLAGS) src/startup_stm32f103xb.s $^ -o $@
	@echo "[OBJDUMP] $(TARGET).lst"
	$Q$(OBJDUMP) -St Release/$(TARGET).elf >Release/$(TARGET).lst
	@echo "[SIZE]    $(TARGET).elf"
	$(SIZE) Release/$(TARGET).elf

%.bin: %.elf
	$(OBJCOPY) -O binary $^ $@

%.hex: %.elf
	$(OBJCOPY) -O ihex $^ $@

openocd:
	$(OCD) -s $(OCD_DIR) $(OCDFLAGS)

program: all
	$(OCD) -s $(OCD_DIR) $(OCDFLAGS) -c "program $(TARGET).elf verify reset"

flash	: all
	st-flash write Release/$(TARGET).bin 0x8000000

debug:
	@if ! nc -z localhost 3333; then \
		echo "\n\t[Error] OpenOCD is not running! Start it with: 'make openocd'\n"; exit 1; \
	else \
		$(GDB)  -ex "target extended localhost:3333" \
			-ex "monitor arm semihosting enable" \
			-ex "monitor reset halt" \
			-ex "load" \
			-ex "monitor reset init" \
			$(GDBFLAGS) $(TARGET).elf; \
	fi

cube:
	rm -fr $(CUBE_DIR)
	wget -O /tmp/cube.zip $(CUBE_URL)
	unzip /tmp/cube.zip
	mv STM32Cube* $(CUBE_DIR)
	chmod -R u+w $(CUBE_DIR)
	rm -f /tmp/cube.zip
template: $(CUBE_DIR) src Include
	cp -ri $(CUBE_DIR)/Projects/$(EXMP_BOARD)/$(EXAMPLE)/Src/* src
	cp -ri $(CUBE_DIR)/Projects/$(EXMP_BOARD)/$(EXAMPLE)/Inc/* Include
	cp -i $(DEV_DIR)/Source/Templates/gcc/startup_stm32f103xb.s src
	cp -i $(HAL_DIR)/Src/stm32f1xx_hal_msp_template.c src/stm32f1xx_hal_msp.c
	cp -i ../$(MCU)_FLASH.ld $(MCU)_FLASH.ld

clean:
	@echo "[RMDIR]   dep"          ; rm -fr Release
