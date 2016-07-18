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
TARGET     = $(notdir $(shell pwd))

# Take a look into $(CUBE_DIR)/Drivers/BSP for available BSPs
BOARD      = STM32F4-Discovery
# GDBFLAGS   =

#EXAMPLE   = Templates
# EXAMPLE    = Examples/GPIO/GPIO_IOToggle


MCU_FAMILY = stm32f4xx
BOARD      = STM32F4-Discovery
MCU        = STM32F407xx
# STM32F407VGTx

OCD_SCRIPTS_DIR    = /usr/local/share/openocd/scripts
OCDFLAGS   = -f board/stm32f4discovery.cfg

GCC_DIR    = /opt/gcc-arm-none-eabi-5_4-2016q2

CUBE_DIR   = ../stm32CubeF4
HAL_DIR    = $(CUBE_DIR)/Drivers/STM32F4xx_HAL_Driver
CMSIS_DIR  = $(CUBE_DIR)/Drivers/CMSIS
DEV_DIR    = $(CMSIS_DIR)/Device/ST/STM32F4xx
BSP_DIR    = $(CUBE_DIR)/Drivers/BSP/STM32F4-Discovery
CUBE_URL   = http://www.st.com/st-web-ui/static/active/en/st_prod_software_internet/resource/technical/software/firmware/stm32cubef4.zip




# ###############    Your C files from the /src directory  ###################

# add any new .c file you use, bellow

SRCS       = main.c
SRCS      += system_$(MCU_FAMILY).c
SRCS      += stm32f4xx_it.c
SRCS      += drv_uart.c
SRCS      += LSM9DS0.c
# SRCS      += uart_intr_c.c
# SRCS      += buffer_cir_c.c



# #####################    Basic HAL libraries  ##############################

# uncomment all the .c files you need for your project from the list below
# SRCS      += stm32f4_discovery.c

SRCS      += stm32f4xx_hal.c
# SRCS      += stm32f4xx_hal_adc.c
# SRCS      += stm32f4xx_hal_adc_ex.c
# SRCS      += stm32f4xx_hal_can.c
# SRCS      += stm32f4xx_hal_cec.c
SRCS      += stm32f4xx_hal_cortex.c
# SRCS      += stm32f4xx_hal_crc.c
# SRCS      += stm32f4xx_hal_cryp.c
# SRCS      += stm32f4xx_hal_cryp_ex.c
# SRCS      += stm32f4xx_hal_dac.c
# SRCS      += stm32f4xx_hal_dac_ex.c
# SRCS      += stm32f4xx_hal_dcmi.c
# SRCS      += stm32f4xx_hal_dcmi_ex.c
# SRCS      += stm32f4xx_hal_dfsdm.c
# SRCS      += stm32f4xx_hal_dma2d.c
SRCS      += stm32f4xx_hal_dma.c
# SRCS      += stm32f4xx_hal_dma_ex.c
# SRCS      += stm32f4xx_hal_dsi.c
# SRCS      += stm32f4xx_hal_eth.c
# SRCS      += stm32f4xx_hal_flash.c
# SRCS      += stm32f4xx_hal_flash_ex.c
# SRCS      += stm32f4xx_hal_flash_ramfunc.c
# SRCS      += stm32f4xx_hal_fmpi2c.c
# SRCS      += stm32f4xx_hal_fmpi2c_ex.c
SRCS      += stm32f4xx_hal_gpio.c
# SRCS      += stm32f4xx_hal_hash.c
# SRCS      += stm32f4xx_hal_hash_ex.c
# SRCS      += stm32f4xx_hal_hcd.c
SRCS      += stm32f4xx_hal_i2c.c
SRCS      += stm32f4xx_hal_i2c_ex.c
# SRCS      += stm32f4xx_hal_i2s.c
# SRCS      += stm32f4xx_hal_i2s_ex.c
# SRCS      += stm32f4xx_hal_irda.c
# SRCS      += stm32f4xx_hal_iwdg.c
# SRCS      += stm32f4xx_hal_lptim.c
# SRCS      += stm32f4xx_hal_ltdc.c
# SRCS      += stm32f4xx_hal_ltdc_ex.c
# SRCS      += stm32f4xx_hal_nand.c
# SRCS      += stm32f4xx_hal_nor.c
# SRCS      += stm32f4xx_hal_pccard.c
# SRCS      += stm32f4xx_hal_pcd.c
# SRCS      += stm32f4xx_hal_pcd_ex.c
# SRCS      += stm32f4xx_hal_pwr.c
# SRCS      += stm32f4xx_hal_pwr_ex.c
# SRCS      += stm32f4xx_hal_qspi.c
SRCS      += stm32f4xx_hal_rcc.c
# SRCS      += stm32f4xx_hal_rcc_ex.c
# SRCS      += stm32f4xx_hal_rng.c
# SRCS      += stm32f4xx_hal_rtc.c
# SRCS      += stm32f4xx_hal_rtc_ex.c
# SRCS      += stm32f4xx_hal_sai.c
# SRCS      += stm32f4xx_hal_sai_ex.c
# SRCS      += stm32f4xx_hal_sd.c
# SRCS      += stm32f4xx_hal_sdram.c
# SRCS      += stm32f4xx_hal_smartcard.c
# SRCS      += stm32f4xx_hal_spdifrx.c
# SRCS      += stm32f4xx_hal_spi.c
# SRCS      += stm32f4xx_hal_sram.c
# SRCS      += stm32f4xx_hal_tim.c
# SRCS      += stm32f4xx_hal_timebase_rtc_alarm_template.c
# SRCS      += stm32f4xx_hal_timebase_rtc_wakeup_template.c
# SRCS      += stm32f4xx_hal_timebase_tim_template.c
# SRCS      += stm32f4xx_hal_tim_ex.c
SRCS      += stm32f4xx_hal_uart.c
# SRCS      += stm32f4xx_hal_usart.c
# SRCS      += stm32f4xx_hal_wwdg.c
# SRCS      += stm32f4xx_ll_fmc.c
# SRCS      += stm32f4xx_ll_fsmc.c
# SRCS      += stm32f4xx_ll_sdmmc.c
# SRCS      += stm32f4xx_ll_usb.c



# that's it, no need to change anything below this line!

###############################################################################
# Toolchain

#change it with the output path of whereis arm-none-eabi-gcc  if you have binries in path
PREFIX     = $(GCC_DIR)/bin/arm-none-eabi

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
DEFS       = -D$(MCU) -DUSE_HAL_DRIVER -DHSE_VALUE=8000000
# -DUSE_STDPERIPH_DRIVER



# Debug specific definitions for semihosting
DEFS       += -DUSE_DBPRINTF

# Include search paths (-I)
INCS       = -IInclude
INCS      += -I$(HAL_DIR)/Inc
# INCS      += -I/home/dimitris/programming/stm32/stm32f4/stm32CubeF4/Drivers/STM32F4xx_HAL_Driver/Inc
INCS      += -I$(HAL_DIR)/Legacy/Inc
INCS      += -I$(BSP_DIR)
INCS      += -I$(CMSIS_DIR)/Include
INCS      += -I$(DEV_DIR)/Include

INCS	  += -I$(GCC_DIR)/arm-none-eabi/include

# Library search paths
LIBS       = -L$(CMSIS_DIR)/Lib

# Compiler flags
CFLAGS     = -Wall -g -std=gnu11 -Og
CFLAGS    += -mlittle-endian -mcpu=cortex-m4 -mthumb
CFLAGS    += -ffunction-sections -fdata-sections
CFLAGS    += $(INCS) $(DEFS)

# Linker flags
LDFLAGS    = -Wl,--gc-sections -Wl,-Map=Release/$(TARGET).map $(LIBS) -TSTM32F407VGTx_FLASH.ld

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
	$Q$(CC) $(CFLAGS) $(LDFLAGS) src/startup_stm32f407xx.s $^ -o $@
	@echo "[OBJDUMP] $(TARGET).lst"
	$Q$(OBJDUMP) -St Release/$(TARGET).elf >Release/$(TARGET).lst
	@echo "[SIZE]    $(TARGET).elf"
	$(SIZE) Release/$(TARGET).elf

%.bin: %.elf
	$(OBJCOPY) -O binary $^ $@

%.hex: %.elf
	$(OBJCOPY) -O ihex $^ $@

openocd:
	$(OCD) -s $(OCD_SCRIPTS_DIR) $(OCDFLAGS)

program: all
	$(OCD) -s $(OCD_SCRIPTS_DIR) $(OCDFLAGS) -c "program $(TARGET).elf verify reset"

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
	# cp -ri $(CUBE_DIR)/Projects/$(EXMP_BOARD)/$(EXAMPLE)/Src/* src
	# cp -ri $(CUBE_DIR)/Projects/$(EXMP_BOARD)/$(EXAMPLE)/Inc/* Include
	# cp -i $(DEV_DIR)/Source/Templates/gcc/startup_stm32f103xb.s src
	# cp -i $(HAL_DIR)/Src/stm32f1xx_hal_msp_template.c src/stm32f1xx_hal_msp.c
	# cp -i ../$(MCU)_FLASH.ld $(MCU)_FLASH.ld

clean:
	@echo "[RMDIR]   dep"          ; rm -fr Release
