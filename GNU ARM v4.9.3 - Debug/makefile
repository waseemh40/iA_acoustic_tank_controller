################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include src/subdir.mk
-include emlib/src/subdir.mk
-include drivers/subdir.mk
-include CMSIS/EFM32WG/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: iA_acoustic_tank_controller.axf

# Tool invocations
iA_acoustic_tank_controller.axf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: GNU ARM C Linker'
	arm-none-eabi-gcc -g -gdwarf-2 -mcpu=cortex-m4 -mthumb -T "iA_acoustic_tank_controller.ld" -Xlinker --gc-sections -Xlinker -Map="iA_acoustic_tank_controller.map" -mfpu=fpv4-sp-d16 -mfloat-abi=softfp --specs=nano.specs -o iA_acoustic_tank_controller.axf "./CMSIS/EFM32WG/startup_efm32wg.o" "./CMSIS/EFM32WG/system_efm32wg.o" "./drivers/delay.o" "./drivers/fifo_rs232.o" "./drivers/gpio.o" "./drivers/pid_controller.o" "./drivers/rs232.o" "./drivers/segmentlcd.o" "./emlib/src/em_acmp.o" "./emlib/src/em_adc.o" "./emlib/src/em_aes.o" "./emlib/src/em_assert.o" "./emlib/src/em_burtc.o" "./emlib/src/em_can.o" "./emlib/src/em_cmu.o" "./emlib/src/em_core.o" "./emlib/src/em_cryotimer.o" "./emlib/src/em_crypto.o" "./emlib/src/em_csen.o" "./emlib/src/em_dac.o" "./emlib/src/em_dbg.o" "./emlib/src/em_dma.o" "./emlib/src/em_ebi.o" "./emlib/src/em_emu.o" "./emlib/src/em_gpcrc.o" "./emlib/src/em_gpio.o" "./emlib/src/em_i2c.o" "./emlib/src/em_idac.o" "./emlib/src/em_int.o" "./emlib/src/em_lcd.o" "./emlib/src/em_ldma.o" "./emlib/src/em_lesense.o" "./emlib/src/em_letimer.o" "./emlib/src/em_leuart.o" "./emlib/src/em_mpu.o" "./emlib/src/em_msc.o" "./emlib/src/em_opamp.o" "./emlib/src/em_pcnt.o" "./emlib/src/em_prs.o" "./emlib/src/em_qspi.o" "./emlib/src/em_rmu.o" "./emlib/src/em_rtc.o" "./emlib/src/em_rtcc.o" "./emlib/src/em_system.o" "./emlib/src/em_timer.o" "./emlib/src/em_usart.o" "./emlib/src/em_vcmp.o" "./emlib/src/em_vdac.o" "./emlib/src/em_wdog.o" "./src/main.o" -Wl,--start-group -lgcc -lc -lnosys -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

	@echo 'Building hex file: iA_acoustic_tank_controller.hex'
	arm-none-eabi-objcopy -O ihex "iA_acoustic_tank_controller.axf" "iA_acoustic_tank_controller.hex"
	@echo ' '

	@echo 'Building bin file: iA_acoustic_tank_controller.bin'
	arm-none-eabi-objcopy -O binary "iA_acoustic_tank_controller.axf" "iA_acoustic_tank_controller.bin"
	@echo ' '

	@echo 'Building s37 file: iA_acoustic_tank_controller.s37'
	arm-none-eabi-objcopy -O srec "iA_acoustic_tank_controller.axf" "iA_acoustic_tank_controller.s37"
	@echo ' '

	@echo 'Running size tool'
	arm-none-eabi-size "iA_acoustic_tank_controller.axf"
	@echo ' '

# Other Targets
clean:
	-$(RM) $(EXECUTABLES)$(OBJS)$(C_DEPS) iA_acoustic_tank_controller.axf
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
