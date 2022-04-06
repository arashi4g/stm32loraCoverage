################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Utilities/trace/adv_trace/stm32_adv_trace.c 

C_DEPS += \
./Utilities/trace/adv_trace/stm32_adv_trace.d 

OBJS += \
./Utilities/trace/adv_trace/stm32_adv_trace.o 


# Each subdirectory must supply rules for building sources it contributes
Utilities/trace/adv_trace/%.o Utilities/trace/adv_trace/%.su: ../Utilities/trace/adv_trace/%.c Utilities/trace/adv_trace/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DSTM32WL55xx -DCORE_CM4 -DDEBUG -c -I../Core/Inc -I../LoRaWAN/App -I../LoRaWAN/Target -I../Drivers/STM32WLxx_HAL_Driver/Inc/Legacy -I../Utilities/lpm/tiny_lpm -I../Middlewares/Third_Party/LoRaWAN/Mac/Region -I../Drivers/STM32WLxx_HAL_Driver/Inc -I../Middlewares/Third_Party/LoRaWAN/Utilities -I../Utilities/trace/adv_trace -I../Utilities/timer -I../Middlewares/Third_Party/SubGHz_Phy/stm32_radio_driver -I../Middlewares/Third_Party/LoRaWAN/LmHandler/Packages -I../Middlewares/Third_Party/LoRaWAN/LmHandler -I../Middlewares/Third_Party/LoRaWAN/Crypto -I../Drivers/CMSIS/Include -I../Drivers/CMSIS/Device/ST/STM32WLxx/Include -I../Utilities/misc -I../Utilities/sequencer -I../Middlewares/Third_Party/SubGHz_Phy -I../Middlewares/Third_Party/LoRaWAN/Mac -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Utilities-2f-trace-2f-adv_trace

clean-Utilities-2f-trace-2f-adv_trace:
	-$(RM) ./Utilities/trace/adv_trace/stm32_adv_trace.d ./Utilities/trace/adv_trace/stm32_adv_trace.o ./Utilities/trace/adv_trace/stm32_adv_trace.su

.PHONY: clean-Utilities-2f-trace-2f-adv_trace

