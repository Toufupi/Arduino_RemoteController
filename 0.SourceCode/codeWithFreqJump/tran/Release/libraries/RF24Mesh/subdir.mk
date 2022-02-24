################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
/usr/opt/eclipse/eclipse/arduinoPlugin/libraries/RF24Mesh/1.1.4/RF24Mesh.cpp 

LINK_OBJ += \
./libraries/RF24Mesh/RF24Mesh.cpp.o 

CPP_DEPS += \
./libraries/RF24Mesh/RF24Mesh.cpp.d 


# Each subdirectory must supply rules for building sources it contributes
libraries/RF24Mesh/RF24Mesh.cpp.o: /usr/opt/eclipse/eclipse/arduinoPlugin/libraries/RF24Mesh/1.1.4/RF24Mesh.cpp
	@echo 'Building file: $<'
	@echo 'Starting C++ compile'
	"/usr/opt/eclipse/eclipse//arduinoPlugin/packages/arduino/tools/avr-gcc/7.3.0-atmel3.6.1-arduino7/bin/avr-g++" -c -g -Os -Wall -Wextra -std=gnu++11 -fpermissive -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -Wno-error=narrowing -MMD -flto -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=10812 -DARDUINO_AVR_NANO -DARDUINO_ARCH_AVR     -I"/usr/opt/eclipse/eclipse/arduinoPlugin/packages/arduino/hardware/avr/1.8.3/cores/arduino" -I"/usr/opt/eclipse/eclipse/arduinoPlugin/packages/arduino/hardware/avr/1.8.3/variants/eightanaloginputs" -I"/home/salimterryli/Arduino/libraries/RF24/utility" -I"/home/salimterryli/Arduino/libraries/RF24" -I"/home/salimterryli/Arduino/libraries/RS-FEC/src" -I"/usr/opt/eclipse/eclipse/arduinoPlugin/libraries/Servo/1.1.7/src" -I"/usr/opt/eclipse/eclipse/arduinoPlugin/packages/arduino/hardware/avr/1.8.3/libraries/SPI/src" -I"/usr/opt/eclipse/eclipse/arduinoPlugin/libraries/RF24Mesh/1.1.4" -I"/home/salimterryli/Arduino/libraries/RF24Network" -I"/usr/opt/eclipse/eclipse/arduinoPlugin/packages/arduino/hardware/avr/1.8.3/libraries/EEPROM/src" -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -D__IN_ECLIPSE__=1 -x c++ "$<"   -o "$@"
	@echo 'Finished building: $<'
	@echo ' '


