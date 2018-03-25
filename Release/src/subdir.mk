################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/AD5301.c \
../src/AD7991.c \
../src/DG8SAQ_cmd.c \
../src/I2C.c \
../src/LCD_bargraphs.c \
../src/Mobo_config.c \
../src/PCF8574.c \
../src/Si570.c \
../src/TMP100.c \
../src/PCM1792.c \
../src/monitor_task.c \
../src/SI5351A.c \
../src/composite_widget.c \
../src/device_audio_task.c \
../src/device_mouse_hid_task.c \
../src/features.c \
../src/flashyBlinky.c \
../src/freq_and_filters.c \
../src/host_audio_task.c \
../src/hpsdr_device_audio_task.c \
../src/hpsdr_usb_descriptors.c \
../src/hpsdr_usb_specific_request.c \
../src/image.c \
../src/rotary_encoder.c \
../src/taskPCM1792A.c \
../src/taskEXERCISE.c \
../src/taskLCD.c \
../src/taskMoboCtrl.c \
../src/taskPowerDisplay.c \
../src/taskPushButtonMenu.c \
../src/taskStartupLogDisplay.c \
../src/uac1_device_audio_task.c \
../src/uac1_image.c \
../src/uac1_usb_descriptors.c \
../src/uac1_usb_specific_request.c \
../src/uac2_device_audio_task.c \
../src/uac2_image.c \
../src/uac2_usb_descriptors.c \
../src/uac2_usb_specific_request.c \
../src/usb_descriptors.c \
../src/usb_specific_request.c \
../src/widget.c \
../src/wm8805.c 


OBJS += \
./src/AD5301.o \
./src/AD7991.o \
./src/DG8SAQ_cmd.o \
./src/I2C.o \
./src/LCD_bargraphs.o \
./src/Mobo_config.o \
./src/PCF8574.o \
./src/Si570.o \
./src/TMP100.o \
./src/PCM1792.o \
./src/monitor_task.o \
./src/SI5351A.o \
./src/composite_widget.o \
./src/device_audio_task.o \
./src/device_mouse_hid_task.o \
./src/features.o \
./src/flashyBlinky.o \
./src/freq_and_filters.o \
./src/host_audio_task.o \
./src/hpsdr_device_audio_task.o \
./src/hpsdr_usb_descriptors.o \
./src/hpsdr_usb_specific_request.o \
./src/image.o \
./src/rotary_encoder.o \
./src/taskPCM1792A.o \
./src/taskEXERCISE.o \
./src/taskLCD.o \
./src/taskMoboCtrl.o \
./src/taskPowerDisplay.o \
./src/taskPushButtonMenu.o \
./src/taskStartupLogDisplay.o \
./src/uac1_device_audio_task.o \
./src/uac1_image.o \
./src/uac1_usb_descriptors.o \
./src/uac1_usb_specific_request.o \
./src/uac2_device_audio_task.o \
./src/uac2_image.o \
./src/uac2_usb_descriptors.o \
./src/uac2_usb_specific_request.o \
./src/usb_descriptors.o \
./src/usb_specific_request.o \
./src/widget.o \
./src/wm8805.o 



C_DEPS += \
./src/AD5301.d \
./src/AD7991.d \
./src/DG8SAQ_cmd.d \
./src/I2C.d \
./src/LCD_bargraphs.d \
./src/Mobo_config.d \
./src/PCF8574.d \
./src/Si570.d \
./src/TMP100.d \
./src/PCM1792.d \
./src/monitor_task.d \
./src/SI5351A.d \
./src/composite_widget.d \
./src/device_audio_task.d \
./src/device_mouse_hid_task.d \
./src/features.d \
./src/flashyBlinky.d \
./src/freq_and_filters.d \
./src/host_audio_task.d \
./src/hpsdr_device_audio_task.d \
./src/hpsdr_usb_descriptors.d \
./src/hpsdr_usb_specific_request.d \
./src/image.d \
./src/rotary_encoder.d \
./src/taskPCM1792A.d \
./src/taskEXERCISE.d \
./src/taskLCD.d \
./src/taskMoboCtrl.d \
./src/taskPowerDisplay.d \
./src/taskPushButtonMenu.d \
./src/taskStartupLogDisplay.d \
./src/uac1_device_audio_task.d \
./src/uac1_image.d \
./src/uac1_usb_descriptors.d \
./src/uac1_usb_specific_request.d \
./src/uac2_device_audio_task.d \
./src/uac2_image.d \
./src/uac2_usb_descriptors.d \
./src/uac2_usb_specific_request.d \
./src/usb_descriptors.d \
./src/usb_specific_request.d \
./src/widget.d \
./src/wm8805.d



# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo Compile $(CFLAGS) $<
	@avr32-gcc $(CFLAGS) -DBOARD=SDRwdgtLite -DFREERTOS_USED -I../src/SOFTWARE_FRAMEWORK/DRIVERS/SSC/I2S -I../src/SOFTWARE_FRAMEWORK/DRIVERS/PDCA -I../src/SOFTWARE_FRAMEWORK/DRIVERS/TWIM -I../src/SOFTWARE_FRAMEWORK/UTILS/DEBUG -I../src/SOFTWARE_FRAMEWORK/SERVICES/USB/CLASS/AUDIO -I../src/SOFTWARE_FRAMEWORK/SERVICES/USB/CLASS/CDC -I../src/SOFTWARE_FRAMEWORK/SERVICES/FREERTOS/Source/portable/GCC/AVR32_UC3 -I../src/SOFTWARE_FRAMEWORK/SERVICES/FREERTOS/Source/include -I../src/SOFTWARE_FRAMEWORK/SERVICES/USB/CLASS/HID -I../src/SOFTWARE_FRAMEWORK/SERVICES/USB -I../src/CONFIG -I../src/SOFTWARE_FRAMEWORK/DRIVERS/USBB/ENUM/DEVICE -I../src/SOFTWARE_FRAMEWORK/DRIVERS/USBB/ENUM -I../src/SOFTWARE_FRAMEWORK/DRIVERS/USBB -I../src/SOFTWARE_FRAMEWORK/DRIVERS/USART -I../src/SOFTWARE_FRAMEWORK/DRIVERS/TC -I../src/SOFTWARE_FRAMEWORK/DRIVERS/WDT -I../src/SOFTWARE_FRAMEWORK/DRIVERS/CPU/CYCLE_COUNTER -I../src/SOFTWARE_FRAMEWORK/DRIVERS/EIC -I../src/SOFTWARE_FRAMEWORK/DRIVERS/RTC -I../src/SOFTWARE_FRAMEWORK/DRIVERS/PM -I../src/SOFTWARE_FRAMEWORK/DRIVERS/GPIO -I../src/SOFTWARE_FRAMEWORK/DRIVERS/FLASHC -I../src/SOFTWARE_FRAMEWORK/UTILS/LIBS/NEWLIB_ADDONS/INCLUDE -I../src/SOFTWARE_FRAMEWORK/UTILS/PREPROCESSOR -I../src/SOFTWARE_FRAMEWORK/UTILS -I../src/SOFTWARE_FRAMEWORK/DRIVERS/INTC -I../src/SOFTWARE_FRAMEWORK/BOARDS -I../src -O2 -fdata-sections -Wall -c -fmessage-length=0 -mpart=uc3a3256 -ffunction-sections -masm-addr-pseudos -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" "$<"


