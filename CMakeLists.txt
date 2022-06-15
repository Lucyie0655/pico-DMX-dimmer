cmake_minimum_required(VERSION 3.19.6)

#add the pico sdk before anything else
include(pico_sdk_import.cmake)

project(DMX_Dimmer C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_COMPILE_OBJECT "${CMAKE_CXX_COMPILER} <SOURCE> -c -o <OBJECT> <FLAGS>")

#initiallize the sdk
pico_sdk_init()

add_executable(${PROJECT_NAME})

#generate the pio stuff
pico_generate_pio_header(${PROJECT_NAME} ${CMAKE_CURRENT_LIST_DIR}/shiftOuts.pio)

target_sources(${PROJECT_NAME} PRIVATE
	init.c
	readSensors.c
	pico-ssd1306/ssd1306.c
	display.c
	busCtrl.c
	DMX.c
	outputs.c
	)

#debug levels:
#no def DEBUG 		- no debugging (only print critical errors)
#DEBUG or DEBUG==1 	- basic debuging (use if you don't know whats wrong)
#DEBUG==2 			- more debugging (use if you are changing something)
#DEBUG==3			- just for me, less hardware is needed
#ISRDEBUG			- spams your terminal with a whole bunch of shit, only use if you are forced to
#NOCTRL				- allows control over the USB debug interface NOTE: does not disable the actual DMX interface
target_compile_definitions(${PROJECT_NAME} PUBLIC -DDEBUG=3)
#target_compile_definitions(${PROJECT_NAME} PUBLIC -DNOCTRL)
target_include_directories(${PROJECT_NAME} PRIVATE "./inc")

pico_set_binary_type(${PROJECT_NAME} blocked_ram)
#set_target_properties(${PROJECT_NAME} PROPERTIES PICO_TARGET_LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/memmap_pwrdwn.ld)


#all the libraries
target_link_libraries(${PROJECT_NAME} pico_stdlib)
target_link_libraries(${PROJECT_NAME} pico_multicore)
target_link_libraries(${PROJECT_NAME} hardware_flash)
#target_link_libraries(${PROJECT_NAME} pico_unique_id)
#target_link_libraries(${PROJECT_NAME} pico_bootrom)
#target_link_libraries(${PROJECT_NAME} pico_time)
#target_link_libraries(${PROJECT_NAME} double_tap_usb_boot)
target_link_libraries(${PROJECT_NAME} hardware_spi)
target_link_libraries(${PROJECT_NAME} hardware_dma)
target_link_libraries(${PROJECT_NAME} hardware_uart)
target_link_libraries(${PROJECT_NAME} hardware_timer)
target_link_libraries(${PROJECT_NAME} hardware_irq)
target_link_libraries(${PROJECT_NAME} hardware_gpio)
#target_link_libraries(${PROJECT_NAME} hardware_divider)
target_link_libraries(${PROJECT_NAME} hardware_i2c)
target_link_libraries(${PROJECT_NAME} hardware_adc)
target_link_libraries(${PROJECT_NAME} hardware_pwm)
target_link_libraries(${PROJECT_NAME} hardware_pio)
#target_link_libraries(${PROJECT_NAME} hardware_rtc)
#target_link_libraries(${PROJECT_NAME} hardware_usb)

pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 0)

#create hex/bin file
pico_add_extra_outputs(${PROJECT_NAME})
