# bsp library
set(BSP_SOURCES
    bsp_init.c
    DWT/bsp_dwt.c
    UART/bsp_uart.c
    SPI/bsp_spi.c
    I2C/bsp_i2c.c
    FLASH/bsp_flash.c
    GPIO/bsp_gpio.c
    PWM/bsp_pwm.c
    LED/bsp_led.c
    BEEP/bsp_beep.c
    CAN/bsp_can.c
    CAN/bsp_can_task.c
)

if(TARGET cherryusb)
    list(APPEND BSP_SOURCES USB/usbd_cdc_acm_user.c)
endif()

add_library(bsp OBJECT ${BSP_SOURCES})

target_compile_options(bsp PRIVATE -O2)

target_include_directories(bsp PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/DWT
    ${CMAKE_CURRENT_SOURCE_DIR}/UART
    ${CMAKE_CURRENT_SOURCE_DIR}/SPI
    ${CMAKE_CURRENT_SOURCE_DIR}/I2C
    ${CMAKE_CURRENT_SOURCE_DIR}/FLASH
    ${CMAKE_CURRENT_SOURCE_DIR}/GPIO
    ${CMAKE_CURRENT_SOURCE_DIR}/PWM
    ${CMAKE_CURRENT_SOURCE_DIR}/LED
    ${CMAKE_CURRENT_SOURCE_DIR}/BEEP
    ${CMAKE_CURRENT_SOURCE_DIR}/CAN
)

if(TARGET cherryusb)
    target_include_directories(bsp PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/USB)
endif()

if(TARGET cherryusb)
    target_link_libraries(bsp PUBLIC stm32cubemx azrtos::threadx utils cherryusb)
else()
    target_link_libraries(bsp PUBLIC stm32cubemx azrtos::threadx utils)
endif()
