# Utils library
add_library(utils OBJECT
    utils_init.c
    ulog/SEGGER_RTT.c
    ulog/ulog.c
    ulog/printf.c
    kfifo/kfifo.c
    SystemView/SYSVIEW/SEGGER_SYSVIEW.c
    SystemView/Sample/ThreadX/SEGGER_SYSVIEW_ThreadX.c
    SystemView/Sample/ThreadX/Config/Cortex-M/SEGGER_SYSVIEW_Config_ThreadX.c
)

target_compile_options(utils PRIVATE -O2)

target_include_directories(utils PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/ulog
    ${CMAKE_CURRENT_SOURCE_DIR}/kfifo
    ${CMAKE_CURRENT_SOURCE_DIR}/SystemView/Config
    ${CMAKE_CURRENT_SOURCE_DIR}/SystemView/SEGGER
    ${CMAKE_CURRENT_SOURCE_DIR}/SystemView/SYSVIEW
    ${CMAKE_CURRENT_SOURCE_DIR}/SystemView/Sample/ThreadX
)

target_link_libraries(utils PUBLIC stm32cubemx azrtos::threadx)
