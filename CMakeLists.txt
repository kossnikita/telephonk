cmake_minimum_required(VERSION 3.27)
project(project_mine LANGUAGES C ASM)

set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/STM32F411CEUx_FLASH.ld CACHE FILEPATH "Path to linker script")

add_executable(
    firmware
    Src/main.c
    Src/stm32f4xx_it.c
    Src/stm32f4xx_hal_msp.c
    Src/system_stm32f4xx.c
    Src/syscall.c
    startup_stm32f411xe.s
)

add_subdirectory(Drivers)

target_include_directories(firmware PRIVATE 
    Inc
)

target_link_options(firmware PRIVATE
    -T${LINKER_SCRIPT}
)

target_compile_definitions(firmware PRIVATE 
    -DSTM32F411xE=1
    -DUSE_HAL_DRIVER=1
    -DUSE_FULL_LL_DRIVER=1
)

find_program(OPENOCD
    openocd
)

add_custom_target(
    flash 
    DEPENDS firmware 
    COMMAND ${OPENOCD} -d1 -s ${CMAKE_SOURCE_DIR} -f phone.cfg -c "program $<TARGET_FILE:firmware> verify reset exit"
)

