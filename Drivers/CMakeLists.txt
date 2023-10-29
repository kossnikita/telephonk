target_sources(
    firmware PRIVATE
    STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c
    STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c
    STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c
    STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c
    STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c
    STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c
    STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.c
    STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c
    STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c
    STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c
    STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c
    STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c
    STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c
    STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c
    STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_exti.c
    STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_dma.c
    STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_exti.c
    STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_gpio.c
    STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_rcc.c
    STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_usart.c
    STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_utils.c
    ATC/atc.c
    GSM/call.c
    GSM/fs.c
    GSM/gprs.c
    GSM/gsm.c
    GSM/gsmCallback.c
    GSM/msg.c
    GSM/pdu.c
)

target_compile_definitions(firmware PRIVATE 
    -DUSE_FULL_LL_DRIVER=1
)

add_subdirectory(RTT)

target_include_directories(firmware PRIVATE 
    RTT/inc
    CMSIS/Include
    CMSIS/Device/ST/STM32F4xx/Include
    STM32F4xx_HAL_Driver/Inc
    STM32F4xx_HAL_Driver/Inc/Legacy
    ATC
    GSM
)