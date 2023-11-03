target_sources(firmware PRIVATE
    src/SEGGER_RTT_printf.c
    src/SEGGER_RTT_Syscalls_GCC.c
    src/SEGGER_RTT.c
    src/SEGGER_RTT_ASM_ARMv7M.S
)

target_include_directories(firmware PRIVATE inc)