set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_TRY_COMPILE_TARGET_TYPE "STATIC_LIBRARY")
set(CMAKE_SYSTEM_PROCESSOR arm)

find_program(CMAKE_C_COMPILER
    arm-none-eabi-gcc
    REQUIRED
)

find_program(CMAKE_ASM_COMPILER
    arm-none-eabi-gcc
    REQUIRED
)

set(MCU "-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard" CACHE STRING "")

set(CMAKE_C_FLAGS_INIT "\
 ${MCU}\
 -specs=nosys.specs\
 -fstrict-volatile-bitfields\
 -fdata-sections\
 -ffunction-sections\
 -Wall\
 -Wextra\
 -Wpedantic"
)

set(CMAKE_C_FLAGS_DEBUG_INIT "-gdwarf-2")

set(CMAKE_EXE_LINKER_FLAGS_INIT "\
 -Wl,--gc-sections\
 -Wl,--no-warn-rwx-segments\
 -Wl,--print-memory-usage"
)

set(CMAKE_ASM_FLAGS_INIT "${MCU} -Wall -fdata-sections -ffunction-sections")
set(CMAKE_ASM_FLAGS_DEBUG "" CACHE STRING "" FORCE)
set(CMAKE_ASM_FLAGS_MINSIZEREL "" CACHE STRING "" FORCE)
set(CMAKE_ASM_FLAGS_RELEASE "" CACHE STRING "" FORCE)
set(CMAKE_ASM_FLAGS_RELWITHDEBINFO "" CACHE STRING "" FORCE)

set(CMAKE_EXECUTABLE_SUFFIX ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_C ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_ASM ".elf")