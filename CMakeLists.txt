cmake_minimum_required(VERSION 3.22)

# ==============================================================================
# Root build. Hand-maintained; the vendor half is included, not copied.
# ==============================================================================
#
# Why this file exists rather than listing everything in one place
#
# CubeMX owns 05_vender/stm32cubemx/ and rewrites its CMakeLists on every
# Generate Code. The previous arrangement kept one hand-written Makefile at the
# root that listed both vendor and framework sources, and a regeneration
# silently replaced the vendor half and took the framework source list with it —
# which is exactly what happened once, and why the split is now structural.
#
# So the vendor CMake stays the single source of truth for what CubeMX
# generated, and this file adds the framework on top. Enabling a peripheral in
# CubeMX updates the build by itself; nothing here needs editing.
#
# Three things make that work, none of them obvious:
#
#   1. cmake/stm32cubemx/CMakeLists.txt writes into ${CMAKE_PROJECT_NAME}
#      directly (target_sources, target_link_libraries, the .map property). So
#      the executable this file creates MUST be named the same as the project,
#      and the project name must match what the vendor tree expects. Renaming
#      either one breaks the subdirectory include with a confusing
#      "no target" error rather than anything pointing here.
#
#   2. The vendor toolchain file resolves the linker script against
#      ${CMAKE_SOURCE_DIR}, which is now this directory rather than the vendor
#      one. That path is repaired below.
#
#   3. add_subdirectory brings in the vendor's include dirs and defines through
#      the `stm32cubemx` INTERFACE library, so the framework half gets the HAL
#      headers by linking that target rather than by repeating -I paths.
# ==============================================================================

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

# RelWithDebInfo (-O2 -g) rather than Debug (-O0 -g3), because -O0 is not a neutral
# choice here: the platform layer is a vtable forwarder per call, and at -O0 each one
# compiles to a 14-instruction function with a stack frame instead of the 7-instruction
# tail call -O2 emits. Measured on this tree 2026/9/4, Debug is 176500 bytes of text
# against 106248 for RelWithDebInfo and 91912 for Release -- so the layering was costing
# 68.6 KB of flash and a per-call overhead that no design change could have recovered.
#
# Not Release: that is -Os -g0 here, and a stripped image breaks the debugging method
# this repository actually relies on (reset halt, then a stack backtrace -- see
# docs/debugging/tim2-timebase.md). Keeping -g costs flash only in the ELF, not on the
# device: the .bin and .hex carry no debug sections.
#
# Both other types remain available: `BUILD_TYPE=Debug ./build.sh` when stepping
# through optimised-away locals is the point, `BUILD_TYPE=Release ./build.sh` for the
# smallest image. All 258 host tests pass at -O0, -O2 and -Os.
#
# Note this only takes effect when CONFIGURING. An existing build directory keeps the
# type it was configured with, and build.sh cannot override it -- see
# docs/ai-memory/build-type-is-cached.md.
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "RelWithDebInfo")
endif()

# Must match what the vendor subdirectory expects; see note 1 above.
set(CMAKE_PROJECT_NAME COD_UniFramework_H7)

# clangd is the only language server used here and it reads nothing else.
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

enable_language(C ASM)

add_executable(${CMAKE_PROJECT_NAME})

# ------------------------------------------------------------------------------
# Vendor half, from CubeMX
# ------------------------------------------------------------------------------
# Pulls in the HAL, CMSIS, the USB device library, the startup file, and every
# Core/Src/*.c CubeMX generated. Also appends those sources to our executable.
set(VENDOR_DIR ${CMAKE_CURRENT_SOURCE_DIR}/05_vender/stm32cubemx)
add_subdirectory(${VENDOR_DIR}/cmake/stm32cubemx)

# The vendor toolchain file resolves the linker script against ${CMAKE_SOURCE_DIR},
# which is this directory — but the script lives in the vendor tree, so the -T flag
# it produces names a file that is not there and has to be redirected.
#
# Rewriting the whole flag rather than appending a second -T is deliberate: GNU ld
# silently takes the last one, so both would link but a reader could not tell which
# script was in force.
string(REPLACE
    "-T \"${CMAKE_SOURCE_DIR}/STM32H723xG_flash.ld\""
    "-T \"${VENDOR_DIR}/STM32H723xG_flash.ld\""
    CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS}")

# Three separate failures, because they have three different causes and a single
# check would report the wrong one.
#
# Empty flags mean configure ran without -DCMAKE_TOOLCHAIN_FILE — the toolchain file
# is what sets them at all. That is easy to do by accident and the resulting error
# would otherwise point here rather than at the command line.
if(CMAKE_EXE_LINKER_FLAGS STREQUAL "")
    message(FATAL_ERROR
        "CMAKE_EXE_LINKER_FLAGS is empty — configure ran without "
        "-DCMAKE_TOOLCHAIN_FILE=05_vender/stm32cubemx/cmake/gcc-arm-none-eabi.cmake. "
        "Delete build/ and configure again with the full command from CLAUDE.md.")
endif()

# The REPLACE silently does nothing when its pattern stops matching, which would
# leave the original -T in place: the build then links against a path that does not
# exist, or worse, against a stale script someone left at the root. Checking that the
# vendor path is present is what makes the rewrite verified rather than assumed.
if(NOT CMAKE_EXE_LINKER_FLAGS MATCHES "${VENDOR_DIR}/STM32H723xG_flash.ld")
    message(FATAL_ERROR
        "linker script path was not redirected into the vendor tree — the vendor "
        "toolchain file changed the -T flag it generates, so update the "
        "string(REPLACE) above to match its new form.")
endif()

if(NOT EXISTS ${VENDOR_DIR}/STM32H723xG_flash.ld)
    message(FATAL_ERROR "linker script missing: ${VENDOR_DIR}/STM32H723xG_flash.ld")
endif()

# ------------------------------------------------------------------------------
# FreeRTOS
# ------------------------------------------------------------------------------
# Split across two layers on purpose:
#
#   05_vender/freertos     upstream V11.3.0, byte-for-byte, never edited.
#                          MANIFEST.sha256 there records the blob hashes.
#   04_impl/rtos/freertos  everything this project wrote: FreeRTOSConfig.h, the
#                          kernel's required callbacks, the task declarations,
#                          and the impl_* backends behind PLAT_Task/Mutex/Sem.
#
# Upgrading the kernel is a directory swap plus a checksum re-run, with nothing
# of ours to rescue from the old tree.
#
# The port is ARM_CM4F even though this is a Cortex-M7. That is upstream's own
# recommendation for any revision past r0p1 (portable/GCC/ARM_CM7/ReadMe.txt);
# this H723 reports r1p2, so the r0p1 port's errata 837070 workaround is not
# needed. Only timers.c/event_groups.c/stream_buffer.c/croutine.c are absent,
# matching the config that turns those features off.
set(KERNEL_DIR    ${CMAKE_CURRENT_SOURCE_DIR}/05_vender/freertos)
set(RTOS_IMPL_DIR ${CMAKE_CURRENT_SOURCE_DIR}/04_impl/rtos/freertos)

# ------------------------------------------------------------------------------
# SEGGER RTT
# ------------------------------------------------------------------------------
# Same split, and upstream V8 made it their own design: SEGGER_RTT_ConfDefaults.h
# holds every default and is marked do-not-edit, while SEGGER_RTT_Conf.h ships
# empty for the user. Upstream's Config/ directory is deliberately NOT installed,
# so ours is the only SEGGER_RTT_Conf.h on the include path — copying theirs in
# would create a second candidate resolved by -I order, a silent way to lose
# every override.
set(SEGGER_DIR      ${CMAKE_CURRENT_SOURCE_DIR}/05_vender/segger_rtt)
set(SEGGER_CONF_DIR ${CMAKE_CURRENT_SOURCE_DIR}/04_impl/bsp/segger_rtt)

# ------------------------------------------------------------------------------
# Framework sources
# ------------------------------------------------------------------------------
# The MCU backend is stm32h7. 04_impl/bsp/stm32f4/ is still present but not
# listed here: it is the F407 implementation this port came from, kept for
# reference. Adding a second MCU means another directory beside it, not edits.
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # --- FreeRTOS kernel
    ${KERNEL_DIR}/tasks.c
    ${KERNEL_DIR}/queue.c
    ${KERNEL_DIR}/list.c
    ${KERNEL_DIR}/portable/GCC/ARM_CM4F/port.c
    ${KERNEL_DIR}/portable/MemMang/heap_4.c

    # --- SEGGER RTT
    ${SEGGER_DIR}/RTT/SEGGER_RTT.c
    ${SEGGER_DIR}/RTT/SEGGER_RTT_printf.c
    ${SEGGER_DIR}/RTT/SEGGER_RTT_ASM_ARMv7M.S

    # --- 06_utils
    06_utils/util_registry/util_registry.c
    06_utils/util_ringbuf/util_ringbuf.c
    06_utils/util_fast_math/util_fast_math.c
    06_utils/util_lpf/util_lpf.c
    06_utils/util_td/util_td.c
    06_utils/util_traj_limit/util_traj_limit.c
    06_utils/util_maf/util_maf.c
    06_utils/util_pid/util_pid.c
    06_utils/util_crc/util_crc.c
    06_utils/util_rls/util_rls.c
    06_utils/util_seq/util_seq.c
    06_utils/util_kf/util_kf.c
    06_utils/util_ahrs/util_ahrs.c
    06_utils/util_msgbus/util_msgbus.c
    06_utils/util_log/util_log.c

    # --- 04_impl: RTOS backend
    ${RTOS_IMPL_DIR}/rtos_hooks.c
    ${RTOS_IMPL_DIR}/rtos_fault.c
    ${RTOS_IMPL_DIR}/task/impl_task.c
    ${RTOS_IMPL_DIR}/mutex/impl_mutex.c
    ${RTOS_IMPL_DIR}/sem/impl_sem.c
    ${RTOS_IMPL_DIR}/memory/impl_memory.c

    # --- 04_impl: MCU backend
    04_impl/bsp/stm32h7/gpio/impl_stm32_gpio.c
    04_impl/bsp/stm32h7/uart/impl_stm32_uart.c
    04_impl/bsp/stm32h7/pwm/impl_stm32_pwm.c
    04_impl/bsp/stm32h7/dwt/impl_stm32_dwt.c
    04_impl/bsp/stm32h7/spi/impl_stm32_spi.c
    04_impl/bsp/stm32h7/flash/impl_stm32_flash.c
    04_impl/bsp/stm32h7/can/impl_stm32_can.c
    04_impl/bsp/stm32h7/adc/impl_stm32_adc.c

    # --- 03_platform
    03_platform/rtos/task/plat_task.c
    03_platform/rtos/mutex/plat_mutex.c
    03_platform/rtos/sem/plat_sem.c
    03_platform/rtos/memory/plat_memory.c
    03_platform/bsp/gpio/plat_gpio.c
    03_platform/bsp/uart/plat_uart.c
    03_platform/bsp/adc/plat_adc.c
    03_platform/bsp/iic/plat_iic.c
    03_platform/bsp/pwm/plat_pwm.c
    03_platform/bsp/dwt/plat_dwt.c
    03_platform/bsp/spi/plat_spi.c
    03_platform/bsp/flash/plat_flash.c
    03_platform/bsp/can/plat_can.c

    # --- 02_device
    02_device/dev_remote/dev_remote.c
    02_device/dev_buzzer/dev_buzzer.c
    02_device/dev_steer_chassis/dev_steer_chassis.c
    02_device/dev_power_limit/dev_power_limit.c
    02_device/dev_dji_motor/dev_dji_motor.c
    02_device/dev_dm_motor/dev_dm_motor.c
    02_device/dev_motor_pid/dev_motor_pid.c
    02_device/dev_bmi088/dev_bmi088.c
    02_device/dev_bmi088/dev_bmi088_store.c
    02_device/dev_ws2812/dev_ws2812.c
    02_device/dev_watchdog/dev_watchdog.c

    # --- 01_application
    # The composition root, named for the chip it composes. Retargeting is a matter
    # of building board_stm32f4.c instead: everything above stays on the Board_*
    # names in board.h and never learns which was chosen. Exactly one of these may
    # be in the build -- they define the same symbols.
    01_application/board/board_stm32h7.c
    01_application/tasks/app_tasks.c
    01_application/indicator/app_indicator.c
    01_application/imu/app_imu.c
    01_application/telemetry/app_telemetry.c
    01_application/health/app_health.c
    01_application/chassis/app_chassis.c
)

# Not built:
#
#   04_impl/bsp/stm32h7/iic/impl_stm32_iic.c
#       There is no I2C peripheral to drive. HAL_I2C_MODULE_ENABLED is defined, so
#       the driver compiles, but the .ioc configures no I2C instance and CubeMX
#       therefore generates no Core/Src/i2c.c and no hi2cN handle — so nothing can
#       be passed to IMPL_STM32_IIC_CreateCtx. (An earlier note here said I2C was
#       enabled in CubeMX; that was wrong. The .ioc mentions I2C only under RCC,
#       which lists a clock frequency for every peripheral whether used or not.)
#
#       Enabling it means configuring an I2C instance and its pins in CubeMX, then
#       adding this file and a BOARD_DEVICE entry. plat_iic.c is already built and
#       works with any backend.

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # --- FreeRTOS: config first, then kernel. Both are needed and the order is
    #     not significant here (no duplicate header names), unlike SEGGER below.
    ${RTOS_IMPL_DIR}
    ${KERNEL_DIR}/include
    ${KERNEL_DIR}/portable/GCC/ARM_CM4F

    # --- SEGGER: ours before upstream's, so ConfDefaults.h finds our Conf.h.
    ${SEGGER_CONF_DIR}
    ${SEGGER_DIR}/RTT

    06_utils/util_assert
    06_utils/util_registry
    06_utils/util_ringbuf
    06_utils/util_fast_math
    06_utils/util_lpf
    06_utils/util_td
    06_utils/util_traj_limit
    06_utils/util_maf
    06_utils/util_pid
    06_utils/util_crc
    06_utils/util_rls
    06_utils/util_seq
    06_utils/util_kf
    06_utils/util_ahrs
    06_utils/util_msgbus
    06_utils/util_log

    04_impl/common
    04_impl/bsp/stm32h7
    04_impl/bsp/stm32h7/adc
    04_impl/bsp/stm32h7/can
    04_impl/bsp/stm32h7/dwt
    04_impl/bsp/stm32h7/gpio
    04_impl/bsp/stm32h7/iic
    04_impl/bsp/stm32h7/pwm
    04_impl/bsp/stm32h7/spi
    04_impl/bsp/stm32h7/flash
    04_impl/bsp/stm32h7/uart

    03_platform/rtos/task
    03_platform/rtos/mutex
    03_platform/rtos/sem
    03_platform/rtos/memory
    03_platform/bsp/adc
    03_platform/bsp/gpio
    03_platform/bsp/iic
    03_platform/bsp/uart
    03_platform/bsp/pwm
    03_platform/bsp/dwt
    03_platform/bsp/spi
    03_platform/bsp/dma_buf
    03_platform/bsp/flash
    03_platform/bsp/can

    02_device
    02_device/dev_remote
    02_device/dev_watchdog
    02_device/dev_buzzer
    02_device/dev_steer_chassis
    02_device/dev_power_limit
    02_device/dev_dji_motor
    02_device/dev_dm_motor
    02_device/dev_motor_pid
    02_device/dev_bmi088
    02_device/dev_ws2812

    01_application/board
    01_application/tasks
    01_application/indicator
    01_application/health
    01_application/chassis
    01_application/imu
    01_application/telemetry
)

# CMSIS-DSP is deliberately absent. The F4 build defined ARM_MATH_CM4 and the
# FPU macros for it, but arm_math.h has never been included anywhere in this
# repository — so those defines configured a library that was not in use. If DSP
# is ever wanted, add ARM_MATH_CM7 plus the DSP sources, and check the FPU
# macros against what the H7 CMSIS already provides.
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
)

# Removes a bogus libob.a dependency the toolchain file leaves behind for C++.
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
)

# ------------------------------------------------------------------------------
# Artifacts
# ------------------------------------------------------------------------------
# CubeMX's CMake produces only the ELF. The Makefile this replaced also emitted
# .hex and .bin, and the flashing step wants them, so they are restored here.
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex
            $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary -S
            $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
    COMMENT "Generating .hex and .bin, and reporting size"
)

# ------------------------------------------------------------------------------
# Flashing
# ------------------------------------------------------------------------------
# `cmake --build build --target flash`. Same openocd_dap.cfg the VS Code debug
# configurations use, so the probe and target settings cannot drift between the
# two paths.
#
# Depends on the ELF rather than on `all`, so it cannot flash a stale image: if a
# source changed, the build runs first.
# Two probes are supported side by side, each named explicitly:
#
#   flash-dap / rtt-dap       CMSIS-DAP via OpenOCD (the WCH probe on this bench)
#   flash-jlink / rtt-jlink   SEGGER J-Link
#
# There is deliberately NO `flash` or `rtt` alias. An alias would have to pick a
# probe at configure time and remember it in the cache, so `PROBE=jlink cmake
# --build build --target flash` would silently flash over the OTHER probe --
# cmake does not read the environment, and the cached choice wins. That is the
# same shape as the BUILD_TYPE trap this build already documents, and here it
# would put the wrong image on a board. `./build.sh flash` reads PROBE per run
# and dispatches to the suffixed target, which cannot go stale.
#
# Both toolchains are looked for unconditionally -- whichever is missing produces
# a target that says so, because "No rule to make target 'flash-jlink'" does not
# tell you which tool to install.
find_program(OPENOCD_EXECUTABLE openocd)
find_program(JLINK_EXECUTABLE JLinkExe)

# A target that explains itself beats a missing target. Used for whichever
# probe's tooling is absent.
function(cod_missing_tool target tool package)
    add_custom_target(${target}
        COMMAND ${CMAKE_COMMAND} -E echo
                "${tool} not found on PATH -- install it (${package}) and re-run cmake"
        COMMAND ${CMAKE_COMMAND} -E false
        COMMENT "Cannot run ${target}: ${tool} missing"
    )
endfunction()

if(OPENOCD_EXECUTABLE)
    add_custom_target(flash-dap
        COMMAND ${OPENOCD_EXECUTABLE}
                -f ${CMAKE_CURRENT_SOURCE_DIR}/openocd_dap.cfg
                -c "program $<TARGET_FILE:${CMAKE_PROJECT_NAME}> verify reset exit"
        DEPENDS ${CMAKE_PROJECT_NAME}
        USES_TERMINAL
        COMMENT "Flashing via CMSIS-DAP"
    )
else()
    cod_missing_tool(flash-dap openocd "apt install openocd")
endif()

# -CommanderScript, because JLinkExe has no -c equivalent; the settings live in
# flash.jlink beside openocd_dap.cfg. NoGui keeps a failure on the terminal
# rather than in a dialog nobody sees from a build target.
# The .jlink scripts are templates: J-Link resolves loadfile against the working
# directory, which for a build target is build/, and its own %CD% is passed
# through literally on Linux (V9.72) -- so the path has to be absolute before
# JLinkExe ever sees it. CMake substitutes it, and the generated copies land in
# build/ next to the ELF they name.
if(JLINK_EXECUTABLE)
    set(LOADFILE $<TARGET_FILE:${CMAKE_PROJECT_NAME}>)
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/rtt.jlink.in
                   ${CMAKE_BINARY_DIR}/rtt.jlink @ONLY)
    # file(GENERATE), not configure_file, because $<TARGET_FILE:...> is a
    # generator expression: configure_file runs at configure time and would write
    # the unexpanded string.
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/flash.jlink.in
                   ${CMAKE_BINARY_DIR}/flash.jlink.tmp @ONLY)
    file(GENERATE OUTPUT ${CMAKE_BINARY_DIR}/flash.jlink
                  INPUT ${CMAKE_BINARY_DIR}/flash.jlink.tmp)

    add_custom_target(flash-jlink
        COMMAND ${JLINK_EXECUTABLE}
                -NoGui 1
                -ExitOnError 1
                -CommanderScript ${CMAKE_BINARY_DIR}/flash.jlink
        DEPENDS ${CMAKE_PROJECT_NAME}
        USES_TERMINAL
        COMMENT "Flashing via J-Link"
    )
else()
    cod_missing_tool(flash-jlink JLinkExe "SEGGER J-Link Software pack")
endif()


# ------------------------------------------------------------------------------
# RTT console
# ------------------------------------------------------------------------------
# `cmake --build build --target rtt`, then `nc localhost 9090` in another
# terminal. This is the only log path in the firmware — SEGGER_RTT_printf writes
# over SWD, not to a UART — so it is how anything reports at bring-up.
#
# The search range is 44 KB from 0x20000000, which is DTCMRAM. That is where THIS
# linker script puts .bss, and therefore _SEGGER_RTT; an H7 project more usually
# has it in AXI SRAM at 0x24000000, so a config copied from elsewhere finds
# nothing — and finding nothing looks exactly like firmware that never printed.
#
# The size is deliberately NOT the full 128 KB of DTCM: a search that runs to
# 0x20020000 reads one word past the end of the region and OpenOCD reports
# "Failed to read memory", abandoning the scan. Verified against the ELF:
# _SEGGER_RTT sits at 0x20009184, so 0xB000 covers it with room to spare.
#
# @par The OpenOCD buffer is consumed once
# Measured on hardware: OpenOCD drains the RTT buffer only while a client is
# attached, and destructively. The control block's RdOff sits at 0 with the boot
# log (214 bytes) buffered, jumps to WrOff on the first connection, and a second
# `nc` gets nothing -- this firmware logs at startup and is silent in steady
# state, so the first client to attach takes everything. Anything that merely
# probes the port counts as that client, `nc -z` included. To see the log again,
# re-run the target: it resets the board.
#
# J-Link has no equivalent step. RTT is SEGGER's own protocol, so JLinkRTTClient
# finds the control block itself -- no address range to get wrong, and reconnects
# keep working.
if(OPENOCD_EXECUTABLE)
    add_custom_target(rtt-dap
        COMMAND ${OPENOCD_EXECUTABLE}
                -f ${CMAKE_CURRENT_SOURCE_DIR}/openocd_dap.cfg
                -c "init"
                # Reset and let the firmware run BEFORE searching for the control
                # block. _SEGGER_RTT lives in .bss and is filled in lazily, on the
                # first SEGGER_RTT call, so a search issued at reset time finds
                # nothing -- and "No control block found" is only an Info line, so
                # the target still starts a server that will never deliver a byte.
                # That is indistinguishable from firmware that never printed.
                #
                # Measured: search immediately after reset -> not found; after
                # ~1.5 s of run time -> found. 3000 ms is deliberate headroom over
                # the ~1.4 s this firmware needs to reach its first log, which is
                # dominated by the 2000-sample gyro calibration.
                -c "reset run"
                -c "sleep 3000"
                -c "rtt setup 0x20000000 0xB000 \"SEGGER RTT\""
                -c "rtt start"
                -c "rtt server start 9090 0"
        USES_TERMINAL
        COMMENT "OpenOCD + RTT on tcp/9090 — connect with: nc localhost 9090"
    )
else()
    cod_missing_tool(rtt-dap openocd "apt install openocd")
endif()

# JLinkExe holds the target; JLinkRTTClient connects to it over localhost:19021,
# which the DLL serves automatically. Same two-terminal shape as the DAP path,
# but without a search range to configure.
if(JLINK_EXECUTABLE)
    add_custom_target(rtt-jlink
        COMMAND ${JLINK_EXECUTABLE}
                -NoGui 1
                -CommanderScript ${CMAKE_BINARY_DIR}/rtt.jlink
        USES_TERMINAL
        COMMENT "J-Link + RTT — connect with: JLinkRTTClient"
    )
else()
    cod_missing_tool(rtt-jlink JLinkExe "SEGGER J-Link Software pack")
endif()

