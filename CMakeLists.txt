cmake_minimum_required(VERSION 3.21)

project(capicola LANGUAGES CXX C ASM)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── Vendored SDK ────────────────────────────────────────────────────────────
# alchemy-sdk is a submodule at lib/alchemy-sdk; it vendors libDaisy at
# lib/alchemy-sdk/vendor/libDaisy.  Run once after cloning:
#   git submodule update --init --recursive
set(ALCHEMY_SDK_DIR "${CMAKE_SOURCE_DIR}/lib/alchemy-sdk")
list(APPEND CMAKE_MODULE_PATH "${ALCHEMY_SDK_DIR}/cmake")

# Native configure = host test build: compile the header-only DSP core in
# lib/ with the native toolchain and run the unit tests.  No SDK, no libDaisy,
# no board.  Firmware images still require the ARM presets.
if(NOT CMAKE_CROSSCOMPILING)
    enable_testing()
    add_subdirectory(tests/host)
    return()
endif()

if(NOT EXISTS "${ALCHEMY_SDK_DIR}/vendor/libDaisy/CMakeLists.txt")
    message(FATAL_ERROR
        "lib/alchemy-sdk/vendor/libDaisy is not initialised.  Run:\n"
        "    git submodule update --init --recursive")
endif()

# Deploy images (DFU / web-programmer) are launched by the Daisy bootloader, so
# BOOT_APP makes Reset_Handler skip SystemInit (the bootloader already set the
# clock tree).  The SWD/SRAM debug image runs WITHOUT the bootloader — the probe
# loads it straight into SRAM — so it must self-init: omit BOOT_APP there.
option(ALCHEMY_DEBUG_SRAM
       "Build a SWD-debuggable SRAM image (no bootloader, no BOOT_APP)" OFF)

# libDaisy's ArmGNUToolchain.cmake blanks every CMAKE_<LANG>_FLAGS_<CONFIG>
# cache variable, so CMAKE_BUILD_TYPE alone contributes NO -O flag — without
# this the deploy image ships at -O0, the engine overruns the audio-callback
# budget, and the module boots frozen with garbage audio (the -Og debug image
# was actually the faster build).  -O2 matches libDaisy's own Makefile OPT.
# Must precede every add_subdirectory() so it reaches libDaisy, the BSP, the
# framework, and the firmware alike; the SRAM debug image keeps -Og -g3.
if(NOT ALCHEMY_DEBUG_SRAM)
    add_compile_options(-O2)
    add_compile_definitions(NDEBUG)
endif()

# ── libDaisy ────────────────────────────────────────────────────────────────
# BOOT_APP: the Daisy bootloader has already configured the clock tree, so
# Reset_Handler must skip SystemInit().  PUBLIC so it applies to libDaisy's own
# startup file and to every consumer of the `daisy` target.  (Mirrors the SDK's
# own root CMakeLists.)
add_subdirectory("${ALCHEMY_SDK_DIR}/vendor/libDaisy"
                 "${CMAKE_BINARY_DIR}/libDaisy" EXCLUDE_FROM_ALL)
if(NOT ALCHEMY_DEBUG_SRAM)
    target_compile_definitions(daisy PUBLIC BOOT_APP)
elseif(TARGET CMSIS_DEVICE_H7)
    # No bootloader set up the chip for the SRAM image, but libDaisy's
    # DaisySeed::Init() uses boot_version heuristics (read from unstable backup
    # SRAM) that otherwise SKIP System::ConfigureClocks() (-> PLL3 off, ADC
    # calibration hangs in hw.Init()) and/or sdram_handle.Init() (-> imprecise
    # bus fault / HardFault on the first SDRAM-buffer write). Force the full
    # bring-up. (daisy_seed.cpp is compiled into the `daisy` target.)
    target_compile_definitions(daisy PUBLIC DAISY_FORCE_FULL_INIT)
    # The SRAM image runs WITHOUT the bootloader, so its own Reset_Handler ->
    # SystemInit() runs (BOOT_APP omitted above).  By default libDaisy's
    # SystemInit sets SCB->VTOR = FLASH_BANK1_BASE (the bootloader's table), so
    # SysTick/audio IRQs would vector into bootloader handlers and step-debug
    # breaks.  VECT_TAB_SRAM makes SystemInit point VTOR at D1_AXISRAM_BASE
    # (0x24000000) — our SRAM vector table — so interrupts use OUR handlers.
    # system_stm32h7xx.c (which contains SystemInit) is compiled into libDaisy's
    # CMSIS_DEVICE_H7 static library, NOT the `daisy` target, so the define must
    # land there.
    target_compile_definitions(CMSIS_DEVICE_H7 PRIVATE VECT_TAB_SRAM)
endif()

# daisy_executable() helper (linker script, .bin/.hex, -size / -flash targets).
include(AlchemySdk)

# ── SDK library layers (board BSP + framework primitives) ───────────────────
# Hardware before framework to match the SDK's own subdirectory order.
set(ALCHEMY_BOARD "v2" CACHE STRING "Alchemy Lab board version (v1 | v2)")

add_subdirectory("${ALCHEMY_SDK_DIR}/hardware/alchemy-lab/v2"
                 "${CMAKE_BINARY_DIR}/alchemy-board-v2")
add_subdirectory("${ALCHEMY_SDK_DIR}/framework"
                 "${CMAKE_BINARY_DIR}/alchemy-framework")

# ── Firmware ────────────────────────────────────────────────────────────────
set(ALCHEMY_FW_SOURCES
    src/main.cpp
    src/audio/audio_engine.cpp)

# The interactive manual rides the HostLink descriptor, which the bench image
# trades away for the serial log — so it only joins the deploy builds.
if(NOT CAPICOLA_BENCH_LOG)
    list(APPEND ALCHEMY_FW_SOURCES src/capicola_manual.cpp)
endif()

# Board BSP — transitively pulls in alchemy::primitives (LedPanel, anims) and
# the `daisy` library.  alchemy::surface provides the UI layer: ControlLoop,
# Pager (pot-catch + pages), VirtualKnob, CvMatrix — composed in src/main.cpp.
if(ALCHEMY_DEBUG_SRAM)
    # SWD/SRAM debug image: same SRAM linker script, but no bootloader and no
    # BOOT_APP, built -Og -g3 for stepping.  The probe loads it into SRAM
    # (0x24000000) and runs it; it does NOT auto-boot on power-up (the H750
    # resets into internal flash), which is exactly what we want for a debug
    # session.  Build with `cmake --preset arm-debug`.
    set(_alchemy_sram_lds
        "${ALCHEMY_SDK_DIR}/cmake/linkers/alchemy_stm32h750ib_sram.lds")
    add_executable(capicola ${ALCHEMY_FW_SOURCES})
    set_target_properties(capicola PROPERTIES
        SUFFIX ".elf"
        LINK_DEPENDS "${_alchemy_sram_lds}")
    target_link_libraries(capicola PRIVATE
        daisy alchemy::board_alchemy_lab_v2 alchemy::surface alchemy::host_link)
    target_compile_options(capicola PRIVATE -Og -g3)
    target_link_options(capicola PRIVATE
        LINKER:-T,${_alchemy_sram_lds}
        LINKER:--gc-sections
        LINKER:-Map=capicola.map)

else()
    daisy_executable(
        NAME    capicola
        SOURCES ${ALCHEMY_FW_SOURCES}
        LIBS    alchemy::board_alchemy_lab_v2 alchemy::surface
                alchemy::host_link)
endif()

target_include_directories(capicola PRIVATE
    src
    src/hal
    src/audio
    lib)

# Resolve the unified board facade headers to V2 (we include the V2 header
# directly, but framework headers that use the facade need this).
target_compile_definitions(capicola PRIVATE ALCHEMY_BOARD_V2)

# ── HostLink identity / bench log ───────────────────────────────────────────
# Short git hash (+ "-dirty"), stamped into the HostLink HELLO + descriptor.
# Regenerated on every build by cmake/git_stamp.cmake; the header is only
# rewritten when the stamp changes, so it never forces a rebuild on its own.
set(CAPICOLA_STAMP_DIR "${CMAKE_BINARY_DIR}/generated")
add_custom_target(capicola_git_stamp
    COMMAND ${CMAKE_COMMAND} -DSRC=${CMAKE_SOURCE_DIR}
            -DOUT=${CAPICOLA_STAMP_DIR}/git_stamp.h
            -P ${CMAKE_SOURCE_DIR}/cmake/git_stamp.cmake
    BYPRODUCTS ${CAPICOLA_STAMP_DIR}/git_stamp.h)
add_dependencies(capicola capicola_git_stamp)
target_include_directories(capicola PRIVATE ${CAPICOLA_STAMP_DIR})

# The 1 Hz CPU/CV bench log and hostlink::Host both want the front-panel USB
# CDC — they are mutually exclusive.  Default: HostLink.  Configure with
# -DCAPICOLA_BENCH_LOG=ON for a bench image with the serial log instead.
option(CAPICOLA_BENCH_LOG
       "1 Hz CPU/CV serial log on the front USB (replaces HostLink)" OFF)
if(CAPICOLA_BENCH_LOG)
    target_compile_definitions(capicola PRIVATE CAPICOLA_BENCH_LOG)
endif()

# ── capicola-flash-live — flash without touching the module ─────────────────
# The stock `capicola-flash` needs the module already in DFU (hold B3 through
# the boot window).  This one asks the RUNNING firmware to reboot into the
# system bootloader over the same USB the web programmer uses, then flashes
# it: dfu-util's -w waits for the DFU device to enumerate after the reset.
#
# Requires node for the SDK's hostlink CLI, and a module running HostLink —
# i.e. NOT a CAPICOLA_BENCH_LOG image, which gives that port to the serial
# log.  Fall back to `capicola-flash` in either case.
find_program(_CAPICOLA_NODE NAMES node nodejs)

# ── Manual text — docs/manual.html is the one source ───────────────────────
# Regenerate src/capicola_manual_text.h whenever the manual's data block or the
# generator changes. The header is checked in, so a build without node still
# compiles — it just can't pick up manual edits.
if(_CAPICOLA_NODE AND NOT CAPICOLA_BENCH_LOG)
    add_custom_command(
        OUTPUT  ${CMAKE_SOURCE_DIR}/src/capicola_manual_text.h
        COMMAND "${_CAPICOLA_NODE}" ${CMAKE_SOURCE_DIR}/tools/manual_sync.mjs
        DEPENDS ${CMAKE_SOURCE_DIR}/docs/manual.html
                ${CMAKE_SOURCE_DIR}/tools/manual_sync.mjs
        COMMENT "Manual text: docs/manual.html -> src/capicola_manual_text.h")
    add_custom_target(capicola_manual_text
        DEPENDS ${CMAKE_SOURCE_DIR}/src/capicola_manual_text.h)
    add_dependencies(capicola capicola_manual_text)
endif()
set(_capicola_hostlink
    "${ALCHEMY_SDK_DIR}/tools/hostlink-cli/hostlink.mjs")

if(_ALCHEMY_DFU_UTIL AND _CAPICOLA_NODE AND NOT CAPICOLA_BENCH_LOG)
    add_custom_target(capicola-flash-live
        COMMAND "${_CAPICOLA_NODE}" "${_capicola_hostlink}" reboot bootloader
        # dfu-util's -w returns the instant the device enumerates, which on
        # Linux is before the kernel is done with it -- claiming the interface
        # then fails LIBUSB_ERROR_BUSY.  Let enumeration settle first.
        COMMAND ${CMAKE_COMMAND} -E sleep 2
        COMMAND "${_ALCHEMY_DFU_UTIL}"
                -w
                -a 0
                -s 0x90040000:leave
                -D $<TARGET_FILE_DIR:capicola>/capicola.bin
                -d ,0483:df11
        DEPENDS capicola
        USES_TERMINAL
        COMMENT "Rebooting the running module into DFU, then flashing"
        VERBATIM)
else()
    if(CAPICOLA_BENCH_LOG)
        set(_capicola_live_why
            "the bench image has no HostLink -- use capicola-flash")
    elseif(NOT _CAPICOLA_NODE)
        set(_capicola_live_why
            "node not found (apt install nodejs) -- use capicola-flash")
    else()
        set(_capicola_live_why
            "dfu-util not found (apt install dfu-util)")
    endif()
    add_custom_target(capicola-flash-live
        COMMAND ${CMAKE_COMMAND} -E echo
                "ERROR: ${_capicola_live_why}"
        COMMAND ${CMAKE_COMMAND} -E false
        COMMENT "capicola-flash-live unavailable"
        VERBATIM)
endif()
