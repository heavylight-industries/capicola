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

if(NOT CMAKE_CROSSCOMPILING)
    message(FATAL_ERROR
        "capicola targets the Alchemy Lab V2 board (cross-compile "
        "only).  Configure with the ARM preset:\n"
        "    cmake --preset arm && cmake --build --preset arm")
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
        daisy alchemy::board_alchemy_lab_v2 alchemy::surface)
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
# Short git hash, stamped into the HostLink HELLO + descriptor.
execute_process(COMMAND git rev-parse --short HEAD
                WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
                OUTPUT_VARIABLE CAPICOLA_GIT_HASH
                OUTPUT_STRIP_TRAILING_WHITESPACE
                ERROR_QUIET)
if(NOT CAPICOLA_GIT_HASH)
    set(CAPICOLA_GIT_HASH "nogit")
endif()
target_compile_definitions(capicola PRIVATE
    CAPICOLA_GIT_HASH="${CAPICOLA_GIT_HASH}")

# The 1 Hz CPU/CV bench log and hostlink::Host both want the front-panel USB
# CDC — they are mutually exclusive.  Default: HostLink.  Configure with
# -DCAPICOLA_BENCH_LOG=ON for a bench image with the serial log instead.
option(CAPICOLA_BENCH_LOG
       "1 Hz CPU/CV serial log on the front USB (replaces HostLink)" OFF)
if(CAPICOLA_BENCH_LOG)
    target_compile_definitions(capicola PRIVATE CAPICOLA_BENCH_LOG)
endif()
