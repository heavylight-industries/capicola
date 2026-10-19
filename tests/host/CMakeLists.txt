# Host-side unit tests for the DSP core (lib/).  Header-only, no hardware:
#
#   cmake --preset host && cmake --build --preset host
#   ctest --preset host
#
# Same tiny CHECK-macro harness as the SDK's tests/host — one binary, one
# ctest entry, zero dependencies.

add_executable(capicola_tests
    capicola_tests.cpp)

target_include_directories(capicola_tests PRIVATE
    "${CMAKE_SOURCE_DIR}/lib"
    "${CMAKE_SOURCE_DIR}/src")   # jack_routing.h — firmware logic, no hardware

target_compile_options(capicola_tests PRIVATE -Wall -Wextra)

add_test(NAME capicola COMMAND capicola_tests)

# Offline A/B renderer (not a test): blend_render <out.wav> [input.wav]
add_executable(blend_render blend_render.cpp)
target_include_directories(blend_render PRIVATE "${CMAKE_SOURCE_DIR}/lib")
target_compile_options(blend_render PRIVATE -O2 -Wall -Wextra)

# Pure routing, trigger edges, preset bytes and DAC switching order.
add_executable(jack_routing_tests jack_routing_tests.cpp)
target_include_directories(jack_routing_tests PRIVATE "${CMAKE_SOURCE_DIR}/src")
target_compile_options(jack_routing_tests PRIVATE -Wall -Wextra)
add_test(NAME jack_routing COMMAND jack_routing_tests)

# Compile the real audio engine; only hardware timing and section placement
# are stubbed. Identical stereo input must survive concurrent slice requests.
find_package(Threads REQUIRED)
add_executable(audio_engine_tests
    audio_engine_tests.cpp
    "${CMAKE_SOURCE_DIR}/src/audio/audio_engine.cpp")
target_include_directories(audio_engine_tests PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/stubs"
    "${CMAKE_SOURCE_DIR}/src/audio"
    "${CMAKE_SOURCE_DIR}/src/hal"
    "${CMAKE_SOURCE_DIR}/lib")
target_compile_options(audio_engine_tests PRIVATE -O1 -Wall -Wextra)
target_link_libraries(audio_engine_tests PRIVATE Threads::Threads)
add_test(NAME audio_engine COMMAND audio_engine_tests)

# Save history must preserve the boot record when any prerequisite fails.
add_executable(save_history_tests save_history_tests.cpp)
target_include_directories(save_history_tests PRIVATE "${CMAKE_SOURCE_DIR}/src")
target_compile_options(save_history_tests PRIVATE -Wall -Wextra)
add_test(NAME save_history COMMAND save_history_tests)
