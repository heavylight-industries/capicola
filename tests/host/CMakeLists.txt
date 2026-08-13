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
    "${CMAKE_SOURCE_DIR}/lib")

target_compile_options(capicola_tests PRIVATE -Wall -Wextra)

add_test(NAME capicola COMMAND capicola_tests)

# Offline A/B renderer (not a test): blend_render <out.wav> [input.wav]
add_executable(blend_render blend_render.cpp)
target_include_directories(blend_render PRIVATE "${CMAKE_SOURCE_DIR}/lib")
target_compile_options(blend_render PRIVATE -O2 -Wall -Wextra)
