# Host-side tests for the DSP core and HostLink control edits; no hardware:
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

add_executable(host_controls_tests
    host_controls_tests.cpp
    "${ALCHEMY_SDK_DIR}/framework/src/surface/pager.cpp")
target_include_directories(host_controls_tests PRIVATE
    "${CMAKE_SOURCE_DIR}/src"
    "${CMAKE_SOURCE_DIR}/src/hal"
    "${ALCHEMY_SDK_DIR}/framework/include"
    "${ALCHEMY_SDK_DIR}/hardware/include")
target_compile_options(host_controls_tests PRIVATE -Wall -Wextra -Werror)
add_test(NAME host_controls COMMAND host_controls_tests)

# Offline A/B renderer (not a test): blend_render <out.wav> [input.wav]
add_executable(blend_render blend_render.cpp)
target_include_directories(blend_render PRIVATE "${CMAKE_SOURCE_DIR}/lib")
target_compile_options(blend_render PRIVATE -O2 -Wall -Wextra)

# Offline render of the feedback loop with the diffuser (not a test):
#   diffuse_render <out.wav> <diffusion> <spread> [feedback] [input.wav] [pitch] [stretch]
add_executable(diffuse_render diffuse_render.cpp)
target_include_directories(diffuse_render PRIVATE "${CMAKE_SOURCE_DIR}/lib")
target_compile_options(diffuse_render PRIVATE -O2 -Wall -Wextra)
