cmake_minimum_required(VERSION 3.21)

include(./simpleble/cmake/prelude.cmake)
project(examples)
include(./simpleble/cmake/epilogue.cmake)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)


set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(SIMPLEBLE_LOCAL "Use local SimpleBLE" ON)

if (SIMPLEBLE_LOCAL)
    add_subdirectory(simpleble/simplecble ${CMAKE_BINARY_DIR}/simplecble)
    # The submodule pins its targets to C++17, but the Windows backend uses
    # C++/WinRT which requires C++20 coroutines. Bump them to match this
    # project's C++20 requirement.
    set_target_properties(simpleble simplecble PROPERTIES CXX_STANDARD 20)
    # The Windows backend uses C++/WinRT, whose runtime entry points
    # (RoInitialize, RoGetActivationFactory, ...) live in runtimeobject.dll.
    if(WIN32)
        target_link_libraries(simpleble PUBLIC runtimeobject)
    endif()
else()
    cmake_policy(SET CMP0144 OLD)
    find_package(simplecble CONFIG REQUIRED)
endif()

# Macro to apply common settings to SimpleCBLE example targets
macro(configure_simplecble_target target_name)
    target_link_libraries(${target_name} simplecble::simplecble)
    install(TARGETS ${target_name} RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
endmacro()

# ---------------------------------------------------------------------------
# dglab: shared library for the DG-LAB Coyote V3 audio-reactive light show.
#
# Extracted from dglab_coyote_audio.cpp so the CLI and the GUI share it:
#   dglab_protocol  - V3 protocol (UUIDs, B0/BF builders, waveform modes),
#                     CoyoteSettings (atomic runtime state mutated by the CLI
#                     keys / GUI controls and read by the streaming tick)
#   dglab_audio     - dominant-tone analysis (Hann + FFT), frequency mapping,
#                     intensity smoothing, and the platform capture backend
#   dglab_session   - CoyoteSession: scan/connect/GATT/streaming lifecycle
# ---------------------------------------------------------------------------
add_library(dglab STATIC
    dglab/dglab_protocol.h
    dglab/dglab_protocol.cpp
    dglab/dglab_audio.h
    dglab/dglab_audio.cpp
    dglab/dglab_audio_backend.h
    dglab/dglab_session.h
    dglab/dglab_session.cpp
    dglab/dglab_protocol_v2.h
    dglab/dglab_protocol_v2.cpp
    dglab/dglab_session_v2.h
    dglab/dglab_session_v2.cpp)

# One audio capture backend per platform (WASAPI on Windows, AUHAL on
# macOS, PortAudio monitor tap on Linux, no-op elsewhere).
if(WIN32)
    target_sources(dglab PRIVATE dglab/dglab_audio_backend_win.cpp)
    target_link_libraries(dglab PRIVATE mmdevapi)
elseif(APPLE)
    target_sources(dglab PRIVATE dglab/dglab_audio_backend_macos.mm)
    target_link_libraries(dglab PUBLIC
        "-framework CoreFoundation"
        "-framework CoreAudio"
        "-framework AudioToolbox")
elseif(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    # Linux: PortAudio tap on the default output's PulseAudio/PipeWire
    # monitor stream. Needs the portaudio development package
    # (e.g. 'libportaudio2-dev' on Debian/Ubuntu, 'portaudio' on macOS).
    find_package(PortAudio REQUIRED)
    target_sources(dglab PRIVATE dglab/dglab_audio_backend_linux.cpp)
    target_link_libraries(dglab PRIVATE PortAudio::PortAudio)
else()
    target_sources(dglab PRIVATE dglab/dglab_audio_backend_none.cpp)
endif()

target_include_directories(dglab PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/dglab)
target_link_libraries(dglab PUBLIC simplecble::simplecble)
target_compile_features(dglab PUBLIC cxx_std_20)

add_library(dglab::dglab ALIAS dglab)

# DG-LAB test applications (Coyote V2/V3 pulse hosts, PawPrint V1.1 button sensor)
set(DGLAB_TEST_APPS
    dglab_coyote
    dglab_coyote_audio
    dglab_coyote2_audio
    dglab_pawprint)

foreach(dglab_app IN LISTS DGLAB_TEST_APPS)
    add_executable(${dglab_app} ${dglab_app}.cpp)
    configure_simplecble_target(${dglab_app})
endforeach()

# dglab_coyote2_audio still carries its own inline audio capture (not the
# dglab backend), so it links the platform audio frameworks directly.
if(WIN32)
    target_link_libraries(dglab_coyote2_audio mmdevapi)
elseif(APPLE)
    target_link_libraries(dglab_coyote2_audio "-framework CoreFoundation"
        "-framework CoreAudio"
        "-framework AudioToolbox")
endif()

# dglab_coyote_audio and dglab_gui share the dglab backend (audio capture +
# BLE session); the audio frameworks and WASAPI are already carried by the
# dglab target itself.
target_link_libraries(dglab_coyote_audio dglab::dglab)

# dglab_gui: microui + fenster GUI for the Coyote audio reactive light show.
# Renders a frame buffer, so it also needs the local microui + fenster +
# renderer stack (renderer.c / microui.c) compiled in and linked, and
# Cocoa on macOS. The live values (audio levels, device status) come from
# the dglab backend; the sliders/radios mutate the shared CoyoteSettings
# the session reads every tick.

# dglab_gui_v2: microui + fenster GUI for the Coyote V2 (ESTIM01) pulse host.
# Mirrors dglab_gui but uses CoyoteV2Settings / CoyoteV2Session and drops the
# smooth / idle-intensity sliders (not present on the V2 backend).
add_executable(dglab_gui
    dglab_gui.cpp
    renderer.c
    microui.c)

target_compile_features(dglab_gui PRIVATE cxx_std_20 c_std_99)

configure_simplecble_target(dglab_gui)
target_link_libraries(dglab_gui dglab::dglab)

add_executable(dglab_gui_v2
    dglab_gui_v2.cpp
    renderer.c
    microui.c)

target_compile_features(dglab_gui_v2 PRIVATE cxx_std_20 c_std_99)

configure_simplecble_target(dglab_gui_v2)
target_link_libraries(dglab_gui_v2 dglab::dglab)

if(APPLE)
    target_link_libraries(dglab_gui "-framework Cocoa")
    target_link_libraries(dglab_gui_v2 "-framework Cocoa")

    # Ship each GUI as a macOS .app bundle so it can be launched from Finder
    # (or a GUI host). Each bundle uses its own Info.plist to carry the
    # correct executable name, bundle identifier and TCC usage descriptions.
    set_target_properties(dglab_gui PROPERTIES
        MACOSX_BUNDLE TRUE
        MACOSX_BUNDLE_INFO_PLIST ${CMAKE_CURRENT_SOURCE_DIR}/Info.plist
        MACOSX_BUNDLE_BUNDLE_NAME "Coyote"
        MACOSX_BUNDLE_GUI_IDENTIFIER "com.dglab.coyote"
        MACOSX_BUNDLE_BUNDLE_VERSION "1.0"
        MACOSX_BUNDLE_SHORT_VERSION_STRING "1.0")
    set_target_properties(dglab_gui_v2 PROPERTIES
        MACOSX_BUNDLE TRUE
        MACOSX_BUNDLE_INFO_PLIST ${CMAKE_CURRENT_SOURCE_DIR}/Info_v2.plist
        MACOSX_BUNDLE_BUNDLE_NAME "Coyote V2"
        MACOSX_BUNDLE_GUI_IDENTIFIER "com.dglab.coyote2"
        MACOSX_BUNDLE_BUNDLE_VERSION "1.0"
        MACOSX_BUNDLE_SHORT_VERSION_STRING "1.0")
endif()

if(UNIX AND NOT APPLE)
    find_package(X11 REQUIRED)
    target_link_libraries(dglab_gui X11::X11)
    target_link_libraries(dglab_gui_v2 X11::X11)
endif()
