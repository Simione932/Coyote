# FindPortAudio.cmake
# Finds the PortAudio library.
#
# Target created:
#   PortAudio::PortAudio

# 1. Attempt to load PortAudio via CMake CONFIG
find_package(PortAudio CONFIG QUIET)

if(TARGET PortAudio::PortAudio)
    set(PortAudio_FOUND TRUE)
    return()
endif()

# 2. Try pkg-config (module names on Linux are 'portaudio-2.0' or 'portaudio')
find_package(PkgConfig QUIET)
if(PKG_CONFIG_FOUND)
    pkg_check_modules(PORTAUDIO_PKG QUIET IMPORTED_TARGET portaudio-2.0 portaudio)
    if(TARGET PkgConfig::PORTAUDIO_PKG)
        if(NOT TARGET PortAudio::PortAudio)
            add_library(PortAudio::PortAudio INTERFACE IMPORTED)
            target_link_libraries(PortAudio::PortAudio INTERFACE PkgConfig::PORTAUDIO_PKG)
        endif()
        set(PortAudio_FOUND TRUE)
        return()
    endif()
endif()

# 3. Fallback: Find header and library manually
find_path(PORTAUDIO_INCLUDE_DIR NAMES portaudio.h)
find_library(PORTAUDIO_LIBRARY NAMES portaudio portaudio-2.0)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(PortAudio
    REQUIRED_VARS PORTAUDIO_LIBRARY PORTAUDIO_INCLUDE_DIR
)

if(PortAudio_FOUND AND NOT TARGET PortAudio::PortAudio)
    add_library(PortAudio::PortAudio UNKNOWN IMPORTED)
    set_target_properties(PortAudio::PortAudio PROPERTIES
        IMPORTED_LOCATION "${PORTAUDIO_LIBRARY}"
        INTERFACE_INCLUDE_DIRECTORIES "${PORTAUDIO_INCLUDE_DIR}"
    )
endif()
