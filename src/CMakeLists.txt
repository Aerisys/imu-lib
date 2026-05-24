# -----------------------------------------------------------------------------
# imu-lib — Standalone ESP-IDF component registration
# -----------------------------------------------------------------------------
# This file is used when building the library as a standalone PlatformIO /
# ESP-IDF project (for manual testing / flashing the example).
#
# When `imu-lib` is consumed as a dependency by another ESP-IDF project, that
# project provides its own component registration: this file is irrelevant in
# that case. Only `src/*.cpp` and `include/*.h` are pulled in by PlatformIO's
# library manager (see `library.json`).
# -----------------------------------------------------------------------------

# Library sources only — do NOT glob anything outside src/ here, to keep this
# list aligned with what gets exported to consuming projects.
file(GLOB lib_sources "${CMAKE_SOURCE_DIR}/src/*.cpp")

# Standalone example entry point (contains app_main). Picked up only when
# building the project on its own; consumers never see this directory.
set(example_dir "${CMAKE_SOURCE_DIR}/examples/basic")
set(example_sources "")
if(EXISTS "${example_dir}/main.cpp")
    list(APPEND example_sources "${example_dir}/main.cpp")
endif()

idf_component_register(
    SRCS ${lib_sources} ${example_sources}
    INCLUDE_DIRS "${CMAKE_SOURCE_DIR}/include"
    # The library uses NVS to persist calibration. Listed under REQUIRES so
    # the new ESP-IDF v5 i2c driver and nvs_flash component are visible to
    # both the library and any consumer of this component.
    REQUIRES driver nvs_flash esp_timer
)
