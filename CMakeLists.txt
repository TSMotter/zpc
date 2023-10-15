cmake_minimum_required(VERSION 3.20.0)

set(BOARD stm32f4_disco)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(zpc LANGUAGES C)

set(SRC_DIR "src")

# list include directories
set(INCLUDE_DIRS
    "${SRC_DIR}/external"
)

# nanopb stuff
list(APPEND CMAKE_MODULE_PATH ${ZEPHYR_BASE}/modules/nanopb)
include(nanopb)

# List 'regular' source files
set(SRC_FILES
    "${SRC_DIR}/main.c"
    "${SRC_DIR}/external/fcs.c"
    "${SRC_DIR}/external/yahdlc.c"
)

target_include_directories(app PRIVATE ${INCLUDE_DIRS})
zephyr_nanopb_sources(app proto/sin_wave.proto)
target_sources(app PRIVATE ${SRC_FILES})