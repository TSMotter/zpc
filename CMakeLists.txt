cmake_minimum_required(VERSION 3.20.0)

set(BOARD stm32f4_disco)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(zpc LANGUAGES C)

set(SRC_DIR "src")

set(INCLUDE_DIRS
    "${SRC_DIR}/external"
)

# nanopb stuff
set(NANOPB_OPTIONS "-I${CMAKE_SOURCE_DIR}")
nanopb_generate_cpp(SRC_PROTOBUF proto_headers RELPATH . proto/sin_wave.proto)
zephyr_library_include_directories(${CMAKE_CURRENT_BINARY_DIR})

# other 'regular' source files
set(SRC_FILES
    "${SRC_DIR}/main.c"
    "${SRC_DIR}/external/fcs.c"
    "${SRC_DIR}/external/yahdlc.c"
)

target_include_directories(app PRIVATE ${INCLUDE_DIRS})
target_sources(app PRIVATE ${SRC_PROTOBUF} ${SRC_FILES})