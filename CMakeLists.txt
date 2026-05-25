cmake_minimum_required(VERSION 3.13)
project(acars_decoder)

# Explicit source list (do NOT glob: keeps test harnesses out of the build).
set(SRC
    "src/main.cpp"
    "src/acars.cpp"
)

include(${SDRPP_MODULE_CMAKE})

target_include_directories(acars_decoder PRIVATE "src/")
