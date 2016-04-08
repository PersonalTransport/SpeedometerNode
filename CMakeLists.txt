cmake_minimum_required(VERSION 3.2)
project(SpeedometerNode)

include(../cmake-xc16/MPLABX.cmake)

set(SOURCES
    src/main.c
    gen/speedometer.c
)

set(HEADERS
    gen/speedometer.h
)

set(NODE_FILES
    speedometer.ncf
)

add_custom_command(OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/gen/speedometer.h ${CMAKE_CURRENT_SOURCE_DIR}/gen/speedometer.c
                   COMMAND LIN -t PIC${CMAKE_SYSTEM_PROCESSOR} -i uart1 ${NODE_FILES}
                   DEPENDS ${NODE_FILES}
                   WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

add_executable(SpeedometerNode ${SOURCES} ${HEADERS} ${NODE_FILES})
target_compile_definitions(SpeedometerNode PUBLIC -DFCY=2000000)
target_compile_options(SpeedometerNode PUBLIC -std=gnu99)
target_include_directories(SpeedometerNode PUBLIC include gen)
generate_mplabx_project(SpeedometerNode)
