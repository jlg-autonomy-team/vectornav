cmake_minimum_required(VERSION 3.16)
project(libvncxx CXX)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    # TODO[DERECK] eliminate warnings in vncxx
    # add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

set(SOURCE
        src/attitude.cpp
        src/compositedata.cpp
        src/conversions.cpp
        src/criticalsection.cpp
        src/dllvalidator.cpp
        src/error_detection.cpp
        src/event.cpp
        src/ezasyncdata.cpp
        src/memoryport.cpp
        src/packet.cpp
        src/packetfinder.cpp
        src/port.cpp
        src/position.cpp
        src/rtcmlistener.cpp
        src/rtcmmessage.cpp
        src/searcher.cpp
        src/sensors.cpp
        src/serialport.cpp
        src/thread.cpp
        src/types.cpp
        src/util.cpp
        src/utilities.cpp
        src/vntime.cpp
        include/vn/exceptions.h
        include/vn/matrix.h
        include/vn/attitude.h
        include/vn/boostpython.h
        include/vn/dllvalidator.h
        include/vn/signal.h
        include/vn/error_detection.h
        include/vn/position.h
        include/vn/registers.h
        include/vn/rtcmlistener.h
        include/vn/rtcmmessage.h
        include/vn/utilities.h
        include/vn/memoryport.h
        include/vn/nocopy.h
        include/vn/compositedata.h
        include/vn/criticalsection.h
        include/vn/compiler.h
        include/vn/sensors.h
        include/vn/searcher.h
        include/vn/event.h
        include/vn/ezasyncdata.h
        include/vn/serialport.h
        include/vn/export.h
        include/vn/vector.h
        include/vn/vntime.h
        include/vn/packetfinder.h
        include/vn/conversions.h
        include/vn/types.h
        include/vn/int.h
        include/vn/thread.h
        include/vn/mock.h
        include/vn/port.h
        include/vn/util.h
        include/vn/consts.h
        include/vn/packet.h)

include_directories(
    include)

add_library(libvncxx ${SOURCE})

target_include_directories(
    ${PROJECT_NAME} PUBLIC include
)

install(TARGETS libvncxx
    LIBRARY DESTINATION lib
)

install(DIRECTORY include/vn DESTINATION include)
