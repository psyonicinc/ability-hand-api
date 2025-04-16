project(ah_wrapper)

include_directories(include)

# Check for Windows platform
if(WIN32)
    set(SERIAL_LIB "src/winserial.cpp")

# Check for Linux platform
elseif(UNIX)
    set(SERIAL_LIB "src/linux_serial.cpp")
endif()

add_library(${PROJECT_NAME} STATIC
    ${SERIAL_LIB}
    src/api.cpp
    src/ppp.cpp
    src/serial_helper.cpp
    src/hand.cpp
    src/parser.cpp
    )
