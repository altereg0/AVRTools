#####################################################################
# simple AVR library
#####################################################################
add_compile_definitions(SERIAL_OUTPUT_EOL=\"\\r\\n\")
set(AVRTOOLS_SRC_FILES
        abi.cpp
        new.cpp
        InitSystem.cpp
        SystemClock.cpp
        MemUtils.cpp
        )

set(AVRTOOLS_A2D_SRC_FILES
        Analog2Digital.cpp
        )
set(AVRTOOLS_PWM_SRC_FILES
        Pwm.cpp
        )
set(AVRTOOLS_I2C_SRC_FILES
        I2cLcd.cpp
        I2cMaster.cpp
        I2cSlave.cpp
        )
set(AVRTOOLS_SPI_SRC_FILES
        SPI.cpp
        )
set(AVRTOOLS_ONEWIRE_SRC_FILES
        OneWire.cpp
        )
set(AVRTOOLS_UART0_SRC_FILES
        USART0.cpp
        RingBuffer.cpp
        Writer.cpp
        Reader.cpp
        )
set(AVRTOOLS_UART0_MINIMAL_SRC_FILES
        USART0Minimal.cpp
        )

add_avr_library(avrtools ${AVRTOOLS_SRC_FILES})

add_avr_library(avrtools-a2d ${AVRTOOLS_A2D_SRC_FILES})
add_avr_library(avrtools-i2c ${AVRTOOLS_I2C_SRC_FILES})
add_avr_library(avrtools-spi ${AVRTOOLS_SPI_SRC_FILES})
add_avr_library(avrtools-ow ${AVRTOOLS_ONEWIRE_SRC_FILES})

add_avr_library(avrtools-usart0 ${AVRTOOLS_UART0_SRC_FILES})
add_avr_library(avrtools-usart0minimal ${AVRTOOLS_UART0_MINIMAL_SRC_FILES})


avr_target_include_common_directories(
        avrtools PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
)
avr_target_include_common_directories(
        avrtools-usart0 PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
)
message(STATUS "AVRTOOLS: ${CMAKE_CURRENT_SOURCE_DIR}")
message(">\tCurrent build type: ${CMAKE_BUILD_TYPE}")