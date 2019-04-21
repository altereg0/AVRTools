#####################################################################
# simple AVR library
#####################################################################
set(AVRTOOLS_SRC_FILES
        abi.cpp
        InitSystem.cpp
        MemUtils.cpp
        new.cpp
        Pwm.cpp
        Reader.cpp
        RingBuffer.cpp
        SystemClock.cpp
        USART0.cpp
        USART0Minimal.cpp
        USART1.cpp
        USART1Minimal.cpp
        USART2.cpp
        USART2Minimal.cpp
        USART3.cpp
        USART3Minimal.cpp
        Writer.cpp
        )

set(AVRTOOLS_A2D_SRC_FILES
        Analog2Digital.cpp
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

add_avr_library(avrtools ${AVRTOOLS_SRC_FILES})

add_avr_library(avrtools-a2d ${AVRTOOLS_A2D_SRC_FILES})
add_avr_library(avrtools-i2c ${AVRTOOLS_I2C_SRC_FILES})
add_avr_library(avrtools-spi ${AVRTOOLS_SPI_SRC_FILES})
add_avr_library(avrtools-ow ${AVRTOOLS_ONEWIRE_SRC_FILES})