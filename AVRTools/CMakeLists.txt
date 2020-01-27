#####################################################################
# AVRTools library
#####################################################################
add_compile_definitions( SERIAL_OUTPUT_EOL=\"\\r\\n\" )
add_compile_definitions( AVRTOOLS_RESET_MCU )

set( AVRTOOLS_SRC_FILES
        abi.cpp
        Analog2Digital.cpp
        I2cLcd.cpp
        I2cMaster.cpp
        I2cSlave.cpp
        InitSystem.cpp
        MemUtils.cpp
        new.cpp
        OneWire.cpp
        Pwm.cpp
        Reader.cpp
        RingBuffer.cpp
        SPI.cpp
        SystemClock.cpp
        USART0.cpp
        USART0Minimal.cpp
#        USART1.cpp
#        USART1Minimal.cpp
#        USART2.cpp
#        USART2Minimal.cpp
#        USART3.cpp
#        USART3Minimal.cpp
        Writer.cpp
        WString.cpp
        )

add_avr_library( avrtools ${AVRTOOLS_SRC_FILES} )

set( AVRTOOLS_RINGBUFFER_TYPE_SRC_FILES
        RingBufferT.h
        )
add_avr_interface_library( avrtools-ringbuffer-type ${AVRTOOLS_RINGBUFFER_TYPE_SRC_FILES} )


avr_target_include_common_directories(
        avrtools PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
)

message( STATUS "AVRTOOLS: ${CMAKE_CURRENT_SOURCE_DIR}" )
message( ">\tCurrent build type: ${CMAKE_BUILD_TYPE}" )
#####################################################################
# WString library
#####################################################################

set( WSTRING_SRC_FILES
        WString.cpp
        )

add_avr_library( wstring ${WSTRING_SRC_FILES} )
avr_target_include_common_directories(
        wstring PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
)
message( STATUS "WSTRING library: ${CMAKE_CURRENT_SOURCE_DIR}" )
message( ">\tCurrent build type: ${CMAKE_BUILD_TYPE}" )

#####################################################################
# DRIVERS
#####################################################################
set( AVRTOOLS_DRIVERS_SRC_FILES
        drivers/DS1307RTC.cpp
        drivers/tm1637.cpp
        drivers/TroykaDHT.cpp
        )

add_avr_library( avrtools-drivers ${AVRTOOLS_DRIVERS_SRC_FILES} )
avr_target_link_libraries(
        avrtools-drivers
        avrtools
)