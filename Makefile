BOARD_TAG    = pro328
ARDUINO_PORT = /dev/ttyUSB*
USER_LIB_PATH := $(realpath ./libraries)
ARDUINO_LIBS = DHT22 Wire DS3231 CRC16

include ./libraries/Arduino-Makefile/arduino-mk/Arduino.mk

