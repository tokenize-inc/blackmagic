SYSTEM_INCLUDE_PATHS=$(shell pkg-config --silence-errors --cflags libusb-1.0 libftdi1)
SYSTEM_LIBRARY_PATHS=$(shell pkg-config --silence-errors --libs libusb-1.0 libftdi1)

swolisten: swolisten.c
	gcc swolisten.c -o swolisten ${SYSTEM_INCLUDE_PATHS} ${SYSTEM_LIBRARY_PATHS}