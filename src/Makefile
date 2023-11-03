PORT=/dev/
PROGRAMMER=usbasp-clone
MCU=attiny25
CFLAGS=-g -Wall -mcall-prologues -mmcu=$(MCU) -Os
CC=avr-gcc
TARGET=main
OBJECT_FILES=main.o

all: $(TARGET).hex

%.hex: %.obj
	avr-objcopy -R .eeprom -O ihex $< $@

%.obj: $(OBJECT_FILES)
	$(CC) $(CFLAGS) $(OBJECT_FILES) $(LDFLAGS) -o $@

program: $(TARGET).hex
	avrdude -p $(MCU) -c $(PROGRAMMER) -U flash:w:$(TARGET).hex

