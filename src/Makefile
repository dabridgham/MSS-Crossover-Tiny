PORT=/dev/
#PROGRAMMER=usbasp-clone
$PROGRAMMER=snap_pdi
PROGRAMMER=snap_isp
MCU=attiny25
CFLAGS=-g -Wall -mcall-prologues -mmcu=$(MCU) -Os
CC=avr-gcc
TARGET=main
OBJECT_FILES=main.o
#AVRDUDE=/home/dab/src/avrdude/build_linux/src/avrdude
AVRDUDE=avrdude

all: $(TARGET).hex

%.hex: %.obj
	avr-objcopy -R .eeprom -O ihex $< $@

%.obj: $(OBJECT_FILES)
	$(CC) $(CFLAGS) $(OBJECT_FILES) $(LDFLAGS) -o $@

program: $(TARGET).hex
	$(AVRDUDE) -v -p $(MCU) -c $(PROGRAMMER) -U flash:w:$(TARGET).hex

