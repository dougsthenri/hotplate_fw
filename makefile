# Requisitos:
# - avrdude
# - avr-libc

SRC = \
	main.c

DEVICE = /dev/tty.usbserial-AH01A6Q8

CC = avr-gcc
CFLAGS = -Wall -Wextra -Wno-main -Os -ffunction-sections -fdata-sections -Wl,--gc-sections -mmcu=atmega328p -std=c11
DFLAGS = -DF_CPU=16000000L
LDFLAGS = -lm

OC = avr-objcopy
OCFLAGS = -O ihex -R .eeprom


# Compilar o código-fonte
all: $(SRC)
	$(CC) $(CFLAGS) $(DFLAGS) $(SRC) -o firmware.elf
	$(OC) $(OCFLAGS) firmware.elf firmware.hex

# Programar o Arduino nano
install:
	avrdude -v -D -p atmega328p -c arduino -b 57600 -P "$(DEVICE)" -U flash:w:"firmware.hex":i

clean:
	rm firmware.hex firmware.elf
