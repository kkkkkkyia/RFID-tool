CC = gcc
CFLAGS = -Wall -g
LDFLAGS = -lmodbus -lm -lpigpio -lrt -lpthread
all:
	$(CC) $(CFLAGS) RfidRead.c -o RfidRead $(LDFLAGS)
clean:
	rm -f RfidRead