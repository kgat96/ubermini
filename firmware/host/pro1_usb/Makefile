CC=gcc
CFLAGS=-I. -Wall -I/usr/include/libusb-1.0  -lusb-1.0
OBJ = main.o

.PHONY: uber

%.o: %.c 
	$(CC) -c -o $@ $< $(CFLAGS)

uber: $(OBJ)
	$(CC) -o $@ $^ $(CFLAGS)

clean:
	rm *.o uber
