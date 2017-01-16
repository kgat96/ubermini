CC=gcc
CFLAGS=-I. -Wall -I../libbtbb
LFLAGS=../libbtbb/libbtbb.a
OBJ = main.o

.PHONY: uber

%.o: %.c
	$(CC) -c -o $@ $< $(CFLAGS)

uber: $(OBJ)
	$(CC) -o $@ $^ $(CFLAGS) $(LFLAGS)

clean:
	rm *.o uber
