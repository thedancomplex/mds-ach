default: all

CFLAGS := -I./include -g -std=gnu99 
CC := gcc

BINARIES := mds-simple-demo 
#BINARIES := mds-daemon mds-can-daemon
all : $(BINARIES)

LIBS := -lach -lrt -lm

mds-simple-demo: src/mds-simple-demo.o
	$(CC) -o $@ $< $(LIBS)

#mds-can-daemon: src/mds-can-daemon.o
#	gcc -o $@ $< $(LIBS)

%.o: %.c
	$(CC) $(CFLAGS) -o $@ -c $<

clean:
	rm -f $(BINARIES) src/*.o
