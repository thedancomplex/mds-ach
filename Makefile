default: all


CFLAGS := -I./include -g -std=gnu99 
CC := gcc

BINARIES := mds-daemon 
#BINARIES := mds-daemon mds-can-daemon
all : $(BINARIES)

LIBS := -lach -lrt -lm

mds-daemon: src/mds-daemon.o
	gcc -o $@ $< $(LIBS)

#mds-can-daemon: src/mds-can-daemon.o
#	gcc -o $@ $< $(LIBS)

%.o: %.c
	$(CC) $(CFLAGS) -o $@ -c $<

clean:
	rm -f $(BINARIES) src/*.o
