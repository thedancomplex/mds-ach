default: all

CFLAGS := -I./include -g -std=gnu99 
CC := gcc

INSTALL_DIR := /usr/bin
CONFIG_DIR := /etc/mds-ach

BINARIES := mds-daemon mds-filter
#BINARIES := mds-daemon mds-can-daemon
all : $(BINARIES)

LIBS := -lach -lrt -lm

mds-daemon: src/mds-daemon.o
	$(CC) -o $@ $< $(LIBS)

mds-filter: src/mds-filter.o
	$(CC) -o $@ $< $(LIBS)

#mds-can-daemon: src/mds-can-daemon.o
#	gcc -o $@ $< $(LIBS)

%.o: %.c
	$(CC) $(CFLAGS) -o $@ -c $<

clean:
	rm -f $(BINARIES) src/*.o

install:
	mkdir -p /etc/mds-ach
	cp -r configs/ ${CONFIG_DIR}/
	cp -r python/ ${CONFIG_DIR}/
	cp ${BINARIES} ${INSTALL_DIR}
	cp scripts/mds-ach ${INSTALL_DIR}

rm:
	rm ${INSTALL_DIR}/mds-daemon
	rm ${INSTALL_DIR}/mds-filter
	rm -rf ${CONFIG_DIR}
	
        
