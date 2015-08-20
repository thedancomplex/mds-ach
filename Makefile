default: all

CFLAGS := -I./include -g -std=gnu99 
CC := gcc

INSTALL_DIR := /usr/bin
CONFIG_DIR := /etc/mds-ach
SIM_DIR := model/mds
PY_DIR := /usr/lib/python2.7/dist-packages
INCLUDE_DIR := /usr/include
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

makesim:
	rm -rf ${SIM_DIR}/build
	mkdir -p ${SIM_DIR}/build
	cmake -B${SIM_DIR}/build -H${SIM_DIR}
	make -C ${SIM_DIR}/build

install:
	mkdir -p /etc/mds-ach
	cp -r configs/ ${CONFIG_DIR}/
	cp -r python/ ${CONFIG_DIR}/
	cp -r model/ ${CONFIG_DIR}/
	cp ${BINARIES} ${INSTALL_DIR}
	cp scripts/mds-ach ${INSTALL_DIR}
	cp python/mds_ach.py ${PY_DIR}/
	cp python/mds_ik.py ${PY_DIR}/
	cp python/mds_ik_include.py ${PY_DIR}/
	cp include/mds.h ${INCLUDE_DIR}/mds.h

installsim:
	rm -f /home/$$USER/.gazebo/models/mds
	ln -s ${CONFIG_DIR}/${SIM_DIR} /home/$$USER/.gazebo/models/mds

rm:
	rm ${INSTALL_DIR}/mds-daemon
	rm ${INSTALL_DIR}/mds-filter
	rm -rf ${CONFIG_DIR}
	rm ${PY_DIR}/mds_ach.py
	rm ${PY_DIR}/mds_ik.py
	rm ${PY_DIR}/mds_ik_include.py
	rm ${INCLUDE_DIR}/mds.h
	
        
