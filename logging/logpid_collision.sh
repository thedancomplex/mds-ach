logpid() { while sleep 1; do  ps -p $1 -o pcpu= -o pmem= ; done; }

#logpid $(pidof gzserver) | tee gzserver.log
logpid $(pidof mds-collision-checker) | tee mds-collision-checker.log
