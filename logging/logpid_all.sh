logpid() { 
   while sleep 1; do  
      for var in "$@"
      do
         ps -p $var -o command= -o etime= -o pcpu= -o pmem= ; 
      done

#         ps -p $1 -o ucmd= -o etime= -o pcpu= -o pmem= ; 
#         ps -p $2 -o ucmd= -o etime= -o pcpu= -o pmem= ; 
#         ps -p $3 -o ucmd= -o etime= -o pcpu= -o pmem= ; 
#         ps -p $4 -o ucmd= -o etime= -o pcpu= -o pmem= ; 
  done; }

MDS_HOME_DIR='/etc/mds-ach'
MDS_PID_FILE='mds-pid.log'
logpid $(cat $MDS_HOME_DIR/$MDS_PID_FILE) | tee mds-ach_all.log
#logpid $(pidof mds-daemon) $(pidof mds-filter) $(pidof gzserver) $(pidof mds-collision-checker) | tee mds-ach_all.log
