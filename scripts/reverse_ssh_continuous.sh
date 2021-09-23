#!/bin/sh

# $REMOTE_PORT is the remote port number that will be used to tunnel
# back to this system
REMOTE_PORT="100"

# $REMOTE_HOST is the name of the remote system
REMOTE_HOST="jongsoo@58.230.119.87 -p 9709"

for PORT in "22" "80"
do
   # $COMMAND is the command used to create the reverse ssh tunnel
   #COMMAND="autossh -f -N -R *:$REMOTE_PORT:localhost:22 $REMOTE_HOST"
   COMMAND="ssh -fN -R ${REMOTE_PORT}${PORT}:localhost:${PORT} $REMOTE_HOST"

   # Is the tunnel up? Perform two tests:

   # 1. Check for relevant process ($COMMAND)
   pgrep -f -x "$COMMAND" > /dev/null 2>&1 || $COMMAND

   # 2. Test tunnel by looking at "netstat" output on $REMOTE_HOST
   ssh $REMOTE_HOST netstat -an | egrep "tcp.*:${REMOTE_PORT}$PORT.*LISTEN" \
      > /dev/null 2>&1
   if [ $? -ne 0 ] ; then
      pkill -f -x "$COMMAND"
      $COMMAND
   fi
done
