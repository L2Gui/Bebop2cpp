#!/bin/sh

# *************************************************** SCRIPT
echo "4 times pressed..." | ulogger -t "$TAG" -p I

echo "restarting the server" | logger -t "$TAG" -p I
killall -9 navdataserver

/data/ftp/internal_000/navdataserver/start_server.sh &
sleep 1