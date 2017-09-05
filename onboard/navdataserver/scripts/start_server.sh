#!/bin/sh

TAG="NAVDATASERVER"

BLDC_Test_Bench -M 1
sleep 1
BLDC_Test_Bench -M 1

# waiting for dragon to initialise
sleep 5


echo "Removing previous blackboxes..." | ulogger -t "$TAG" -p I
rm -f /data/ftp/internal_000/Debug/archive/debug_*.tar.lzo
echo "Done." | ulogger -t "$TAG" -p I

echo "Launching navdata server..." | ulogger -t "$TAG" -p I
/data/ftp/internal_000/navdataserver/navdataserver &
echo "Done." | ulogger -t "$TAG" -p I


BLDC_Test_Bench -M 2
sleep 1
BLDC_Test_Bench -M 2
