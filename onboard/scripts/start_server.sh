#!/bin/bash

TAG="NAVDATASERVER"

echo "Removing previous blackboxes..." | ulogger -t "$TAG" -p I
rm /data/ftp/internal_000/Debug/archive/debug_*.tar.lzo
echo "Done." | ulogger -t "$TAG" -p I

echo "Launching navdata server..." | ulogger -t "$TAG" -p I
/data/ftp/internal_000/navdataserver/a.out &
echo "Done." | ulogger -t "$TAG" -p I

