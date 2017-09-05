#!/bin/bash

ftp -n <<EOF  > /dev/null 2>&1
open 192.168.42.1
user user
mkdir internal_000/navdataserver
put scripts/onboard__enable_blackbox.sh internal_000/navdataserver/onboard__enable_blackbox.sh
put scripts/onboard__install.sh internal_000/navdataserver/onboard__install.sh
put scripts/shortpress_2.sh internal_000/navdataserver/shortpress_2.sh
put scripts/shortpress_6.sh internal_000/navdataserver/shortpress_6.sh
put scripts/start_server.sh internal_000/navdataserver/start_server.sh
put navdataserver internal_000/navdataserver/navdataserver
EOF
