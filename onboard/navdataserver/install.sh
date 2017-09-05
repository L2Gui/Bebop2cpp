#!/bin/bash


echo "You will install an onboard extension in order to receive full navdata from the connected Bebop2"
echo "Please make sure you are currently connected to the bebop2 wifi or provide the relevant address when asked"
echo "Tested with Bebop2 firmware 4.0"
echo "If you are not sure about the firmware version, you can follow the manual installation procedure"
read -p "Continue? (y/N) "

echo ""

if [[ $REPLY =~ ^[Yy]$ ]]; then

    echo "Testing connexion..."
    nc -z -w2 192.168.42.1 21
    if [ $? != 0 ]; then
        echo -e "---\nYou are not connected to the drone, please connect to the drone."
        exit 1
    fi
    nc -z -w2 192.168.42.1 23
    if [ $? != 0 ]; then
        echo -e "---\nTelnet is not enabled on the drone, please quickly press the drone button 4 times and try again."
        exit 1
    fi

    echo "Compiling server..."
    ./compile.sh
    echo "Uploading files to the drone..."
    ./uploadfiles.sh
    echo "Onboard installation..."
    { sleep 1; echo "sh /data/ftp/internal_000/navdataserver/onboard__install.sh"; sleep 1; } | telnet 192.168.42.1
    exit 0;
else
    echo "Installation aborted!"
fi

