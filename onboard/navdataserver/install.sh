#!/bin/bash

SHORTPRESS2=scripts/shortpress_2.sh
# Used to configure shortpress_2 (connect the drone to existing wifi)
DRONE_ESSID=NWLdrone
DRONE_IP=10.42.0.10
DRONE_GW=10.42.0.1
DRONE_NETMASK=255.255.255.0

if [ $1 == "-h" ]; then
    echo "Usage: $0 [drone IP [drone GW [wifi SSID [drone NETMASK]]]]"
    echo "Default values are:"
    echo "IP = $DRONE_IP (you usually just have to replace the last number, avoiding 1)"
    echo "GW = $DRONE_GW"
    echo "SSID = $DRONE_ESSID "
    echo "NETMASK = $DRONE_NETMASK"
    exit 0
fi

echo "You will install an onboard extension in order to receive full navdata from the connected Bebop2"
echo "Please make sure you are currently connected to the bebop2 wifi"
echo "Tested with Bebop2 firmware 4.0"
echo "If you are not sure about the firmware version, you can follow the manual installation procedure"
echo -e "\nDrone wifi client mode will use this configuration:"

if [ "$1" ]; then DRONE_IP=$1;        fi
if [ "$2" ]; then DRONE_GW=$2;        fi
if [ "$3" ]; then DRONE_ESSID="$3";     fi
if [ "$4" ]; then DRONE_NETMASK=$4;   fi

echo -e "\tIP = $DRONE_IP"
echo -e "\tGW = $DRONE_GW"
echo -e "\tSSID = $DRONE_ESSID"
echo -e "\tNETMASK = $DRONE_NETMASK\n"

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

    echo "Configuring wifi..."

    sed -i '/ESSID=.*/{s/=.*/'"=${DRONE_ESSID}"'/};' $SHORTPRESS2
    sed -i '/IP=.*/{s/=.*/'"=${DRONE_IP}"'/};' $SHORTPRESS2
    sed -i '/GW=.*/{s/=.*/'"=${DRONE_GW}"'/};' $SHORTPRESS2
    sed -i '/NETMASK=.*/{s/=.*/'"=${DRONE_NETMASK}"'/};' $SHORTPRESS2

    echo "Uploading files to the drone..."
    ./uploadfiles.sh
    echo "Onboard installation..."
    { sleep 1; echo "sh /data/ftp/internal_000/navdataserver/onboard__install.sh"; sleep 1; } | telnet 192.168.42.1
    exit 0;
else
    echo "Installation aborted!"
fi

