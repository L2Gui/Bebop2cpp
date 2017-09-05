#!/bin/sh

# *************************************************** SETTINGS
TAG=WifiSwap
ESSID=NWLdrone
DEFAULT_WIFI_SETUP=/sbin/broadcom_setup.sh
IP=10.42.0.10
GW=10.42.0.1
NETMASK=255.255.255.0

# *************************************************** SCRIPT
echo "2 times pressed..." | ulogger -t "$TAG" -p I

# Bip
BLDC_Test_Bench -M 1

if [ $(bcmwl ap) -eq 1 ]
then
        echo "Trying to connect to $ESSID" | logger -t "$TAG" -p I

        # Turning AccessPoint down
        $DEFAULT_WIFI_SETUP remove_net_interface

        sleep 1

        # Configure wifi to connect to an other AccessPoint
        ifconfig eth0 down
        bcmwl down
        bcmwl band auto
        bcmwl autocountry 1
        bcmwl up
        bcmwl ap 0
        bcmwl join ${ESSID}
        ifconfig eth0 $IP netmask $NETMASK up
        route add default gw $GW

        # DHCP client (get an ip address)
        # udhcpc -b -i eth0 --hostname=$(hostname)

        # Bip bip
        BLDC_Test_Bench -M 2
        sleep 1
        BLDC_Test_Bench -M 2
else
        echo "Trying to launch AP again" | logger -t "$TAG" -p

        # Might not work, reboot instead
        $DEFAULT_WIFI_SETUP create_net_interface

        # bip
        BLDC_Test_Bench -M 2
fi

echo "2 times pressed done" | ulogger -t "$TAG" -p I
