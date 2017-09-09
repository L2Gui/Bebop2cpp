# Bebop Navdata server

Parrot sdk retransmits few data, and at 5Hz. This program will retransmit full navigation data at 100Hz. It uses ~80kB/s of bandwidth.

## Automatic installation

It is recommended to run the automated installer.

### Linux

Just ``cd`` into this directory and run ``./install.sh``.
Usage:
```
$ ./install.sh -h
Usage: ./install.sh [drone IP [drone GW [wifi SSID [drone NETMASK]]]]
Default values are:
IP = 10.42.0.10 (you usually just have to replace the last number, avoiding 1)
GW = 10.42.0.1
SSID = NWLdrone 
NETMASK = 255.255.255.0
```

Then reboot your drone and it should be done.

### Windows

TODO

## Manual installation

Please read ``MANUAL_INSTALL.md``.

## Copyright
src/navdataserver.c is originally under MIT licence at [lukaslaobeyer/libdrone](https://github.com/lukaslaobeyer/libdrone/tree/master/bebop-onboard/navdataserver) for bebop1.
