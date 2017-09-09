# Installing the Bebop2 navdata server

The program will retransmit the full navigation (extacted from the blackbox) at 100Hz.

Cross-compile ``src/navdataserver.c`` with ``./compile.sh``. You will need the ``gcc-arm-linux-gnueabi`` utility.

## 1. Telnet into the Bebop drone

The Bebop has a telnet server that can be activated by pressing the drone's button 4 short times.
After that, you will be able to connect to the drone.

```
telnet 192.168.42.1
```

Activate writing to the filesystem:
```
mount -o remount,rw /
```

## 2. Activate debug mode

``/etc/debug.conf``: Change ``BLACKBOX=0`` to ``BLACKBOX=1`` (if not present, just add it)

``/data/dragon.conf``: Change ``"blackbox_enable" : false`` to ``"blackbox_enable" : true`` (if not present, just add it)

## 3. Upload the navdata server

```
cd /data/ftp/internal_000/
mkdir navdataserver
```

Now copy the content of ``scripts`` into the ``/data/ftp/internal_000/navdataserver`` directory using FTP.

Mark ``navdataserver`` and ``start_server.sh`` as executable:
```
chmod +x /data/ftp/internal_000/navdataserver/navdataserver
chmod +x /data/ftp/internal_000/navdataserver/start_server.sh
```

## 4. Autostart on Bebop boot

edit ``/etc/init.d/rcS_mode_default``

Insert "/data/ftp/internal_000/navdataserver/start_server.sh &" at the end of the file (before the sleep 1 to speed up the boot time)

## 5. Enable wifi client mode

Edit the file ``shortpress_2.sh`` and modify the wifi configuration values if needed
```
ESSID=NWLdrone
IP=10.42.0.11
GW=10.42.0.1
NETMASK=255.255.255.0
```

If using several drones, you must give each drone a distinct IP address.

Mark as executable and copy the file to ``/bin/onoffbutton/``

```
chmod +x /data/ftp/internal_000/navdataserver/shortpress_2.sh
cp /data/ftp/internal_000/navdataserver/shortpress_2.sh /bin/onoffbutton/
```

## 6. Enable navdataserver reset

Mark the file ``shortpress_6.sh`` as executable and copy it to ``/bin/onoffbutton/``

```
chmod +x /data/ftp/internal_000/navdataserver/shortpress_6.sh
cp /data/ftp/internal_000/navdataserver/shortpress_6.sh /bin/onoffbutton/
```
## 7. Reboot

Done.
