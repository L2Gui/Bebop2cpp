#!/bin/bash

DEBUG_CONF=debug.conf
DRAGON_CONF=dragon.conf
RCS=rcS_mode_default

INFO_sta="# ADDED BY BEBOP2CPP v"
INFO_end="# ADDED BY BEBOP2CPP ^"
EXT=orig

STARTSERVER="/data/ftp/internal_000/navdataserver/start_server.sh"

if [ ! -f $DEBUG_CONF ]; then
    echo "$DEBUG_CONF is missing!"
    exit 1
fi

if [ ! -f $DRAGON_CONF ]; then
    echo "$DRAGON_CONF is missing!"
    exit 1
fi

if [ ! -f $RCS ]; then
    echo "$RCS is missing!"
    exit 1
fi

# DEBUG.CONF
echo "$DEBUG_CONF ..."
sed -i.$EXT '/BLACKBOX=[0-1]/{s//BLACKBOX=1/;h};${x;/./{x;q0};x;q1}' $DEBUG_CONF

ret_code=$?

if [ $ret_code != 0 ]; then
    echo "BLACKBOX option not found, adding it at the end of the file"
    echo $INFO_sta >> $DEBUG_CONF
    echo "# Enable blackbox" >> $DEBUG_CONF
    echo "BLACKBOX=1" >> $DEBUG_CONF
    echo $INFO_end >> $DEBUG_CONF
fi

echo "$DEBUG_CONF OK"

# DRAGON.CONF

echo "$DRAGON_CONF ..."
sed -i.$EXT '/\"blackbox_enable\"\ :\ \(false\|true\)/{s/false/true/;h};${x;/./{x;q0};x;q1}' $DRAGON_CONF

ret_code=$?

if [ $ret_code != 0 ]; then
    echo "blackbox_enable option not found, adding it to the file"
    sed -i.$EXT '/\"absolute_control\"\ :\ \(false\|true\),/{s/,/,\n\t\t\"blackbox_enable\"\ :\ true,/;h};${x;/./{x;q0};x;q1}' $DRAGON_CONF
    ret_code=$?
    if [ $ret_code != 0 ]; then
        echo "ERROR during the configuration of $DRAGON_CONF"
        exit $ret_code
    fi
fi

echo "$DRAGON_CONF OK"

# Launch at startup

echo "$RCS ..."
grep -q $STARTSERVER $RCS

ret_code=$?

if [ $ret_code != 0 ]; then
    echo "Adding automatic navdataserver startup"
    cp $RCS ${RCS}.$EXT
    echo $INFO_sta >> $RCS
    echo "$STARTSERVER &" >> $RCS
    echo "sleep 1" >> $RCS
    echo $INFO_end >> $RCS
fi

echo "$RCS OK"

