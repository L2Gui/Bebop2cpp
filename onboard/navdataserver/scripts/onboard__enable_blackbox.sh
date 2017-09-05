#!/bin/sh

DEBUG_CONF=/etc/debug.conf
DRAGON_CONF=/data/dragon.conf
RCS=/etc/init.d/rcS_mode_default

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
#GNU sed requires a parameter for -i
cp $DEBUG_CONF ${DEBUG_CONF}.$EXT

grep -q "BLACKBOX=[0-1]" $DEBUG_CONF
ret_code=$?
# if blackbox option is not present
if [ $ret_code != 0 ]; then
        echo "BLACKBOX option not found, adding it at the end of the file"
        echo $INFO_sta >> $DEBUG_CONF
        echo "# Enable blackbox" >> $DEBUG_CONF
        echo "BLACKBOX=1" >> $DEBUG_CONF
        echo $INFO_end >> $DEBUG_CONF
else
    sed -i '/BLACKBOX=[0-1]/{s//BLACKBOX=1/};' $DEBUG_CONF
fi

# Making sure blackbox is now enabled
grep -q "BLACKBOX=1" $DEBUG_CONF
if [ $? == 0 ]; then
    echo "OK"
else
    echo "Something went wrong, abort"
    cp ${DEBUG_CONF}.$EXT $DEBUG_CONF
    exit 1
fi

# DRAGON.CONF
# may not be needed but some say it is, so it probably depends on firmware version
echo "$DRAGON_CONF ..."
cp $DRAGON_CONF ${DRAGON_CONF}.$EXT

grep -q "\"blackbox_enable\" : \(false\|true\)" $DRAGON_CONF
ret_code=$?
# if blackbox option is not present
if [ $ret_code != 0 ]; then
    echo "blackbox_enable option not found, adding it to the file"
    sed -i '/\"absolute_control\"\ :\ \(false\|true\),/{s/,/,\n\t\t\"blackbox_enable\"\ :\ true,/}' $DRAGON_CONF
    ret_code=$?
    if [ $ret_code != 0 ]; then
        echo "ERROR during the configuration of $DRAGON_CONF"
        exit $ret_code
    fi
else
    sed -i '/\"blackbox_enable\"\ :\ \(false\|true\)/{s/false/true/}' $DRAGON_CONF
fi

# Making sure blackbox is now enabled
grep -q "\"blackbox_enable\" : true" $DRAGON_CONF
if [ $? == 0 ]; then
    echo "OK"
else
    echo "Something went wrong, abort"
    cp ${DRAGON_CONF}.$EXT $DRAGON_CONF
    exit 1
fi

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

echo "OK"

