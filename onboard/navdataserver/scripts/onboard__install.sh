#!/bin/sh

safe_run(){
    echo -e "  -- exec \"$1\"..."
    eval $1
    ret=$?
   

    if [ $ret != 0 ]; then
        echo "Error $ret, abort"
        exit 1
    else
        echo "  -- done"
    fi
    return $ret
}

BLDC_Test_Bench -M 1 > /dev/null 2>&1

NAVDATAFOLDER="/data/ftp/internal_000/navdataserver/"

navdataserver_name="navdataserver"
blackbox_name="onboard__enable_blackbox.sh"
wifiswap_name="shortpress_2.sh"
restartserver_name="shortpress_6.sh"
start_server_name="start_server.sh"


navdataserver_path="${NAVDATAFOLDER}${navdataserver_name}"
blackbox_path="${NAVDATAFOLDER}${blackbox_name}"
wifiswap_path="${NAVDATAFOLDER}${wifiswap_name}"
restartserver_path="${NAVDATAFOLDER}${restartserver_name}"
start_server_path="${NAVDATAFOLDER}${start_server_name}"

BUTTONFOLDER="/bin/onoffbutton/"

# Checking files
FILES_OK=1
echo "-- Checking uploaded files..."
for FILE in $navdataserver_path $blackbox_path $wifiswap_path $start_server_path $restartserver_path
do
    echo -ne "\t$FILE: "
    if [ -f $FILE ]; then
        echo "ok"
    else
        echo "missing!"
        FILES_OK=0
    fi
done

if [ $FILES_OK == 0 ]; then
    echo "Some files are missing. try to send files again"
    exit 1
fi
echo -e "-- done\n"

# Installation of the files
echo "-- Onboard installation running..."

safe_run "mount -o remount,rw /"

safe_run "cp $wifiswap_path ${BUTTONFOLDER}$wifiswap_name"
safe_run "chmod +x ${BUTTONFOLDER}$wifiswap_name"

safe_run "cp $restartserver_path ${BUTTONFOLDER}$restartserver_name"
safe_run "chmod +x ${BUTTONFOLDER}$restartserver_name"

safe_run "sh $blackbox_path"

safe_run "chmod +x $start_server_path"
safe_run "chmod +x $navdataserver_path"

echo "-- done"

BLDC_Test_Bench -M 2 > /dev/null 2>&1

echo -e "\n\nPlease reboot your drone\n\n"
