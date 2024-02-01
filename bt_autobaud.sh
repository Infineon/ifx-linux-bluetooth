#!/bin/bash
#####################################################################
#
# Script to take chip in the autobaud mode
#
# v1.1
#####################################################################

echo -e "\nMake chip in the autobaud mode"

SCRIPTS_PATH="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"


echo "script:"
echo -e $SCRIPTS_PATH



#echo -e "\tCTS Low"
#source ${SCRIPTS_PATH}/bt_uart_cts.sh 0
sleep 0.5

# Toggle BT_REG_ON (reset chip)
echo -e "\tBT Power Off and On"
#source ${SCRIPTS_PATH}/bt_reg_onoff.sh 1
source ${SCRIPTS_PATH}/bt_reg_onoff.sh &
#source ${SCRIPTS_PATH}/bt_reg_onoff.sh 0

#sleep 0.5

