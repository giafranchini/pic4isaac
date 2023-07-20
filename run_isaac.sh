#! /bin/sh
set -e

SCRIPT_DIR=$( cd -- "$( dirname -- "$0" )" &> /dev/null && pwd )
SCRIPT_PATH="${SCRIPT_DIR}/$1"
SCRIPT_ARG_NAME=$2
SCRIPT_ARG_VALUE="${SCRIPT_DIR}/$3"

cd /isaac-sim
# echo $SCRIPT_PATH 
# echo $SCRIPT_ARG_NAME
# echo $SCRIPT_ARG_VALUE

./python.sh $SCRIPT_PATH $SCRIPT_ARG_NAME $SCRIPT_ARG_VALUE