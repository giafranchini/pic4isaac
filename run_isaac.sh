#! /bin/sh
set -e

SCRIPT_DIR=$( cd -- "$( dirname -- "$0" )" &> /dev/null && pwd )
SCRIPT_PATH="${SCRIPT_DIR}/scripts/main.py"

display_help() {
    echo "Launch this script inside the pic4isaac dir!"
    echo
    echo "Usage: $0 [-c | --config-file] {relative-path}" >&2
    echo
    echo "   -c, --config-file          Relative path to configuration file"
    echo
    exit 1
}

while :
do
    case "$1" in
      -h | --help)
            display_help
            exit 0
            ;;
      -c | --config-file)
            CONFIG_PATH="${SCRIPT_DIR}/$2"
            break
            ;;
      -*)
            echo "Error: Unknown option: $1" >&2
            echo
            display_help
            exit 1 
            ;;
      *)  
            break
            ;;
    esac
done

cd /isaac-sim
# echo $SCRIPT_PATH 
# echo $SCRIPT_ARG_NAME
# echo $SCRIPT_ARG_VALUE

./python.sh $SCRIPT_PATH --config-file $CONFIG_PATH