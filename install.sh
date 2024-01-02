#!/bin/bash
# Constants
EXTENSION_NAME="roadrunner-filament-sensor"

# Force script to exit if an error occurs
set -e

# Find SRCDIR from the pathname of this script
SRCDIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )"/ && pwd )"

KLIPPER_PATH="${HOME}/klipper"
PYTHONDIR="${HOME}/klippy-env"
# Parse command line arguments to allow KLIPPER_PATH override
while getopts "k:e:" arg; do
    case $arg in
        k) KLIPPER_PATH=$OPTARG;;
        e) PYTHONDIR=$OPTARG;;
    esac
done

# Verify conditions for the install to take place
check_preconditions()
{
    if [ "$EUID" -eq 0 ]; then
        echo "This script must not run as root"
        exit -1
    fi

    if [ "$(sudo systemctl list-units --full -all -t service --no-legend | grep -F "klipper.service")" ]; then
        echo "Klipper service found!"
    else
        echo "Klipper service not found, please install Klipper first"
        exit -1
    fi
}

# create a symlinks to the extension files in the klippy/extras directory
link_extension()
{
    echo "Linking ${EXTENSION_NAME} to Klippy extras..."
    ln -sf ${SRCDIR}/klippy/extras/*.py ${KLIPPER_PATH}/klippy/extras/
}

# install python requirements to klippy env
install_requirements()
{
    echo "Updating python virtual environment..."

    # Create virtualenv if it doesn't already exist
    [ ! -d ${PYTHONDIR} ] && echo "klipper env not found at ${PYTHONDIR}" && exit -1

    # Install/update dependencies
    ${PYTHONDIR}/bin/pip install -r ${SRCDIR}/klippy-requirements.txt

}

# restarting Klipper
restart_klipper()
{
    echo "Restarting Klipper..."
    sudo systemctl restart klipper
}

# Installation steps:
check_preconditions
link_extension
install_requirements
restart_klipper
