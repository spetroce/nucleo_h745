#!/bin/bash

SCRIPT_DIR="$( dirname -- "$0"; )"

cp $SCRIPT_DIR/build/CM4/h745_test_cm4.hex /run/media/$USER/NOD_H745ZIQ/
# Need to wait for nucleo board to restart after first binary is copied. The
# quick and easy way to do this is just wait.
sleep 5
cp $SCRIPT_DIR/build/CM7/h745_test_cm7.hex /run/media/$USER/NOD_H745ZIQ/
