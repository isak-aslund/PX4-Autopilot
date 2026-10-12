#!/bin/bash
# usage: sih.sh <repo root> <instance> -> runs SIH headless, log in $S/run_<instance>.log; KEEP=1 keeps the instance's parameters
S="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
B=$1/build/px4_sitl_default
D=$B/inst$2; [ -z "$KEEP" ] && rm -rf $D; mkdir -p $D; cd $D
export PX4_SIM_MODEL=sihsim_quadx PX4_SIM_SPEED_FACTOR=${3:-1} HEADLESS=1
exec $B/bin/px4 -i $2 -d $B/etc > $S/run_$2.log 2>&1
